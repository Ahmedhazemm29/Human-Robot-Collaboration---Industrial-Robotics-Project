// reach_location_server.cpp
//
// UR5e ReachLocation action server with predictive collision response.
//
// State machine (per goal):
//   PLAN_TO_GOAL ──▶ EXECUTE_TO_GOAL ──▶ DONE_OK
//                          │ (predicted collision)
//                          ▼
//                        BRAKE  ── (in parallel) ── Phase 3 candidate scoring
//                          │
//                          ▼
//                   MOVE_TO_SAFE_POINT ── (in parallel) ── Phase 5 goal pre-plan
//                          │
//                          ▼
//                    AT_SAFE_POINT ──▶ EXECUTE_TO_GOAL  (preferred)
//                          │
//                          ▼
//                    HOLD_AND_WAIT  (last resort)
//
// Phase 1 — a dedicated monitor thread runs while execute() drives a
// trajectory. Every monitor_period seconds it samples the next
// lookahead_window of waypoints of the in-flight trajectory and asks
// /check_state_validity whether they collide with the latest scene
// (hand included). One predicted collision flips collision_predicted_
// and interrupts execute() so the main thread sees it.
//
// Phase 2 — when interrupted, the main thread immediately builds a short
// braking trajectory (linear deceleration from current joint velocities
// to zero over braking_duration) and sends it via execute(). The
// controller follows it smoothly to a controlled rest.
//
// Phase 3 — while the brake is being executed, an async task scores
// Cartesian safe-point candidates around the rest pose. Candidates that
// fail IK, sit closer than clearance_margin to the hand OBB, or imply a
// near-singular Jacobian (condition number > threshold) are rejected.
// Remaining ones are ranked by a weighted sum of joint displacement,
// conditioning, and clearance.
//
// Phase 4 — best candidate is reached via OMPL (free-space, no Cartesian
// straight-line) with conservative velocity/acceleration scaling.
//
// Phase 5 — during that move, an independent MoveGroupInterface plans
// from the candidate joint state to the original goal, so the robot
// arrives with a fresh plan already in hand.
//
// Phase 6 — on arrival, the pre-plan is launched immediately if the
// scene is clear. Otherwise we replan from the safe point at short
// intervals, then try the next safe candidate. Only after every
// candidate is exhausted do we fall into HOLD_AND_WAIT, replanning
// every hold_replan_interval seconds while the hand is in the scene.
//
// The existing Cartesian-first / OMPL-fallback logic is preserved for
// the normal (no-collision) PLAN_TO_GOAL path.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <deque>
#include <shape_msgs/msg/solid_primitive.hpp>
#include "bt_action_server/action/reach_location.hpp"

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace
{

// Minimal oriented bounding box for the hand collision primitive. Stored
// once per /planning_scene update so candidate scoring doesn't need a
// service call for every clearance check.
struct HandBox
{
    bool valid = false;
    Eigen::Vector3d center{0.0, 0.0, 0.0};
    Eigen::Vector3d half_extents{0.0, 0.0, 0.0};
    Eigen::Quaterniond orientation{1.0, 0.0, 0.0, 0.0};
};

double distance_point_to_obb(const Eigen::Vector3d & p, const HandBox & box)
{
    if (!box.valid) return std::numeric_limits<double>::infinity();
    const Eigen::Vector3d local = box.orientation.conjugate() * (p - box.center);
    Eigen::Vector3d d;
    for (int i = 0; i < 3; ++i) {
        d[i] = std::max(0.0, std::abs(local[i]) - box.half_extents[i]);
    }
    return d.norm();
}

// ── Waypoint pose loaded from table_corners.json ─────────────────────────────
struct WaypointPose
{
    std::string                  name;
    geometry_msgs::msg::Pose     pose;
};

// ── Minimal JSON helpers (no external deps) ───────────────────────────────────
// Finds the JSON object body for a named key, e.g. "TOP_LEFT": { … }
static std::string json_find_object(const std::string & src,
                                     const std::string & key)
{
    const std::string needle = "\"" + key + "\"";
    auto pos = src.find(needle);
    if (pos == std::string::npos) return {};
    pos = src.find('{', pos + needle.size());
    if (pos == std::string::npos) return {};
    int depth = 1;
    size_t i  = pos + 1;
    while (i < src.size() && depth > 0) {
        if      (src[i] == '{') ++depth;
        else if (src[i] == '}') --depth;
        ++i;
    }
    if (depth != 0) return {};
    return src.substr(pos, i - pos);
}

// Extracts a JSON array of doubles for a named key within a JSON snippet.
static bool json_extract_array(const std::string & src,
                                const std::string & key,
                                std::vector<double> & out)
{
    const std::string needle = "\"" + key + "\"";
    auto pos = src.find(needle);
    if (pos == std::string::npos) return false;
    pos = src.find('[', pos + needle.size());
    if (pos == std::string::npos) return false;
    const auto end = src.find(']', pos);
    if (end == std::string::npos) return false;

    std::istringstream iss(src.substr(pos + 1, end - pos - 1));
    std::string tok;
    while (std::getline(iss, tok, ',')) {
        tok.erase(0, tok.find_first_not_of(" \t\n\r"));
        if (tok.empty()) continue;
        try { out.push_back(std::stod(tok)); }
        catch (...) { return false; }
    }
    return !out.empty();
}

}  // namespace

class ReachLocationActionServer : public rclcpp::Node
{
public:
    using ReachLocation = bt_action_server::action::ReachLocation;
    using GoalHandle    = rclcpp_action::ServerGoalHandle<ReachLocation>;
    using Plan          = moveit::planning_interface::MoveGroupInterface::Plan;

    explicit ReachLocationActionServer()
    : Node("reach_location_action_server",
           rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        // ── Tunable parameters ────────────────────────────────────────────
        lookahead_window_s_     = get_param<double>("lookahead_window",       1.5);
        braking_duration_s_     = get_param<double>("braking_duration",       0.4);
        clearance_margin_m_     = get_param<double>("clearance_margin",       0.10);
        max_safe_candidates_    = get_param<int>   ("max_safe_candidates",    16);
        max_safe_attempts_      = get_param<int>   ("max_safe_attempts",      3);
        vel_scale_normal_       = get_param<double>("vel_scale_normal",       0.25);
        acc_scale_normal_       = get_param<double>("acc_scale_normal",       0.25);
        vel_scale_safe_move_    = get_param<double>("vel_scale_safe_move",    0.15);
        acc_scale_safe_move_    = get_param<double>("acc_scale_safe_move",    0.15);
        vel_scale_caution_      = get_param<double>("vel_scale_caution",      0.08);
        acc_scale_caution_      = get_param<double>("acc_scale_caution",      0.08);
        caution_distance_m_     = get_param<double>("caution_distance",       0.25);
        monitor_period_s_       = get_param<double>("monitor_period",         0.10);
        safe_point_retreat_m_   = get_param<double>("safe_point_retreat",     0.15);
        safe_point_lift_m_      = get_param<double>("safe_point_lift",        0.15);
        condition_number_max_   = get_param<double>("condition_number_max",   40.0);
        hold_replan_interval_s_ = get_param<double>("hold_replan_interval",   1.0);
        validity_stride_        = get_param<int>   ("validity_check_stride",  3);
        planning_time_s_        = get_param<double>("planning_time",          5.0);
        enable_waypoint_fallback_ = get_param<bool>("enable_waypoint_fallback", true);

        {
            const char * home_c = std::getenv("HOME");
            const std::string home = home_c ? home_c : "/home/hazem";
            const std::string default_file = home + "/table_corners.json";
            const std::string wp_file =
                get_param<std::string>("waypoints_file", default_file);
            load_waypoints(wp_file);
        }

        action_server_ = rclcpp_action::create_server<ReachLocation>(
            this, "reach_location",
            std::bind(&ReachLocationActionServer::handle_goal,     this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&ReachLocationActionServer::handle_cancel,   this,
                      std::placeholders::_1),
            std::bind(&ReachLocationActionServer::handle_accepted, this,
                      std::placeholders::_1));

        monitor_cb_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = monitor_cb_group_;

        planning_scene_sub_ = this->create_subscription<moveit_msgs::msg::PlanningScene>(
            "/planning_scene", rclcpp::QoS(10),
            std::bind(&ReachLocationActionServer::planning_scene_cb, this,
                      std::placeholders::_1),
            sub_opts);


        RCLCPP_INFO(this->get_logger(),
            "ReachLocation ready (lookahead=%.2fs brake=%.2fs clearance=%.2fm "
            "v_norm=%.2f v_safe=%.2f v_caution=%.2f caution_dist=%.2fm "
            "waypoints=%d fallback=%s).",
            lookahead_window_s_, braking_duration_s_, clearance_margin_m_,
            vel_scale_normal_, vel_scale_safe_move_, vel_scale_caution_,
            caution_distance_m_,
            static_cast<int>(waypoints_.size()),
            enable_waypoint_fallback_ ? "ON" : "OFF");
    }

    ~ReachLocationActionServer() override
    {
        monitor_run_.store(false);
        if (monitor_thread_.joinable()) monitor_thread_.join();
    }

private:
    template <typename T>
    T get_param(const std::string & name, const T & default_value)
    {
        if (!this->has_parameter(name)) {
            this->declare_parameter(name, default_value);
        }
        return this->get_parameter(name).get_value<T>();
    }

    static constexpr const char * kHandObjectId  = "hand_exclusion_zone";
    static constexpr const char * kPlanningGroup = "ur_manipulator";
    static constexpr int    kMaxReplanAttempts   = 3;

    // Links checked in direct FK collision detection (origin vs hand OBB)
    static constexpr const char * kCheckedLinks[] = {
        "upper_arm_link", "forearm_link",
        "wrist_1_link", "wrist_2_link", "wrist_3_link", "flange"
    };
    static constexpr size_t kHandHistorySize = 6;
    static constexpr double kCartesianEefStep       = 0.005;
    static constexpr double kCartesianJumpThreshold = 0.0;
    static constexpr double kCartesianMinFraction   = 0.90;

    // Parameters.
    double lookahead_window_s_;
    double braking_duration_s_;
    double clearance_margin_m_;
    int    max_safe_candidates_;
    int    max_safe_attempts_;
    double vel_scale_normal_;
    double acc_scale_normal_;
    double vel_scale_safe_move_;
    double acc_scale_safe_move_;
    double vel_scale_caution_;
    double acc_scale_caution_;
    double caution_distance_m_;
    double monitor_period_s_;
    double safe_point_retreat_m_;
    double safe_point_lift_m_;
    double condition_number_max_;
    double hold_replan_interval_s_;
    int    validity_stride_;
    double planning_time_s_;

    // Waypoint fallback.
    bool                      enable_waypoint_fallback_;
    std::vector<WaypointPose> waypoints_;

    // ROS interfaces.
    rclcpp_action::Server<ReachLocation>::SharedPtr action_server_;
    rclcpp::CallbackGroup::SharedPtr monitor_cb_group_;
    rclcpp::Subscription<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_sub_;
    // Robot model stored once for direct FK collision checking
    moveit::core::RobotModelConstPtr robot_model_;

    // Cross-thread state.
    std::atomic<bool> is_executing_       {false};
    std::atomic<bool> hand_detected_      {false};
    std::atomic<bool> collision_predicted_{false};
    std::atomic<bool> caution_triggered_  {false};
    std::atomic<bool> caution_active_     {false};
    std::atomic<bool> caution_cleared_    {false};
    std::atomic<bool> monitor_run_        {false};

    std::mutex move_group_mutex_;
    moveit::planning_interface::MoveGroupInterface * active_move_group_{nullptr};

    // Plan currently being executed + wall-clock time execute() started.
    // The monitor needs both to know where in the trajectory the
    // controller currently is.
    std::mutex plan_mutex_;
    Plan         current_plan_;
    bool         current_plan_valid_{false};
    rclcpp::Time exec_start_time_;

    // Latest hand OBB + velocity history (all under hand_mutex_).
    struct HandSample { rclcpp::Time stamp; Eigen::Vector3d center; };
    mutable std::mutex      hand_mutex_;
    HandBox                 hand_box_;
    std::deque<HandSample>  hand_history_;
    Eigen::Vector3d         hand_velocity_{0.0, 0.0, 0.0};

    std::thread monitor_thread_;

    // ─── /planning_scene ingestion ────────────────────────────────────────
    void planning_scene_cb(const moveit_msgs::msg::PlanningScene::SharedPtr msg)
    {
        bool found = false;
        HandBox box;
        for (const auto & obj : msg->world.collision_objects) {
            if (obj.id != kHandObjectId) continue;
            if (obj.operation == moveit_msgs::msg::CollisionObject::REMOVE) continue;
            found = true;
            if (!obj.primitives.empty() && !obj.primitive_poses.empty()) {
                const auto & prim = obj.primitives.front();
                const auto & pose = obj.primitive_poses.front();
                if (prim.type == shape_msgs::msg::SolidPrimitive::BOX &&
                    prim.dimensions.size() >= 3)
                {
                    box.valid = true;
                    box.half_extents = Eigen::Vector3d(
                        prim.dimensions[0] * 0.5,
                        prim.dimensions[1] * 0.5,
                        prim.dimensions[2] * 0.5);
                    box.center = Eigen::Vector3d(
                        pose.position.x, pose.position.y, pose.position.z);
                    box.orientation = Eigen::Quaterniond(
                        pose.orientation.w, pose.orientation.x,
                        pose.orientation.y, pose.orientation.z);
                }
            }
            break;
        }
        const bool was_detected = hand_detected_.exchange(found);
        {
            std::lock_guard<std::mutex> lock(hand_mutex_);
            hand_box_ = box;
            if (found && box.valid) {
                hand_history_.push_back({this->now(), box.center});
                if (hand_history_.size() > kHandHistorySize)
                    hand_history_.pop_front();
                if (hand_history_.size() >= 2) {
                    const double dt =
                        (hand_history_.back().stamp -
                         hand_history_.front().stamp).seconds();
                    if (dt > 0.02)
                        hand_velocity_ =
                            (hand_history_.back().center -
                             hand_history_.front().center) / dt;
                }
            } else {
                hand_history_.clear();
                hand_velocity_.setZero();
            }
        }
        if (found && !was_detected) {
            RCLCPP_WARN(this->get_logger(), "Hand entered scene.");
        } else if (!found && was_detected) {
            RCLCPP_INFO(this->get_logger(), "Hand cleared from scene.");
        }
    }

    // ─── Phase 1: predictive monitor thread ───────────────────────────────
    void monitor_loop()
    {
        const auto period = std::chrono::duration<double>(monitor_period_s_);
        while (rclcpp::ok() && monitor_run_.load()) {
            std::this_thread::sleep_for(period);
            if (!is_executing_.load()) continue;
            if (collision_predicted_.load()) continue;  // already braking
            if (hand_detected_.load()) check_lookahead_collision();
            check_caution_proximity();
        }
    }

    // Direct FK-based predictive collision check.
    // For each upcoming trajectory waypoint, runs FK to get each monitored
    // link's world position, then checks it against the hand OBB shifted by
    // the estimated hand velocity × time-to-waypoint. No RPC round trips —
    // runs entirely in-process at roughly 10× the speed of the old service
    // call approach.
    void check_lookahead_collision()
    {
        if (!robot_model_) return;

        Plan plan_copy;
        rclcpp::Time start;
        {
            std::lock_guard<std::mutex> lock(plan_mutex_);
            if (!current_plan_valid_) return;
            plan_copy = current_plan_;
            start = exec_start_time_;
        }

        HandBox         cur_box;
        Eigen::Vector3d hand_vel;
        {
            std::lock_guard<std::mutex> lk(hand_mutex_);
            cur_box  = hand_box_;
            hand_vel = hand_velocity_;
        }
        if (!cur_box.valid) return;

        const moveit::core::JointModelGroup * jmg =
            robot_model_->getJointModelGroup(kPlanningGroup);
        if (!jmg) return;

        const auto & jt = plan_copy.trajectory_.joint_trajectory;
        if (jt.points.empty()) return;

        const double now_s     = (this->now() - start).seconds();
        const double horizon_s = now_s + lookahead_window_s_;

        moveit::core::RobotState rs(robot_model_);
        rs.setToDefaultValues();

        const int stride = std::max(1, validity_stride_);
        for (size_t i = 0; i < jt.points.size(); i += stride) {
            if (!is_executing_.load() || !hand_detected_.load()) return;

            const auto & pt = jt.points[i];
            const double t  = rclcpp::Duration(pt.time_from_start).seconds();
            if (t < now_s)     continue;
            if (t > horizon_s) break;

            // Predict hand OBB position when the robot reaches this waypoint
            HandBox predicted = cur_box;
            predicted.center += hand_vel * std::max(0.0, t - now_s);

            // FK: set joint positions and update all link transforms
            rs.setJointGroupPositions(jmg, pt.positions);
            rs.updateLinkTransforms();

            // Check each monitored link origin against the predicted hand OBB
            for (const auto * link : kCheckedLinks) {
                const Eigen::Vector3d lp =
                    rs.getGlobalLinkTransform(link).translation();
                const double dist = distance_point_to_obb(lp, predicted);
                if (dist < clearance_margin_m_) {
                    RCLCPP_WARN(this->get_logger(),
                        "Predicted collision at t=%.2fs: link '%s' dist=%.3fm "
                        "(margin=%.2fm, hand_vel=%.2fm/s) — braking.",
                        t, link, dist, clearance_margin_m_, hand_vel.norm());
                    collision_predicted_.store(true);
                    std::lock_guard<std::mutex> lk(move_group_mutex_);
                    if (active_move_group_) active_move_group_->stop();
                    return;
                }
            }
        }
    }

    // Checks whether the current EE position is within caution_distance_m_ of
    // the hand OBB. Decelerates (caution_triggered_) on entry, accelerates
    // back (caution_cleared_) when the hand moves away. Uses try_lock so it
    // never blocks the main planning thread.
    void check_caution_proximity()
    {
        if (caution_triggered_.load() || caution_cleared_.load()) return;

        HandBox box;
        const bool detected = hand_detected_.load();
        { std::lock_guard<std::mutex> lk(hand_mutex_); box = hand_box_; }

        if (!detected || !box.valid) {
            if (caution_active_.load()) {
                // Hand left the scene entirely — flag a speed change for the
                // next plan without interrupting the current trajectory.
                RCLCPP_INFO(this->get_logger(),
                    "Hand cleared — will resume normal speed after current move.");
                caution_cleared_.store(true);
                caution_active_.store(false);
            }
            return;
        }

        std::unique_lock<std::mutex> lk(move_group_mutex_, std::try_to_lock);
        if (!lk.owns_lock() || !active_move_group_) return;

        geometry_msgs::msg::PoseStamped ee;
        try { ee = active_move_group_->getCurrentPose(); }
        catch (...) { return; }
        lk.unlock();

        const Eigen::Vector3d ee_pos(ee.pose.position.x,
                                      ee.pose.position.y,
                                      ee.pose.position.z);
        const double dist = distance_point_to_obb(ee_pos, box);

        if (dist < caution_distance_m_ && !caution_active_.load()) {
            // Hand entered caution zone — flag for next replan, do NOT stop
            // the running trajectory (avoids the stop→replan lurch).
            RCLCPP_WARN(this->get_logger(),
                "Hand %.3fm from EE (threshold %.2fm) — will replan at caution speed.",
                dist, caution_distance_m_);
            caution_triggered_.store(true);
            caution_active_.store(true);
            std::lock_guard<std::mutex> lk2(move_group_mutex_);
            if (active_move_group_) active_move_group_->stop();
        } else if (dist >= caution_distance_m_ * 1.3 && caution_active_.load()) {
            // Hand left the caution zone — flag for next replan, do NOT stop.
            RCLCPP_INFO(this->get_logger(),
                "Hand %.3fm from EE — will resume normal speed after current move.", dist);
            caution_cleared_.store(true);
            caution_active_.store(false);
        }
    }

    // Returns the appropriate velocity scale for the current hand proximity.
    // Called from the main thread at every planning decision point.
    double proximity_vel_scale(
        moveit::planning_interface::MoveGroupInterface & mg) const
    {
        const bool detected = hand_detected_.load();
        HandBox box;
        { std::lock_guard<std::mutex> lk(hand_mutex_); box = hand_box_; }
        if (!detected || !box.valid) return vel_scale_normal_;
        try {
            const auto ee = mg.getCurrentPose();
            const Eigen::Vector3d p(ee.pose.position.x,
                                    ee.pose.position.y,
                                    ee.pose.position.z);
            return distance_point_to_obb(p, box) < caution_distance_m_
                   ? vel_scale_caution_ : vel_scale_normal_;
        } catch (...) {
            return vel_scale_normal_;
        }
    }

    double proximity_acc_scale(
        moveit::planning_interface::MoveGroupInterface & mg) const
    {
        const bool detected = hand_detected_.load();
        HandBox box;
        { std::lock_guard<std::mutex> lk(hand_mutex_); box = hand_box_; }
        if (!detected || !box.valid) return acc_scale_normal_;
        try {
            const auto ee = mg.getCurrentPose();
            const Eigen::Vector3d p(ee.pose.position.x,
                                    ee.pose.position.y,
                                    ee.pose.position.z);
            return distance_point_to_obb(p, box) < caution_distance_m_
                   ? acc_scale_caution_ : acc_scale_normal_;
        } catch (...) {
            return acc_scale_normal_;
        }
    }

    // ─── Phase 2: braking trajectory builder ──────────────────────────────
    // Linear velocity ramp: v(t) = v0 (1 - t/T); positions integrated
    // accordingly. Six segments are dense enough that the controller's
    // own interpolation stays smooth.
    Plan build_braking_trajectory(
        moveit::planning_interface::MoveGroupInterface & mg)
    {
        Plan brake;
        auto state = mg.getCurrentState(0.3);
        if (!state) return brake;
        const auto * jmg = state->getJointModelGroup(kPlanningGroup);
        if (!jmg) return brake;

        std::vector<double> q0, v0;
        state->copyJointGroupPositions(jmg, q0);
        state->copyJointGroupVelocities(jmg, v0);
        if (v0.size() != q0.size()) v0.assign(q0.size(), 0.0);

        // If the robot is already essentially stopped, emit a single hold
        // waypoint so the caller still gets a non-empty plan.
        double vmax = 0.0;
        for (double v : v0) vmax = std::max(vmax, std::abs(v));
        const double T = std::max(0.05, braking_duration_s_);

        trajectory_msgs::msg::JointTrajectory out;
        out.joint_names = jmg->getActiveJointModelNames();
        const size_t n = q0.size();

        if (vmax < 1e-3) {
            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions = q0;
            pt.velocities.assign(n, 0.0);
            pt.accelerations.assign(n, 0.0);
            pt.time_from_start = rclcpp::Duration::from_seconds(0.05);
            out.points.push_back(pt);
        } else {
            constexpr int K = 6;
            out.points.reserve(K + 1);
            for (int k = 0; k <= K; ++k) {
                const double t = T * static_cast<double>(k) / K;
                trajectory_msgs::msg::JointTrajectoryPoint pt;
                pt.positions.resize(n);
                pt.velocities.resize(n);
                pt.accelerations.assign(n, 0.0);
                for (size_t j = 0; j < n; ++j) {
                    pt.positions[j]  = q0[j] + v0[j] * t - v0[j] * t * t / (2.0 * T);
                    pt.velocities[j] = v0[j] * (1.0 - t / T);
                }
                pt.time_from_start = rclcpp::Duration::from_seconds(t);
                out.points.push_back(pt);
            }
        }

        brake.trajectory_.joint_trajectory = out;
        moveit::core::robotStateToRobotStateMsg(*state, brake.start_state_);
        brake.start_state_.is_diff = false;
        return brake;
    }

    // ─── Phase 3: safe-point candidate generation ─────────────────────────
    struct SafeCandidate
    {
        geometry_msgs::msg::Pose pose;
        std::vector<double>      joint_values;
        double clearance;
        double condition_number;
        double joint_displacement;
        double score;          // lower is better
    };

    bool evaluate_candidate(
        const moveit::core::RobotState &       seed,
        const moveit::core::JointModelGroup *  jmg,
        const std::string &                    tip_link,
        const Eigen::Isometry3d &              target_iso,
        SafeCandidate &                        out) const
    {
        moveit::core::RobotState st(seed);
        const double ik_timeout = 0.05;
        if (!st.setFromIK(jmg, target_iso, tip_link, ik_timeout)) return false;

        Eigen::MatrixXd J;
        const Eigen::Vector3d ref(0.0, 0.0, 0.0);
        if (!st.getJacobian(jmg, st.getLinkModel(tip_link), ref, J)) return false;

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
        const auto & sv = svd.singularValues();
        if (sv.size() == 0) return false;
        const double smax = sv(0);
        const double smin = sv(sv.size() - 1);
        const double cond = (smin > 1e-6)
            ? smax / smin
            : std::numeric_limits<double>::infinity();
        if (!std::isfinite(cond) || cond > condition_number_max_) return false;

        std::vector<double> seed_q, cand_q;
        seed.copyJointGroupPositions(jmg, seed_q);
        st.copyJointGroupPositions(jmg, cand_q);
        double disp = 0.0;
        for (size_t i = 0; i < cand_q.size(); ++i) {
            const double d = cand_q[i] - seed_q[i];
            disp += d * d;
        }
        disp = std::sqrt(disp);

        const Eigen::Vector3d ee_pos = target_iso.translation();
        HandBox box;
        {
            std::lock_guard<std::mutex> lock(hand_mutex_);
            box = hand_box_;
        }
        const double clearance = distance_point_to_obb(ee_pos, box);
        if (clearance < clearance_margin_m_) return false;

        // Reward extra clearance, penalise large joint moves and poor
        // conditioning. Coefficients chosen so all three terms sit in the
        // same rough order of magnitude.
        out.joint_values       = cand_q;
        out.condition_number   = cond;
        out.joint_displacement = disp;
        out.clearance          = clearance;
        out.score              = disp
                               + 0.5 * (cond / condition_number_max_)
                               - 1.0 * clearance;

        geometry_msgs::msg::Pose pose;
        pose.position.x = ee_pos.x();
        pose.position.y = ee_pos.y();
        pose.position.z = ee_pos.z();
        const Eigen::Quaterniond q(target_iso.rotation());
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        out.pose = pose;
        return true;
    }

    // Generate combinations of retreat (along -approach axis, i.e. away
    // from the goal in EE frame) and lift (along world +Z). The seed is
    // captured on the main thread before the async call so the
    // generator doesn't touch the shared MoveGroupInterface.
    std::vector<SafeCandidate> generate_safe_candidates(
        moveit::core::RobotStatePtr seed,
        const std::string &         tip_link,
        std::atomic<bool> &         cancel) const
    {
        std::vector<SafeCandidate> good;
        if (!seed) return good;
        const auto * jmg = seed->getJointModelGroup(kPlanningGroup);
        if (!jmg) return good;

        const Eigen::Isometry3d ee_now = seed->getGlobalLinkTransform(tip_link);
        const Eigen::Vector3d approach_axis_world =
            ee_now.rotation() * Eigen::Vector3d::UnitZ();

        const std::vector<double> retreats = {
            0.0,
            safe_point_retreat_m_ * 0.5,
            safe_point_retreat_m_,
            safe_point_retreat_m_ * 1.5};
        const std::vector<double> lifts = {
            0.0,
            safe_point_lift_m_ * 0.5,
            safe_point_lift_m_,
            safe_point_lift_m_ * 1.5};

        int evaluated = 0;
        for (double r : retreats) {
            for (double l : lifts) {
                if (cancel.load()) return good;
                if (evaluated >= max_safe_candidates_) break;
                if (r == 0.0 && l == 0.0) continue;
                Eigen::Isometry3d target = ee_now;
                target.translation() -= r * approach_axis_world;
                target.translation().z() += l;
                SafeCandidate cand;
                ++evaluated;
                if (evaluate_candidate(*seed, jmg, tip_link, target, cand)) {
                    good.push_back(cand);
                }
            }
        }
        std::sort(good.begin(), good.end(),
                  [](const SafeCandidate & a, const SafeCandidate & b) {
                      return a.score < b.score;
                  });
        return good;
    }

    // ─── Planning helpers ─────────────────────────────────────────────────
    geometry_msgs::msg::Pose make_goal_pose(
        std::shared_ptr<const ReachLocation::Goal> goal) const
    {
        geometry_msgs::msg::Pose p;
        p.position.x = goal->x; p.position.y = goal->y; p.position.z = goal->z;
        p.orientation.x = goal->qx; p.orientation.y = goal->qy;
        p.orientation.z = goal->qz; p.orientation.w = goal->qw;
        return p;
    }

    // Plans to a Cartesian pose. The primary planner is Pilz PTP — a
    // deterministic, smooth point-to-point joint motion that looks the same on
    // every run (no random RRT swings, no self-collision because the path is
    // still collision-checked). If PTP can't solve the pose we fall back to
    // OMPL. An optional Cartesian straight-line path is tried first when asked.
    bool plan_to_pose(
        moveit::planning_interface::MoveGroupInterface & mg,
        const geometry_msgs::msg::Pose & target,
        Plan & plan_out,
        bool allow_cartesian)
    {
        if (allow_cartesian) {
            // Pilz LIN — the tool tip travels a straight Cartesian line to the
            // target. It honors the configured Cartesian limits and the
            // velocity/acceleration scaling already set on the move group. If
            // the straight-line path is infeasible (e.g. it would cross a wrist
            // singularity or leave the workspace) LIN fails and we drop through
            // to the PTP path below.
            mg.setPlanningPipelineId("pilz_industrial_motion_planner");
            mg.setPlannerId("LIN");
            mg.clearPoseTargets();
            mg.setPoseTarget(target);
            if (mg.plan(plan_out) == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(),
                    "Planned with Pilz LIN (straight-line Cartesian).");
                return true;
            }
            RCLCPP_WARN(this->get_logger(),
                "Pilz LIN failed — falling back to PTP.");
        }

        // Primary: Pilz PTP — deterministic, smooth, repeatable.
        mg.setPlanningPipelineId("pilz_industrial_motion_planner");
        mg.setPlannerId("PTP");
        mg.clearPoseTargets();
        mg.setPoseTarget(target);
        if (mg.plan(plan_out) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planned with Pilz PTP.");
            return true;
        }

        // Fallback: OMPL (sampling-based).
        RCLCPP_WARN(this->get_logger(),
            "Pilz PTP failed — falling back to OMPL.");
        mg.setPlanningPipelineId("ompl");
        mg.setPlannerId("");
        mg.clearPoseTargets();
        mg.setPoseTarget(target);
        return mg.plan(plan_out) == moveit::core::MoveItErrorCode::SUCCESS;
    }

    bool plan_to_joint(
        moveit::planning_interface::MoveGroupInterface & mg,
        const std::vector<double> & target_joints,
        Plan & plan_out)
    {
        // Pilz PTP for the safe-point hops too, so recovery moves are just as
        // smooth and predictable as the main goal moves. OMPL fallback.
        mg.setPlanningPipelineId("pilz_industrial_motion_planner");
        mg.setPlannerId("PTP");
        mg.setJointValueTarget(target_joints);
        if (mg.plan(plan_out) == moveit::core::MoveItErrorCode::SUCCESS) {
            return true;
        }
        RCLCPP_WARN(this->get_logger(),
            "Pilz PTP (joint target) failed — falling back to OMPL.");
        mg.setPlanningPipelineId("ompl");
        mg.setPlannerId("");
        mg.setJointValueTarget(target_joints);
        return mg.plan(plan_out) == moveit::core::MoveItErrorCode::SUCCESS;
    }

    // ─── Waypoint loading & nearest-corner query ──────────────────────────────
    void load_waypoints(const std::string & filepath)
    {
        // Hardcoded table-corner defaults derived from table.xacro geometry.
        // All corners share the same gripper orientation.
        constexpr double kQx = -0.293, kQy =  0.956;
        constexpr double kQz =  0.000, kQw =  0.017;

        auto add_wp = [&](const char * n, double x, double y, double z) {
            WaypointPose wp;
            wp.name = n;
            wp.pose.position.x = x; wp.pose.position.y = y;
            wp.pose.position.z = z;
            wp.pose.orientation.x = kQx; wp.pose.orientation.y = kQy;
            wp.pose.orientation.z = kQz; wp.pose.orientation.w = kQw;
            waypoints_.push_back(wp);
        };

        std::ifstream f(filepath);
        if (!f.is_open()) {
            RCLCPP_WARN(this->get_logger(),
                "Waypoints file '%s' not found — using hardcoded table corners.",
                filepath.c_str());
            add_wp("TOP_LEFT",     -1.620,  0.454, 1.103);
            add_wp("TOP_RIGHT",    -0.220,  0.454, 1.103);
            add_wp("BOTTOM_LEFT",  -1.620, -0.256, 1.103);
            add_wp("BOTTOM_RIGHT", -0.220, -0.256, 1.103);
            return;
        }

        const std::string json_text{
            std::istreambuf_iterator<char>(f),
            std::istreambuf_iterator<char>()};

        const std::vector<std::string> names = {
            "TOP_LEFT", "TOP_RIGHT", "BOTTOM_LEFT", "BOTTOM_RIGHT"};

        bool any_loaded = false;
        for (const auto & cname : names) {
            const std::string section = json_find_object(json_text, cname);
            if (section.empty()) {
                RCLCPP_WARN(this->get_logger(),
                    "Corner '%s' not found in JSON — skipping.", cname.c_str());
                continue;
            }
            std::vector<double> trans, quat;
            if (!json_extract_array(section, "translation",     trans) ||
                trans.size() < 3 ||
                !json_extract_array(section, "quaternion_xyzw", quat) ||
                quat.size()  < 4)
            {
                RCLCPP_WARN(this->get_logger(),
                    "Corner '%s' has malformed data — skipping.", cname.c_str());
                continue;
            }
            WaypointPose wp;
            wp.name = cname;
            wp.pose.position.x    = trans[0];
            wp.pose.position.y    = trans[1];
            wp.pose.position.z    = trans[2];
            wp.pose.orientation.x = quat[0];
            wp.pose.orientation.y = quat[1];
            wp.pose.orientation.z = quat[2];
            wp.pose.orientation.w = quat[3];
            waypoints_.push_back(wp);
            any_loaded = true;
        }

        if (!any_loaded) {
            RCLCPP_WARN(this->get_logger(),
                "No corners parsed from '%s' — using hardcoded defaults.",
                filepath.c_str());
            add_wp("TOP_LEFT",     -1.620,  0.454, 1.103);
            add_wp("TOP_RIGHT",    -0.220,  0.454, 1.103);
            add_wp("BOTTOM_LEFT",  -1.620, -0.256, 1.103);
            add_wp("BOTTOM_RIGHT", -0.220, -0.256, 1.103);
            return;
        }
        RCLCPP_INFO(this->get_logger(),
            "Loaded %d corner waypoints from '%s'.",
            static_cast<int>(waypoints_.size()), filepath.c_str());
    }

    // Returns waypoint indices sorted by ascending 3-D distance to goal.
    std::vector<std::pair<double, size_t>> sort_waypoints_by_dist(
        const geometry_msgs::msg::Pose & goal) const
    {
        std::vector<std::pair<double, size_t>> indexed;
        indexed.reserve(waypoints_.size());
        for (size_t i = 0; i < waypoints_.size(); ++i) {
            const auto & p = waypoints_[i].pose.position;
            const double dx = p.x - goal.position.x;
            const double dy = p.y - goal.position.y;
            const double dz = p.z - goal.position.z;
            indexed.emplace_back(std::sqrt(dx*dx + dy*dy + dz*dz), i);
        }
        std::sort(indexed.begin(), indexed.end());
        return indexed;
    }

    // ─── Action handlers ──────────────────────────────────────────────────
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const ReachLocation::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(),
            "Received goal: pos=(%.3f, %.3f, %.3f) "
            "quat=(%.3f, %.3f, %.3f, %.3f) timeout=%.1f",
            goal->x, goal->y, goal->z,
            goal->qx, goal->qy, goal->qz, goal->qw, goal->timeout);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
    {
        RCLCPP_INFO(this->get_logger(), "Goal cancel requested.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        std::thread{std::bind(&ReachLocationActionServer::execute,
                              this, std::placeholders::_1),
                    goal_handle}.detach();
    }

    // ─── Main control flow ────────────────────────────────────────────────
    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback   = std::make_shared<ReachLocation::Feedback>();
        auto result     = std::make_shared<ReachLocation::Result>();

        moveit::planning_interface::MoveGroupInterface move_group(
            shared_from_this(), kPlanningGroup);
        move_group.setPlanningTime(planning_time_s_);
        move_group.setNumPlanningAttempts(5);
        move_group.allowReplanning(false);   // we drive recovery ourselves

        if (!robot_model_)
            robot_model_ = move_group.getRobotModel();

        feedback->current_x = goal->x;
        feedback->current_y = goal->y;
        goal_handle->publish_feedback(feedback);

        {
            std::lock_guard<std::mutex> lock(move_group_mutex_);
            active_move_group_ = &move_group;
        }

        monitor_run_.store(true);
        monitor_thread_ = std::thread(&ReachLocationActionServer::monitor_loop, this);

        struct Cleanup {
            ReachLocationActionServer * self;
            ~Cleanup() {
                self->monitor_run_.store(false);
                if (self->monitor_thread_.joinable()) self->monitor_thread_.join();
                { std::lock_guard<std::mutex> lock(self->move_group_mutex_);
                  self->active_move_group_ = nullptr; }
                { std::lock_guard<std::mutex> lock(self->plan_mutex_);
                  self->current_plan_valid_ = false; }
                self->is_executing_.store(false);
                self->collision_predicted_.store(false);
            }
        } cleanup{this};

        const geometry_msgs::msg::Pose target_pose = make_goal_pose(goal);
        // Straight-line Cartesian (Pilz LIN) requested for this goal? Only the
        // primary plan to the goal honors it; recovery paths stay PTP/OMPL.
        const bool goal_cartesian = goal->cartesian;

        enum class S {
            PLAN_TO_GOAL,
            EXECUTE_TO_GOAL,
            BRAKE,
            MOVE_TO_SAFE_POINT,
            AT_SAFE_POINT,
            HOLD_AND_WAIT,
            PLAN_VIA_WAYPOINT,
            EXECUTE_VIA_WAYPOINT,
            DONE_OK,
            DONE_FAIL
        };
        S state = S::PLAN_TO_GOAL;

        Plan goal_plan;
        std::vector<SafeCandidate> safe_candidates;
        size_t safe_attempt_idx = 0;

        // ── Waypoint fallback per-goal state ──────────────────────────────
        // sorted_wps: (dist_to_goal, waypoints_ index), ascending distance.
        // via_waypoint_used: prevents a second waypoint hop (no infinite loops).
        std::vector<std::pair<double, size_t>> sorted_wps;
        size_t  wp_try_idx       = 0;
        bool    via_waypoint_used = false;
        Plan    waypoint_plan;

        std::future<std::vector<SafeCandidate>> safe_select_fut;
        std::atomic<bool> safe_select_cancel{false};

        std::future<std::optional<Plan>> goal_preplan_fut;
        std::atomic<bool> goal_preplan_cancel{false};

        auto join_async = [&]() {
            safe_select_cancel.store(true);
            goal_preplan_cancel.store(true);
            if (safe_select_fut.valid())  safe_select_fut.wait();
            if (goal_preplan_fut.valid()) goal_preplan_fut.wait();
            safe_select_cancel.store(false);
            goal_preplan_cancel.store(false);
        };

        auto handle_cancel_request = [&]() {
            if (!goal_handle->is_canceling()) return false;
            { std::lock_guard<std::mutex> lock(move_group_mutex_);
              if (active_move_group_) active_move_group_->stop(); }
            join_async();
            result->success = false;
            result->message = "Cancelled by client.";
            goal_handle->canceled(result);
            return true;
        };

        while (rclcpp::ok() && state != S::DONE_OK && state != S::DONE_FAIL) {
            if (handle_cancel_request()) return;

            switch (state) {

            // ── PLAN_TO_GOAL ──────────────────────────────────────────────
            case S::PLAN_TO_GOAL: {
                move_group.setStartStateToCurrentState();
                {
                    const double v = proximity_vel_scale(move_group);
                    const double a = proximity_acc_scale(move_group);
                    move_group.setMaxVelocityScalingFactor(v);
                    move_group.setMaxAccelerationScalingFactor(a);
                    RCLCPP_INFO(this->get_logger(),
                        "Planning to goal (v=%.2f a=%.2f%s).",
                        v, a, v <= vel_scale_caution_ ? " [caution]" : "");
                }

                bool ok = false;
                for (int a = 1; a <= kMaxReplanAttempts && rclcpp::ok(); ++a) {
                    if (plan_to_pose(move_group, target_pose, goal_plan, goal_cartesian)) {
                        ok = true;
                        break;
                    }
                    RCLCPP_WARN(this->get_logger(),
                        "Plan attempt %d/%d failed — retrying in 500ms.",
                        a, kMaxReplanAttempts);
                    std::this_thread::sleep_for(300ms);
                    move_group.setStartStateToCurrentState();
                }
                if (!ok) {
                    if (enable_waypoint_fallback_ &&
                        !waypoints_.empty()        &&
                        !via_waypoint_used)
                    {
                        sorted_wps = sort_waypoints_by_dist(target_pose);
                        wp_try_idx = 0;
                        RCLCPP_WARN(this->get_logger(),
                            "Direct planning failed after %d attempts — "
                            "trying %d corner waypoints as intermediates.",
                            kMaxReplanAttempts,
                            static_cast<int>(sorted_wps.size()));
                        state = S::PLAN_VIA_WAYPOINT;
                    } else if (hand_detected_.load()) {
                        RCLCPP_WARN(this->get_logger(),
                            "Planning failed after %d attempts — hand is blocking "
                            "the scene. Holding until hand clears.",
                            kMaxReplanAttempts);
                        state = S::HOLD_AND_WAIT;
                    } else {
                        RCLCPP_ERROR(this->get_logger(),
                            "Planning failed after %d attempts%s.",
                            kMaxReplanAttempts,
                            via_waypoint_used
                                ? " (all waypoints already tried)"
                                : " (waypoint fallback disabled)");
                        state = S::DONE_FAIL;
                    }
                    break;
                }
                state = S::EXECUTE_TO_GOAL;
                break;
            }

            // ── EXECUTE_TO_GOAL ──────────────────────────────────────────
            case S::EXECUTE_TO_GOAL: {
                collision_predicted_.store(false);
                {
                    std::lock_guard<std::mutex> lock(plan_mutex_);
                    current_plan_       = goal_plan;
                    current_plan_valid_ = true;
                    exec_start_time_    = this->now();
                }
                is_executing_.store(true);
                RCLCPP_INFO(this->get_logger(), "Executing trajectory to goal.");
                auto rc = move_group.execute(goal_plan);
                is_executing_.store(false);
                {
                    std::lock_guard<std::mutex> lock(plan_mutex_);
                    current_plan_valid_ = false;
                }

                if (rc == moveit::core::MoveItErrorCode::SUCCESS) {
                    caution_active_.store(false);
                    state = S::DONE_OK;
                    break;
                }
                if (collision_predicted_.load()) {
                    caution_triggered_.store(false);
                    caution_cleared_.store(false);
                    state = S::BRAKE;
                    break;
                }
                // Hand entered caution zone mid-execution — replan at reduced speed.
                if (caution_triggered_.load()) {
                    caution_triggered_.store(false);
                    move_group.setStartStateToCurrentState();
                    move_group.setMaxVelocityScalingFactor(vel_scale_caution_);
                    move_group.setMaxAccelerationScalingFactor(acc_scale_caution_);
                    move_group.clearPoseTargets();
                    RCLCPP_INFO(this->get_logger(),
                        "Replanning at caution speed (v=%.2f) — hand is near.",
                        vel_scale_caution_);
                    if (plan_to_pose(move_group, target_pose, goal_plan, false)) break;
                    RCLCPP_WARN(this->get_logger(),
                        "Caution replan failed — falling back to normal speed.");
                    move_group.setMaxVelocityScalingFactor(vel_scale_normal_);
                    move_group.setMaxAccelerationScalingFactor(acc_scale_normal_);
                    if (plan_to_pose(move_group, target_pose, goal_plan, false)) break;
                    state = S::DONE_FAIL;
                    break;
                }
                // Hand left caution zone mid-execution — resume at normal speed.
                if (caution_cleared_.load()) {
                    caution_cleared_.store(false);
                    move_group.setStartStateToCurrentState();
                    move_group.setMaxVelocityScalingFactor(vel_scale_normal_);
                    move_group.setMaxAccelerationScalingFactor(acc_scale_normal_);
                    move_group.clearPoseTargets();
                    RCLCPP_INFO(this->get_logger(),
                        "Hand cleared — resuming at normal speed (v=%.2f).",
                        vel_scale_normal_);
                    if (plan_to_pose(move_group, target_pose, goal_plan, false)) break;
                    state = S::DONE_FAIL;
                    break;
                }
                RCLCPP_ERROR(this->get_logger(),
                    "Execution failed (code %d) with no collision prediction — "
                    "aborting.", rc.val);
                state = S::DONE_FAIL;
                break;
            }

            // ── BRAKE  +  parallel Phase 3 candidate scoring ─────────────
            case S::BRAKE: {
                caution_triggered_.store(false);
                caution_cleared_.store(false);
                // Kick off Phase 3 BEFORE issuing the brake so the
                // candidate set is ready (or nearly so) by the time the
                // robot is at rest.
                moveit::core::RobotStatePtr seed;
                if (auto cs = move_group.getCurrentState(0.3)) {
                    seed = std::make_shared<moveit::core::RobotState>(*cs);
                }
                const std::string tip_link = move_group.getEndEffectorLink();
                safe_select_cancel.store(false);
                safe_select_fut = std::async(std::launch::async,
                    [this, seed, tip_link, &safe_select_cancel]() {
                        return generate_safe_candidates(seed, tip_link,
                                                        safe_select_cancel);
                    });

                Plan brake = build_braking_trajectory(move_group);
                if (!brake.trajectory_.joint_trajectory.points.empty()) {
                    RCLCPP_INFO(this->get_logger(),
                        "Sending %zu-point braking trajectory (~%.2fs).",
                        brake.trajectory_.joint_trajectory.points.size(),
                        braking_duration_s_);
                    move_group.execute(brake);
                } else {
                    RCLCPP_WARN(this->get_logger(),
                        "Brake build returned no points — using controller stop.");
                    move_group.stop();
                }

                safe_candidates = safe_select_fut.get();
                safe_attempt_idx = 0;
                if (safe_candidates.empty()) {
                    RCLCPP_WARN(this->get_logger(),
                        "Zero viable safe-point candidates — replanning to goal.");
                    state = S::PLAN_TO_GOAL;
                    break;
                }
                const auto & best = safe_candidates.front();
                RCLCPP_INFO(this->get_logger(),
                    "Brake done. %zu candidates ranked; best: clearance=%.3fm "
                    "cond=%.1f Δq=%.3f score=%.3f.",
                    safe_candidates.size(),
                    best.clearance, best.condition_number,
                    best.joint_displacement, best.score);
                state = S::MOVE_TO_SAFE_POINT;
                break;
            }

            // ── MOVE_TO_SAFE_POINT  +  parallel Phase 5 goal pre-plan ────
            case S::MOVE_TO_SAFE_POINT: {
                if (safe_attempt_idx >= safe_candidates.size() ||
                    static_cast<int>(safe_attempt_idx) >= max_safe_attempts_)
                {
                    RCLCPP_WARN(this->get_logger(),
                        "Exhausted %d safe candidate(s) — replanning to goal.",
                        static_cast<int>(safe_attempt_idx));
                    state = S::PLAN_TO_GOAL;
                    break;
                }

                const SafeCandidate cand = safe_candidates[safe_attempt_idx];

                move_group.setStartStateToCurrentState();
                move_group.setMaxVelocityScalingFactor(vel_scale_safe_move_);
                move_group.setMaxAccelerationScalingFactor(acc_scale_safe_move_);
                move_group.clearPoseTargets();
                Plan to_safe;
                if (!plan_to_joint(move_group, cand.joint_values, to_safe)) {
                    RCLCPP_WARN(this->get_logger(),
                        "Plan to candidate #%zu failed — trying next.",
                        safe_attempt_idx);
                    ++safe_attempt_idx;
                    break;
                }

                // Phase 5: start pre-planning to goal while the arm moves.
                // Uses an independent MoveGroupInterface so it can't race
                // with the main one. The candidate joint values seed its
                // start state so the plan is valid from the rest pose.
                goal_preplan_cancel.store(false);
                const std::vector<double> seed_joints = cand.joint_values;
                const double v_norm = vel_scale_normal_;
                const double a_norm = acc_scale_normal_;
                const double plan_time = planning_time_s_;
                auto node_self = shared_from_this();

                goal_preplan_fut = std::async(std::launch::async,
                    [this, node_self, seed_joints, target_pose,
                     v_norm, a_norm, plan_time,
                     &goal_preplan_cancel]() -> std::optional<Plan>
                {
                    try {
                        if (goal_preplan_cancel.load()) return std::nullopt;
                        moveit::planning_interface::MoveGroupInterface mg(
                            node_self, kPlanningGroup);
                        mg.setPlanningTime(plan_time);
                        mg.setNumPlanningAttempts(5);
                        mg.setMaxVelocityScalingFactor(v_norm);
                        mg.setMaxAccelerationScalingFactor(a_norm);

                        auto base_state = mg.getCurrentState(0.5);
                        if (!base_state) return std::nullopt;
                        auto rs = std::make_shared<moveit::core::RobotState>(*base_state);
                        const auto * jmg = rs->getJointModelGroup(kPlanningGroup);
                        if (!jmg) return std::nullopt;
                        rs->setJointGroupPositions(jmg, seed_joints);
                        rs->update();
                        mg.setStartState(*rs);

                        if (goal_preplan_cancel.load()) return std::nullopt;
                        Plan p;
                        // Pilz PTP primary, OMPL fallback (mirrors plan_to_pose).
                        mg.setPlanningPipelineId("pilz_industrial_motion_planner");
                        mg.setPlannerId("PTP");
                        mg.setPoseTarget(target_pose);
                        if (mg.plan(p) == moveit::core::MoveItErrorCode::SUCCESS) {
                            return p;
                        }
                        if (goal_preplan_cancel.load()) return std::nullopt;
                        mg.setPlanningPipelineId("ompl");
                        mg.setPlannerId("");
                        mg.clearPoseTargets();
                        mg.setPoseTarget(target_pose);
                        if (mg.plan(p) == moveit::core::MoveItErrorCode::SUCCESS) {
                            return p;
                        }
                        return std::nullopt;
                    } catch (const std::exception & e) {
                        RCLCPP_WARN(this->get_logger(),
                            "Pre-plan thread error: %s", e.what());
                        return std::nullopt;
                    }
                });

                // Execute the move to the safe point.
                collision_predicted_.store(false);
                {
                    std::lock_guard<std::mutex> lock(plan_mutex_);
                    current_plan_       = to_safe;
                    current_plan_valid_ = true;
                    exec_start_time_    = this->now();
                }
                is_executing_.store(true);
                RCLCPP_INFO(this->get_logger(),
                    "Moving to safe candidate #%zu (clearance=%.3fm, cond=%.1f).",
                    safe_attempt_idx, cand.clearance, cand.condition_number);
                auto rc = move_group.execute(to_safe);
                is_executing_.store(false);
                {
                    std::lock_guard<std::mutex> lock(plan_mutex_);
                    current_plan_valid_ = false;
                }

                if (rc != moveit::core::MoveItErrorCode::SUCCESS) {
                    if (collision_predicted_.load()) {
                        RCLCPP_WARN(this->get_logger(),
                            "Predicted collision en-route to safe point — re-braking.");
                        goal_preplan_cancel.store(true);
                        if (goal_preplan_fut.valid()) goal_preplan_fut.wait();
                        state = S::BRAKE;
                    } else {
                        RCLCPP_WARN(this->get_logger(),
                            "Move to safe point failed (code %d) — trying next.",
                            rc.val);
                        goal_preplan_cancel.store(true);
                        if (goal_preplan_fut.valid()) goal_preplan_fut.wait();
                        ++safe_attempt_idx;
                    }
                    break;
                }
                state = S::AT_SAFE_POINT;
                break;
            }

            // ── AT_SAFE_POINT — Phase 6: prefer immediate execution ──────
            case S::AT_SAFE_POINT: {
                std::optional<Plan> preplan;
                if (goal_preplan_fut.valid()) preplan = goal_preplan_fut.get();

                if (preplan.has_value() && !hand_detected_.load()) {
                    caution_active_.store(false);
                    RCLCPP_INFO(this->get_logger(),
                        "At safe point — pre-plan ready and scene clear, launching.");
                    goal_plan = *preplan;
                    state = S::EXECUTE_TO_GOAL;
                    break;
                }

                if (hand_detected_.load()) {
                    RCLCPP_INFO(this->get_logger(),
                        "At safe point — hand still in scene; attempting fresh plan.");
                } else {
                    caution_active_.store(false);
                    RCLCPP_INFO(this->get_logger(),
                        "At safe point — pre-plan unavailable; planning afresh.");
                }

                move_group.setStartStateToCurrentState();
                move_group.setMaxVelocityScalingFactor(proximity_vel_scale(move_group));
                move_group.setMaxAccelerationScalingFactor(proximity_acc_scale(move_group));
                move_group.clearPoseTargets();
                Plan fresh;
                if (plan_to_pose(move_group, target_pose, fresh, false)) {
                    goal_plan = fresh;
                    state = S::EXECUTE_TO_GOAL;
                    break;
                }
                RCLCPP_INFO(this->get_logger(),
                    "No plan from safe point #%zu — trying next candidate.",
                    safe_attempt_idx);
                ++safe_attempt_idx;
                state = S::MOVE_TO_SAFE_POINT;
                break;
            }

            // ── HOLD_AND_WAIT — absolute last resort ─────────────────────
            case S::HOLD_AND_WAIT: {
                RCLCPP_WARN(this->get_logger(),
                    "Holding position; will retry planning every %.1fs until "
                    "the hand clears.",
                    hold_replan_interval_s_);
                while (rclcpp::ok()) {
                    if (handle_cancel_request()) return;
                    std::this_thread::sleep_for(
                        std::chrono::duration<double>(hold_replan_interval_s_));
                    if (hand_detected_.load()) continue;

                    caution_active_.store(false);
                    move_group.setStartStateToCurrentState();
                    move_group.setMaxVelocityScalingFactor(proximity_vel_scale(move_group));
                    move_group.setMaxAccelerationScalingFactor(proximity_acc_scale(move_group));
                    move_group.clearPoseTargets();
                    Plan fresh;
                    if (plan_to_pose(move_group, target_pose, fresh, false)) {
                        RCLCPP_INFO(this->get_logger(),
                            "Hand cleared — resuming toward goal.");
                        goal_plan = fresh;
                        state = S::EXECUTE_TO_GOAL;
                        break;
                    }
                    RCLCPP_WARN(this->get_logger(),
                        "Hand clear but plan still failed — continuing to hold.");
                }
                if (state == S::HOLD_AND_WAIT) state = S::DONE_FAIL;
                break;
            }

            // ── PLAN_VIA_WAYPOINT ─────────────────────────────────────────
            // Try corners in order of ascending 3-D distance to the goal.
            // Each corner gets up to 3 plan attempts before we advance.
            // If all corners are exhausted the goal is aborted.
            case S::PLAN_VIA_WAYPOINT: {
                if (wp_try_idx >= sorted_wps.size()) {
                    RCLCPP_ERROR(this->get_logger(),
                        "All %d waypoints exhausted — cannot reach goal.",
                        static_cast<int>(sorted_wps.size()));
                    state = S::DONE_FAIL;
                    break;
                }

                const size_t wp_idx  = sorted_wps[wp_try_idx].second;
                const auto & wp      = waypoints_[wp_idx];
                const double wp_dist = sorted_wps[wp_try_idx].first;

                RCLCPP_INFO(this->get_logger(),
                    "Waypoint fallback [%d/%d]: planning to '%s' "
                    "(%.3f m from goal).",
                    static_cast<int>(wp_try_idx + 1),
                    static_cast<int>(sorted_wps.size()),
                    wp.name.c_str(), wp_dist);

                move_group.setStartStateToCurrentState();
                move_group.setMaxVelocityScalingFactor(proximity_vel_scale(move_group));
                move_group.setMaxAccelerationScalingFactor(proximity_acc_scale(move_group));
                move_group.clearPoseTargets();

                bool ok = false;
                for (int a = 1; a <= 3 && rclcpp::ok(); ++a) {
                    if (plan_to_pose(move_group, wp.pose, waypoint_plan, false)) {
                        ok = true;
                        break;
                    }
                    RCLCPP_WARN(this->get_logger(),
                        "  Waypoint '%s' plan attempt %d/3 failed.",
                        wp.name.c_str(), a);
                    std::this_thread::sleep_for(200ms);
                    move_group.setStartStateToCurrentState();
                }

                if (!ok) {
                    RCLCPP_WARN(this->get_logger(),
                        "  Cannot plan to '%s' — trying next corner.",
                        wp.name.c_str());
                    ++wp_try_idx;
                    break;
                }
                RCLCPP_INFO(this->get_logger(),
                    "  Plan to '%s' ready — executing.", wp.name.c_str());
                state = S::EXECUTE_VIA_WAYPOINT;
                break;
            }

            // ── EXECUTE_VIA_WAYPOINT ──────────────────────────────────────
            // Execute the plan to the intermediate corner.
            // On success, restart PLAN_TO_GOAL from the new (corner) position.
            // On collision, hand off to the existing BRAKE recovery chain.
            // On plain failure, advance to the next corner.
            case S::EXECUTE_VIA_WAYPOINT: {
                const size_t wp_idx = sorted_wps[wp_try_idx].second;
                const auto & wp     = waypoints_[wp_idx];

                collision_predicted_.store(false);
                {
                    std::lock_guard<std::mutex> lock(plan_mutex_);
                    current_plan_       = waypoint_plan;
                    current_plan_valid_ = true;
                    exec_start_time_    = this->now();
                }
                is_executing_.store(true);
                RCLCPP_INFO(this->get_logger(),
                    "Executing to waypoint '%s' (v=%.2f a=%.2f).",
                    wp.name.c_str(),
                    vel_scale_normal_, acc_scale_normal_);
                auto rc = move_group.execute(waypoint_plan);
                is_executing_.store(false);
                {
                    std::lock_guard<std::mutex> lock(plan_mutex_);
                    current_plan_valid_ = false;
                }

                if (rc == moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(this->get_logger(),
                        "Reached waypoint '%s' — replanning directly to goal.",
                        wp.name.c_str());
                    via_waypoint_used = true;   // one hop only: prevents re-entry
                    state = S::PLAN_TO_GOAL;
                    break;
                }
                if (collision_predicted_.load()) {
                    RCLCPP_WARN(this->get_logger(),
                        "Collision predicted en-route to waypoint '%s' — braking.",
                        wp.name.c_str());
                    state = S::BRAKE;
                    break;
                }
                RCLCPP_WARN(this->get_logger(),
                    "Execution to waypoint '%s' failed (code %d) — trying next.",
                    wp.name.c_str(), rc.val);
                ++wp_try_idx;
                state = S::PLAN_VIA_WAYPOINT;
                break;
            }

            default: state = S::DONE_FAIL; break;
            }
        }

        join_async();

        if (state == S::DONE_OK) {
            result->success = true;
            result->message = "Target reached successfully.";
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded.");
        } else {
            result->success = false;
            result->message = "Failed to reach goal.";
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Goal aborted.");
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReachLocationActionServer>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
