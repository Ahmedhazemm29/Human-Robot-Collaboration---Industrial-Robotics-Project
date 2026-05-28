// ════════════════════════════════════════════════════════════════════
//  sort_blocks.cpp — Human/robot collaborative block-sorting demo
//
//  Scenario:
//    Three blocks named "block_1", "block_2", "block_3" are published
//    as collision objects in /planning_scene at arbitrary positions on
//    the table. A human reaches in, picks one block, and removes it
//    from the workspace. The robot then picks one of the two remaining
//    blocks and places it in that block's pre-assigned sort slot.
//    Repeats for the second remaining block.
//
//  Collision avoidance:
//    The same hand_exclusion_zone pattern as bt_action_server. A
//    /planning_scene subscriber watches for an obstacle with the
//    id "hand_exclusion_zone". On rising-edge detection, while a
//    trajectory is executing, the stored plan is re-validated against
//    the current scene via /check_state_validity. If any waypoint is
//    invalid the motion is stopped, the node waits for the hand to
//    leave, then replans to the same goal from the current state.
//
//  Hardcoded values you will want to tune for your cell:
//    - kSlotPose[]: the three drop targets ("sorted" positions)
//    - kApproachZOffset: how high above each block / slot to plan
//    - kTopOrientation:  end-effector orientation used for top-down picks
//
//  Out of scope (left as // TODO):
//    - Gripper open/close. Plug in your Robotiq / OnRobot / custom
//      command at the two TODOs in pick_and_place().
// ════════════════════════════════════════════════════════════════════

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

class BlockSorterNode : public rclcpp::Node
{
public:
    BlockSorterNode()
    : Node("block_sorter",
           rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        monitor_cb_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = monitor_cb_group_;

        planning_scene_sub_ = this->create_subscription<moveit_msgs::msg::PlanningScene>(
            "/planning_scene", rclcpp::QoS(10),
            std::bind(&BlockSorterNode::planning_scene_cb, this,
                      std::placeholders::_1),
            sub_opts);

        validity_client_ = this->create_client<moveit_msgs::srv::GetStateValidity>(
            "/check_state_validity",
            rmw_qos_profile_services_default,
            monitor_cb_group_);

        // Sorter idles after launch and only starts when explicitly triggered
        // via /block_sorter/start. This is the coordination point that keeps
        // it from planning while bt_action_server is also using MoveIt
        // ur_manipulator: callers (a human operator with `ros2 service call`,
        // or a future BT leaf) only trigger when the BT is idle.
        start_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/block_sorter/start",
            std::bind(&BlockSorterNode::on_start_request, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(),
                    "block_sorter ready — waiting for `ros2 service call "
                    "/block_sorter/start std_srvs/srv/Trigger {}`.");
    }

    ~BlockSorterNode() override {
        if (run_thread_.joinable()) run_thread_.join();
    }

    void run();

private:
    static constexpr const char * kPlanningGroup = "ur_manipulator";
    static constexpr const char * kHandObjectId  = "hand_exclusion_zone";
    static constexpr double kApproachZOffset     = 0.10;   // m above block / slot

    // Top-down orientation copied from reach_location_server's hardcoded pose
    // — adjust to your end-effector convention.
    struct Quat { double x, y, z, w; };
    static constexpr Quat kTopOrientation = {-0.087, 0.996, 0.010, 0.007};

    // Each block's destination on the sorted side of the table.
    // Edit these to match where you actually want each block placed.
    static const std::map<std::string, geometry_msgs::msg::Pose> & slotMap();

    static const std::vector<std::string> & blockIds();   // {"block_1",...}

    // ---- ROS handles ----
    rclcpp::CallbackGroup::SharedPtr monitor_cb_group_;
    rclcpp::Subscription<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_sub_;
    rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr   validity_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr              start_service_;

    // Trigger-driven run: the service callback launches run() on a worker
    // thread and returns immediately. is_running_ rejects duplicate triggers.
    std::atomic<bool> is_running_{false};
    std::thread       run_thread_;

    void on_start_request(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
    {
        bool already = is_running_.exchange(true);
        if (already) {
            resp->success = false;
            resp->message = "Sorter is already running — refusing duplicate trigger.";
            RCLCPP_WARN(this->get_logger(), "%s", resp->message.c_str());
            return;
        }
        if (run_thread_.joinable()) run_thread_.join();
        run_thread_ = std::thread([this]() {
            try { this->run(); }
            catch (const std::exception & e) {
                RCLCPP_ERROR(this->get_logger(),
                             "Sorter aborted: %s", e.what());
            }
            is_running_.store(false);
        });
        resp->success = true;
        resp->message = "Sorter started.";
        RCLCPP_INFO(this->get_logger(), "%s", resp->message.c_str());
    }

    // ---- Scene state (guarded by scene_mutex_) ----
    std::mutex scene_mutex_;
    std::set<std::string>                                present_blocks_;
    std::map<std::string, geometry_msgs::msg::Pose>      last_block_poses_;

    // ---- Cross-thread flags for hand collision avoidance ----
    std::atomic<bool> hand_detected_{false};
    std::atomic<bool> is_executing_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> check_in_flight_{false};

    std::mutex move_group_mutex_;
    moveit::planning_interface::MoveGroupInterface * active_move_group_{nullptr};

    std::mutex plan_mutex_;
    moveit::planning_interface::MoveGroupInterface::Plan current_plan_;
    bool current_plan_valid_{false};

    // ---- Callbacks / helpers ----
    void planning_scene_cb(const moveit_msgs::msg::PlanningScene::SharedPtr msg);
    void check_plan_and_maybe_stop();
    void request_stop();
    void wait_until_hand_cleared();

    bool wait_for_three_blocks(std::chrono::seconds timeout);
    bool wait_for_human_pick(std::chrono::seconds timeout, std::string & picked_id);

    bool move_to_pose(moveit::planning_interface::MoveGroupInterface & mg,
                      const geometry_msgs::msg::Pose & target,
                      const std::string & description);
    bool pick_and_place(moveit::planning_interface::MoveGroupInterface & mg,
                        const std::string & block_id);

    static geometry_msgs::msg::Pose withOrientation(geometry_msgs::msg::Pose p);
};

// ──────────────────────────────────────────────────────────────────────
//  Static configuration
// ──────────────────────────────────────────────────────────────────────

const std::vector<std::string> & BlockSorterNode::blockIds()
{
    static const std::vector<std::string> ids = {"block_1", "block_2", "block_3"};
    return ids;
}

const std::map<std::string, geometry_msgs::msg::Pose> & BlockSorterNode::slotMap()
{
    static std::map<std::string, geometry_msgs::msg::Pose> slots;
    if (!slots.empty()) return slots;

    auto make = [](double x, double y, double z) {
        geometry_msgs::msg::Pose p;
        p.position.x = x; p.position.y = y; p.position.z = z;
        return withOrientation(p);
    };
    // Three sort slots, placed in a row on the far side of the workspace.
    // Tune to your table layout.
    slots["block_1"] = make(0.50, -0.15, 1.10);
    slots["block_2"] = make(0.50,  0.00, 1.10);
    slots["block_3"] = make(0.50,  0.15, 1.10);
    return slots;
}

geometry_msgs::msg::Pose BlockSorterNode::withOrientation(geometry_msgs::msg::Pose p)
{
    p.orientation.x = kTopOrientation.x;
    p.orientation.y = kTopOrientation.y;
    p.orientation.z = kTopOrientation.z;
    p.orientation.w = kTopOrientation.w;
    return p;
}

// ──────────────────────────────────────────────────────────────────────
//  Planning-scene callback — track blocks + hand
// ──────────────────────────────────────────────────────────────────────

void BlockSorterNode::planning_scene_cb(
    const moveit_msgs::msg::PlanningScene::SharedPtr msg)
{
    bool hand_now = false;

    {
        std::lock_guard<std::mutex> lock(scene_mutex_);
        for (const auto & obj : msg->world.collision_objects) {
            if (obj.id == kHandObjectId) {
                if (obj.operation != moveit_msgs::msg::CollisionObject::REMOVE) {
                    hand_now = true;
                }
                continue;
            }

            for (const auto & bid : blockIds()) {
                if (obj.id != bid) continue;
                if (obj.operation == moveit_msgs::msg::CollisionObject::REMOVE) {
                    present_blocks_.erase(bid);
                } else {
                    present_blocks_.insert(bid);
                    if (!obj.primitive_poses.empty()) {
                        last_block_poses_[bid] = obj.primitive_poses.front();
                    } else if (!obj.mesh_poses.empty()) {
                        last_block_poses_[bid] = obj.mesh_poses.front();
                    }
                }
                break;
            }
        }
    }

    const bool was_detected = hand_detected_.exchange(hand_now);
    if (!hand_now || was_detected || !is_executing_.load()) return;

    bool expected = false;
    if (!check_in_flight_.compare_exchange_strong(expected, true)) return;
    std::thread([this]() {
        check_plan_and_maybe_stop();
        check_in_flight_.store(false);
    }).detach();
}

// ──────────────────────────────────────────────────────────────────────
//  Validity-check worker — only stop if hand actually intersects the path
// ──────────────────────────────────────────────────────────────────────

void BlockSorterNode::check_plan_and_maybe_stop()
{
    moveit::planning_interface::MoveGroupInterface::Plan plan_copy;
    {
        std::lock_guard<std::mutex> lock(plan_mutex_);
        if (!current_plan_valid_) return;
        plan_copy = current_plan_;
    }

    if (!validity_client_->wait_for_service(500ms)) {
        RCLCPP_WARN(this->get_logger(),
                    "/check_state_validity unavailable — stopping conservatively.");
        request_stop();
        return;
    }

    const auto & jt = plan_copy.trajectory_.joint_trajectory;
    if (jt.points.empty()) return;

    moveit_msgs::msg::RobotState rs = plan_copy.start_state_;
    rs.is_diff = false;
    rs.joint_state.name = jt.joint_names;
    rs.joint_state.velocity.clear();
    rs.joint_state.effort.clear();

    for (const auto & pt : jt.points) {
        if (!is_executing_.load() || !hand_detected_.load()) return;

        rs.joint_state.position = pt.positions;
        auto req = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
        req->robot_state = rs;
        req->group_name  = kPlanningGroup;

        auto future = validity_client_->async_send_request(req);
        if (future.wait_for(300ms) != std::future_status::ready) continue;

        if (!future.get()->valid) {
            RCLCPP_WARN(this->get_logger(),
                "Hand blocks waypoint at t=%.2fs — stopping.",
                rclcpp::Duration(pt.time_from_start).seconds());
            request_stop();
            return;
        }
    }
    RCLCPP_INFO(this->get_logger(),
                "Hand present but does not intersect planned path — continuing.");
}

void BlockSorterNode::request_stop()
{
    stop_requested_.store(true);
    std::lock_guard<std::mutex> lock(move_group_mutex_);
    if (active_move_group_ != nullptr) active_move_group_->stop();
}

void BlockSorterNode::wait_until_hand_cleared()
{
    if (!hand_detected_.load()) return;
    RCLCPP_INFO(this->get_logger(),
                "Waiting for human hand to leave the workspace...");
    while (rclcpp::ok() && hand_detected_.load()) {
        std::this_thread::sleep_for(100ms);
    }
    RCLCPP_INFO(this->get_logger(), "Hand cleared — proceeding.");
}

// ──────────────────────────────────────────────────────────────────────
//  Waiting helpers
// ──────────────────────────────────────────────────────────────────────

bool BlockSorterNode::wait_for_three_blocks(std::chrono::seconds timeout)
{
    const auto deadline = this->now() + rclcpp::Duration(timeout);
    while (rclcpp::ok() && this->now() < deadline) {
        {
            std::lock_guard<std::mutex> lock(scene_mutex_);
            if (present_blocks_.size() == blockIds().size()) return true;
        }
        std::this_thread::sleep_for(100ms);
    }
    return false;
}

bool BlockSorterNode::wait_for_human_pick(std::chrono::seconds timeout,
                                          std::string & picked_id)
{
    const auto deadline = this->now() + rclcpp::Duration(timeout);
    while (rclcpp::ok() && this->now() < deadline) {
        std::set<std::string> snapshot;
        {
            std::lock_guard<std::mutex> lock(scene_mutex_);
            snapshot = present_blocks_;
        }
        if (snapshot.size() == blockIds().size() - 1) {
            for (const auto & bid : blockIds()) {
                if (snapshot.find(bid) == snapshot.end()) {
                    picked_id = bid;
                    return true;
                }
            }
        }
        std::this_thread::sleep_for(100ms);
    }
    return false;
}

// ──────────────────────────────────────────────────────────────────────
//  Plan → execute with stop/replan-on-hand
// ──────────────────────────────────────────────────────────────────────

bool BlockSorterNode::move_to_pose(
    moveit::planning_interface::MoveGroupInterface & mg,
    const geometry_msgs::msg::Pose & target,
    const std::string & description)
{
    while (rclcpp::ok()) {
        mg.setStartStateToCurrentState();
        mg.setPoseTarget(target);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto plan_result = mg.plan(plan);
        if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(),
                         "Plan failed for %s (code %d)",
                         description.c_str(), plan_result.val);
            return false;
        }

        {
            std::lock_guard<std::mutex> lock(plan_mutex_);
            current_plan_       = plan;
            current_plan_valid_ = true;
        }

        RCLCPP_INFO(this->get_logger(), "Executing motion: %s", description.c_str());
        stop_requested_.store(false);
        is_executing_.store(true);
        auto exec_result = mg.execute(plan);
        is_executing_.store(false);

        {
            std::lock_guard<std::mutex> lock(plan_mutex_);
            current_plan_valid_ = false;
        }

        if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Reached: %s", description.c_str());
            return true;
        }

        if (stop_requested_.load()) {
            RCLCPP_WARN(this->get_logger(),
                        "Interrupted by hand — waiting and replanning to %s.",
                        description.c_str());
            std::this_thread::sleep_for(300ms);
            wait_until_hand_cleared();
            stop_requested_.store(false);
            continue;
        }

        RCLCPP_ERROR(this->get_logger(),
                     "Execution failed for %s (code %d)",
                     description.c_str(), exec_result.val);
        return false;
    }
    return false;
}

bool BlockSorterNode::pick_and_place(
    moveit::planning_interface::MoveGroupInterface & mg,
    const std::string & block_id)
{
    geometry_msgs::msg::Pose pick_pose;
    {
        std::lock_guard<std::mutex> lock(scene_mutex_);
        auto it = last_block_poses_.find(block_id);
        if (it == last_block_poses_.end()) {
            RCLCPP_ERROR(this->get_logger(),
                         "No cached pose for %s", block_id.c_str());
            return false;
        }
        pick_pose = it->second;
    }
    pick_pose = withOrientation(pick_pose);
    pick_pose.position.z += kApproachZOffset;

    if (!move_to_pose(mg, pick_pose, "approach " + block_id)) return false;

    // TODO: trigger gripper CLOSE here (Robotiq, OnRobot, custom).
    RCLCPP_INFO(this->get_logger(), "[stub] Gripper close on %s", block_id.c_str());
    std::this_thread::sleep_for(500ms);

    const auto & slots = slotMap();
    auto sit = slots.find(block_id);
    if (sit == slots.end()) {
        RCLCPP_ERROR(this->get_logger(),
                     "No slot mapping for %s", block_id.c_str());
        return false;
    }
    geometry_msgs::msg::Pose slot_pose = sit->second;
    slot_pose.position.z += kApproachZOffset;

    if (!move_to_pose(mg, slot_pose, "place " + block_id)) return false;

    // TODO: trigger gripper OPEN here.
    RCLCPP_INFO(this->get_logger(), "[stub] Gripper open at slot of %s", block_id.c_str());
    std::this_thread::sleep_for(500ms);

    return true;
}

// ──────────────────────────────────────────────────────────────────────
//  Main sequence
// ──────────────────────────────────────────────────────────────────────

void BlockSorterNode::run()
{
    moveit::planning_interface::MoveGroupInterface mg(
        shared_from_this(), kPlanningGroup);
    mg.setPlanningTime(15.0);
    mg.setMaxVelocityScalingFactor(0.3);
    mg.setMaxAccelerationScalingFactor(0.3);
    mg.setNumPlanningAttempts(10);
    mg.allowReplanning(true);

    {
        std::lock_guard<std::mutex> lock(move_group_mutex_);
        active_move_group_ = &mg;
    }
    struct ScopeGuard {
        BlockSorterNode * self;
        ~ScopeGuard() {
            std::lock_guard<std::mutex> lock(self->move_group_mutex_);
            self->active_move_group_ = nullptr;
        }
    } guard{this};

    RCLCPP_INFO(this->get_logger(),
                "Waiting for three blocks (block_1/2/3) in /planning_scene...");
    if (!wait_for_three_blocks(60s)) {
        RCLCPP_ERROR(this->get_logger(),
                     "Timed out waiting for blocks. Have they been published?");
        return;
    }

    RCLCPP_INFO(this->get_logger(),
                "Three blocks present. Waiting for the human to pick one...");
    std::string human_picked;
    if (!wait_for_human_pick(300s, human_picked)) {
        RCLCPP_ERROR(this->get_logger(), "Timed out waiting for human pick.");
        return;
    }
    RCLCPP_INFO(this->get_logger(),
                "Human picked %s. Robot will sort the remaining two.",
                human_picked.c_str());

    std::vector<std::string> remaining;
    {
        std::lock_guard<std::mutex> lock(scene_mutex_);
        for (const auto & b : present_blocks_) remaining.push_back(b);
    }
    std::sort(remaining.begin(), remaining.end());

    for (const auto & block_id : remaining) {
        // The hand may already be present — wait before each pick so we
        // don't start planning into the human.
        wait_until_hand_cleared();
        if (!pick_and_place(mg, block_id)) {
            RCLCPP_ERROR(this->get_logger(),
                         "Aborting sort: failed on %s", block_id.c_str());
            return;
        }
    }

    RCLCPP_INFO(this->get_logger(), "Sorting complete.");
}

// ════════════════════════════════════════════════════════════════════

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BlockSorterNode>();

    // The node idles after construction; run() is launched from
    // /block_sorter/start service callbacks. We just spin the executor
    // until the process is killed.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
