#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include "bt_action_server/action/reach_location.hpp"

class ReachLocationActionServer : public rclcpp::Node
{
public:
    using ReachLocation = bt_action_server::action::ReachLocation;
    using GoalHandle    = rclcpp_action::ServerGoalHandle<ReachLocation>;

    explicit ReachLocationActionServer()
    : Node("reach_location_action_server",
           rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        action_server_ = rclcpp_action::create_server<ReachLocation>(
            this, "reach_location",
            std::bind(&ReachLocationActionServer::handle_goal,     this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&ReachLocationActionServer::handle_cancel,   this,
                      std::placeholders::_1),
            std::bind(&ReachLocationActionServer::handle_accepted, this,
                      std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "ReachLocation action server ready.");
    }

private:
    rclcpp_action::Server<ReachLocation>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const ReachLocation::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Received goal: x=%.3f  y=%.3f  timeout=%.1f",
                    goal->x, goal->y, goal->timeout);
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

    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback   = std::make_shared<ReachLocation::Feedback>();
        auto result     = std::make_shared<ReachLocation::Result>();

        moveit::planning_interface::MoveGroupInterface move_group(
            shared_from_this(), "ur_manipulator");

        move_group.setPlanningTime(goal->timeout);
        move_group.setMaxVelocityScalingFactor(0.3);
        move_group.setMaxAccelerationScalingFactor(0.3);
        move_group.setNumPlanningAttempts(10);
        move_group.allowReplanning(true);		// Enabling Moveit2 continuous replanning

        feedback->current_x = goal->x;
        feedback->current_y = goal->y;
        goal_handle->publish_feedback(feedback);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::core::MoveItErrorCode plan_result;

        // If x=0 and y=0 exactly, return to named home state
        if (std::abs(goal->x) < 1e-6 && std::abs(goal->y) < 1e-6) {
            RCLCPP_INFO(this->get_logger(), "Returning to named home state...");
            move_group.setNamedTarget("home");
            plan_result = move_group.plan(plan);
        } else {
            geometry_msgs::msg::Pose target;
            target.position.x    = goal->x;
            target.position.y    = goal->y;
            target.position.z    =  0.980;
            target.orientation.x =  -0.500;
            target.orientation.y =  0.500;
            target.orientation.z =  0.500;
            target.orientation.w =  0.500;

            RCLCPP_INFO(this->get_logger(),
                        "Planning to (%.3f, %.3f, 0.900)...", goal->x, goal->y);
            move_group.setPoseTarget(target);
            plan_result = move_group.plan(plan);
        }

        if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(),
                         "Planning failed with code %d", plan_result.val);
            result->success = false;
            result->message = "Planning failed.";
            goal_handle->abort(result);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Plan found — executing...");

        auto exec_result = move_group.execute(plan);
        if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
            result->success = true;
            result->message = "Target reached successfully.";
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded.");
        } else {
            result->success = false;
            result->message = "Execution failed.";
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(),
                         "Execution failed with code %d", exec_result.val);
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReachLocationActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
