#include <memory>
#include <string>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

class EEPoseCommander : public rclcpp::Node
{
public:
  explicit EEPoseCommander(const rclcpp::NodeOptions& options)
  : Node("ee_pose_commander", options)
  {
    // Point 1
    this->declare_parameter("point1_x", 0.188);
    this->declare_parameter("point1_y", -0.238);
    this->declare_parameter("point1_z", 0.832);
    this->declare_parameter("point1_roll", 1.571);
    this->declare_parameter("point1_pitch", 0.000);
    this->declare_parameter("point1_yaw", 0.000);

    // Point 2
    this->declare_parameter("point2_x", -0.024);
    this->declare_parameter("point2_y", -0.232);
    this->declare_parameter("point2_z", 0.870);
    this->declare_parameter("point2_roll", 1.561);
    this->declare_parameter("point2_pitch", -0.072);
    this->declare_parameter("point2_yaw", 0.032);


    // Point 3
    this->declare_parameter("point3_x", 0.0);
    this->declare_parameter("point3_y", -0.232);
    this->declare_parameter("point3_z", 0.870);
    this->declare_parameter("point3_roll", 1.571);
    this->declare_parameter("point3_pitch", 0.000);
    this->declare_parameter("point3_yaw", -1.571);

    // MoveIt setup
    this->declare_parameter("planning_group", "ur_manipulator");
    this->declare_parameter("pose_reference_frame", "world");
    this->declare_parameter("end_effector_link", "tool0");
    this->declare_parameter("home_named_target", "up");

    this->declare_parameter("position_tolerance", 0.01);
    this->declare_parameter("orientation_tolerance", 0.10);

    this->declare_parameter("planning_time", 10.0);
    this->declare_parameter("num_planning_attempts", 10);

    this->declare_parameter("velocity_scale", 0.05);
    this->declare_parameter("acceleration_scale", 0.05);

    this->declare_parameter("allow_replanning", true);
    this->declare_parameter("startup_delay_sec", 2.0);
    this->declare_parameter("state_wait_sec", 10.0);
    this->declare_parameter("pause_between_moves_sec", 2.0);
  }

private:
  bool moveToNamedTarget(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const std::string& target_name)
  {
    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
    RCLCPP_INFO(this->get_logger(), "Moving to named target: %s", target_name.c_str());

    move_group.setStartStateToCurrentState();
    move_group.clearPoseTargets();
    move_group.clearPathConstraints();

    if (!move_group.setNamedTarget(target_name))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to set named target: %s", target_name.c_str());
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode plan_result = move_group.plan(plan);

    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Planning failed for named target %s. Error code: %d",
        target_name.c_str(), plan_result.val
      );
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Planning succeeded for named target: %s", target_name.c_str());

    moveit::core::MoveItErrorCode exec_result = move_group.execute(plan);

    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Execution failed for named target %s. Error code: %d",
        target_name.c_str(), exec_result.val
      );
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Execution completed for named target: %s", target_name.c_str());
    return true;
  }

  bool moveToPoseTargetApprox(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const geometry_msgs::msg::Pose& target_pose,
    const std::string& label,
    const std::string& end_effector_link)
  {
    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
    RCLCPP_INFO(this->get_logger(), "Moving to %s using approximate IK", label.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "%s -> pos:[%.4f, %.4f, %.4f], quat:[%.4f, %.4f, %.4f, %.4f]",
      label.c_str(),
      target_pose.position.x,
      target_pose.position.y,
      target_pose.position.z,
      target_pose.orientation.x,
      target_pose.orientation.y,
      target_pose.orientation.z,
      target_pose.orientation.w
    );

    move_group.setStartStateToCurrentState();
    move_group.clearPoseTargets();
    move_group.clearPathConstraints();

    if (!move_group.setApproximateJointValueTarget(target_pose, end_effector_link))
    {
      RCLCPP_ERROR(this->get_logger(), "Approximate IK target failed for %s", label.c_str());
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode plan_result = move_group.plan(plan);

    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Planning failed for %s. Error code: %d",
        label.c_str(), plan_result.val
      );
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Planning succeeded for %s", label.c_str());

    moveit::core::MoveItErrorCode exec_result = move_group.execute(plan);

    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Execution failed for %s. Error code: %d",
        label.c_str(), exec_result.val
      );
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Execution completed for %s", label.c_str());
    return true;
  }

public:
  void run()
  {
    const double startup_delay_sec =
      this->get_parameter("startup_delay_sec").as_double();
    const double state_wait_sec =
      this->get_parameter("state_wait_sec").as_double();
    const double pause_between_moves_sec =
      this->get_parameter("pause_between_moves_sec").as_double();

    const std::string planning_group =
      this->get_parameter("planning_group").as_string();
    const std::string pose_reference_frame =
      this->get_parameter("pose_reference_frame").as_string();
    const std::string end_effector_link =
      this->get_parameter("end_effector_link").as_string();
    const std::string home_named_target =
      this->get_parameter("home_named_target").as_string();

    const double point1_x = this->get_parameter("point1_x").as_double();
    const double point1_y = this->get_parameter("point1_y").as_double();
    const double point1_z = this->get_parameter("point1_z").as_double();
    const double point1_roll = this->get_parameter("point1_roll").as_double();
    const double point1_pitch = this->get_parameter("point1_pitch").as_double();
    const double point1_yaw = this->get_parameter("point1_yaw").as_double();

    const double point2_x = this->get_parameter("point2_x").as_double();
    const double point2_y = this->get_parameter("point2_y").as_double();
    const double point2_z = this->get_parameter("point2_z").as_double();
    const double point2_roll = this->get_parameter("point2_roll").as_double();
    const double point2_pitch = this->get_parameter("point2_pitch").as_double();
    const double point2_yaw = this->get_parameter("point2_yaw").as_double();

    const double point3_x = this->get_parameter("point3_x").as_double();
    const double point3_y = this->get_parameter("point3_y").as_double();
    const double point3_z = this->get_parameter("point3_z").as_double();
    const double point3_roll = this->get_parameter("point3_roll").as_double();
    const double point3_pitch = this->get_parameter("point3_pitch").as_double();
    const double point3_yaw = this->get_parameter("point3_yaw").as_double();

    const double planning_time =
      this->get_parameter("planning_time").as_double();
    const int num_planning_attempts =
      this->get_parameter("num_planning_attempts").as_int();

    const double velocity_scale =
      this->get_parameter("velocity_scale").as_double();
    const double acceleration_scale =
      this->get_parameter("acceleration_scale").as_double();

    const double position_tolerance =
      this->get_parameter("position_tolerance").as_double();
    const double orientation_tolerance =
      this->get_parameter("orientation_tolerance").as_double();

    const bool allow_replanning =
      this->get_parameter("allow_replanning").as_bool();

    RCLCPP_INFO(this->get_logger(), "UP -> POINT1 -> UP -> POINT2 -> UP");
    RCLCPP_INFO(this->get_logger(), "Waiting %.1f seconds before starting...", startup_delay_sec);

    std::this_thread::sleep_for(std::chrono::duration<double>(startup_delay_sec));

    moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), planning_group);

    move_group.setPlanningTime(planning_time);
    move_group.setNumPlanningAttempts(num_planning_attempts);
    move_group.setMaxVelocityScalingFactor(velocity_scale);
    move_group.setMaxAccelerationScalingFactor(acceleration_scale);
    move_group.allowReplanning(allow_replanning);

    move_group.setPoseReferenceFrame(pose_reference_frame);
    move_group.setEndEffectorLink(end_effector_link);
    move_group.setGoalPositionTolerance(position_tolerance);
    move_group.setGoalOrientationTolerance(orientation_tolerance);

    move_group.startStateMonitor();

    RCLCPP_INFO(this->get_logger(), "Waiting for valid current robot state...");
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState(state_wait_sec);

    if (!current_state)
    {
      RCLCPP_ERROR(this->get_logger(), "Could not get a valid current robot state.");
      rclcpp::shutdown();
      return;
    }

    geometry_msgs::msg::Pose point1_pose;
    point1_pose.position.x = point1_x;
    point1_pose.position.y = point1_y;
    point1_pose.position.z = point1_z;

    tf2::Quaternion q1;
    q1.setRPY(point1_roll, point1_pitch, point1_yaw);
    q1.normalize();
    point1_pose.orientation = tf2::toMsg(q1);

    geometry_msgs::msg::Pose point2_pose;
    point2_pose.position.x = point2_x;
    point2_pose.position.y = point2_y;
    point2_pose.position.z = point2_z;

    tf2::Quaternion q2;
    q2.setRPY(point2_roll, point2_pitch, point2_yaw);
    q2.normalize();
    point2_pose.orientation = tf2::toMsg(q2);

    geometry_msgs::msg::Pose point3_pose;
    point3_pose.position.x = point3_x;
    point3_pose.position.y = point3_y;
    point3_pose.position.z = point3_z;

    tf2::Quaternion q3;
    q3.setRPY(point3_roll, point3_pitch, point3_yaw);
    q3.normalize();
    point3_pose.orientation = tf2::toMsg(q3);

    if (!moveToNamedTarget(move_group, home_named_target))
    {
      rclcpp::shutdown();
      return;
    }

    // std::this_thread::sleep_for(std::chrono::duration<double>(pause_between_moves_sec));

    if (!moveToPoseTargetApprox(move_group, point1_pose, "POINT_1", end_effector_link))
    {
      rclcpp::shutdown();
      return;
    }

    /* std::this_thread::sleep_for(std::chrono::duration<double>(pause_between_moves_sec));

    if (!moveToNamedTarget(move_group, home_named_target))
    {
      rclcpp::shutdown();
      return;
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(pause_between_moves_sec));
    */
    if (!moveToPoseTargetApprox(move_group, point2_pose, "POINT_2", end_effector_link))
    {
      rclcpp::shutdown();
      return;
    }

    /*std::this_thread::sleep_for(std::chrono::duration<double>(pause_between_moves_sec));

    if (!moveToNamedTarget(move_group, home_named_target))
    {
      rclcpp::shutdown();
      return;
    } */

    if (!moveToPoseTargetApprox(move_group, point3_pose, "POINT_3", end_effector_link))
    {
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Sequence finished successfully.");
    rclcpp::shutdown();
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<EEPoseCommander>(
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  std::thread worker([node]() {
    node->run();
  });

  executor.spin();

  if (worker.joinable())
  {
    worker.join();
  }

  rclcpp::shutdown();
  return 0;
}
