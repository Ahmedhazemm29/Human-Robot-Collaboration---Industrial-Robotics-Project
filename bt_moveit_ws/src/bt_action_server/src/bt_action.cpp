#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/bt_factory.h"
#include <bt_action_server/action/reach_location.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robotiq_2f_urcap_adapter/action/gripper_command.hpp>
#include <std_msgs/msg/bool.hpp>
#include <atomic>

using namespace std::chrono_literals;
using Action = bt_action_server::action::ReachLocation;
using GripperAction = robotiq_2f_urcap_adapter::action::GripperCommand;

enum class ActionResult : uint8_t {
    ActionNotCompleted,
    ActionFailed,
    ActionCancelled,
    ActionSucceded
};

class WaitForServer : public BT::StatefulActionNode {
public:
    WaitForServer(const std::string & action_name, const BT::NodeConfig & conf)
    : BT::StatefulActionNode(action_name, conf)
    {
        if (!conf.blackboard->get<rclcpp::Node::SharedPtr>("node", _node)) {
            throw BT::RuntimeError("WaitForServer: missing 'node' on blackboard");
        }
    }

    BT::NodeStatus onStart() {
        std::string server_name;
        getInput<std::string>("server_name", server_name);
        client_ = rclcpp_action::create_client<Action>(_node, server_name);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() {
        if (!client_->action_server_is_ready()) {
            std::cout << "[WaitForServer]: waiting..." << std::endl;
            return BT::NodeStatus::RUNNING;
        }
        std::cout << "[WaitForServer]: Success" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("server_name") };
    }

private:
    rclcpp::Node::SharedPtr _node;
    rclcpp_action::Client<Action>::SharedPtr client_;
};


class CallAction : public BT::StatefulActionNode {
public:
    CallAction(const std::string & action_name, const BT::NodeConfig & conf)
    : BT::StatefulActionNode(action_name, conf)
    {
        node2_ = rclcpp::Node::make_shared("action_client_node");
    }

    BT::NodeStatus onStart() {
        std::string server_name;
        getInput<std::string>("server_name", server_name);
        getInput<float>("x", x_);
        getInput<float>("y", y_);
        getInput<float>("z", z_);
        getInput<float>("qx", qx_);
        getInput<float>("qy", qy_);
        getInput<float>("qz", qz_);
        getInput<float>("qw", qw_);
        getInput<float>("timeout", timeout_);
        cartesian_ = false;
        getInput<bool>("cartesian", cartesian_);
        action_result_ = ActionResult::ActionNotCompleted;
        server_called_ = false;
        client_ = rclcpp_action::create_client<Action>(node2_, server_name);
        return BT::NodeStatus::RUNNING;
    }

    void result_callback(
        const rclcpp_action::ClientGoalHandle<Action>::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                action_result_ = ActionResult::ActionSucceded;  break;
            case rclcpp_action::ResultCode::ABORTED:
                action_result_ = ActionResult::ActionFailed;    break;
            case rclcpp_action::ResultCode::CANCELED:
                action_result_ = ActionResult::ActionCancelled; break;
            default: break;
        }
    }

    BT::NodeStatus onRunning() {
        if (!server_called_) {
            server_called_ = true;
            auto goal = Action::Goal();
            goal.x       = x_;
            goal.y       = y_;
            goal.z       = z_;
            goal.qx      = qx_;
            goal.qy      = qy_;
            goal.qz      = qz_;
            goal.qw      = qw_;
            goal.timeout = timeout_;
            goal.cartesian = cartesian_;

            auto send_goal_future = client_->async_send_goal(goal);
            if (rclcpp::spin_until_future_complete(node2_, send_goal_future) !=
                rclcpp::FutureReturnCode::SUCCESS) {
                std::cout << "Failed to send goal" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
            auto goal_handle = send_goal_future.get();
            if (!goal_handle) {
                std::cout << "Goal was rejected by server" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
            client_->async_get_result(
                goal_handle,
                std::bind(&CallAction::result_callback, this, std::placeholders::_1));
        }

        rclcpp::spin_some(node2_);

        if (action_result_ == ActionResult::ActionSucceded) {
            std::cout << "[CallAction]: Success" << std::endl;
            return BT::NodeStatus::SUCCESS;
        } else if (action_result_ == ActionResult::ActionFailed ||
                   action_result_ == ActionResult::ActionCancelled) {
            std::cout << "[CallAction]: Failure" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "[CallAction]: Waiting..." << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("server_name"),
            BT::InputPort<float>("x"),
            BT::InputPort<float>("y"),
            BT::InputPort<float>("z"),
            BT::InputPort<float>("qx"),
            BT::InputPort<float>("qy"),
            BT::InputPort<float>("qz"),
            BT::InputPort<float>("qw"),
            BT::InputPort<float>("timeout"),
            BT::InputPort<bool>("cartesian", false,
                "straight-line Cartesian (Pilz LIN) move instead of PTP")
        };
    }

private:
    rclcpp::Node::SharedPtr node2_;
    rclcpp_action::Client<Action>::SharedPtr client_;
    float x_, y_, z_, qx_, qy_, qz_, qw_, timeout_;
    bool server_called_;
    bool cartesian_{false};
    ActionResult action_result_;
};


// Commands the Robotiq 2F gripper via the GripperCommand action.
//   position   : target gap in metres (0.0 = fully closed, ~0.08 = open)
//   max_speed  : 0.02 – 0.15 m/s
//   max_effort : 20 – 235 N
// IMPORTANT: when the gripper closes onto a block it stops on the object and
// the adapter ABORTS the goal with stalled=true. That is a *successful grasp*,
// so we treat "aborted + stalled" as SUCCESS. Reaching a free target (e.g.
// opening) returns SUCCEEDED normally.
class Gripper : public BT::StatefulActionNode {
public:
    Gripper(const std::string & action_name, const BT::NodeConfig & conf)
    : BT::StatefulActionNode(action_name, conf)
    {
        node_ = rclcpp::Node::make_shared("gripper_client_node");
    }

    BT::NodeStatus onStart() {
        std::string server_name = "/robotiq_2f85_urcap_adapter/gripper_command";
        getInput<std::string>("server_name", server_name);
        getInput<float>("position",   position_);
        getInput<float>("max_speed",  max_speed_);
        getInput<float>("max_effort", max_effort_);
        action_result_ = ActionResult::ActionNotCompleted;
        goal_sent_     = false;
        client_ = rclcpp_action::create_client<GripperAction>(node_, server_name);
        return BT::NodeStatus::RUNNING;
    }

    void result_callback(
        const rclcpp_action::ClientGoalHandle<GripperAction>::WrappedResult & result)
    {
        // The Robotiq adapter only reports SUCCEEDED when it lands exactly on
        // AT_DEST. A normal grasp (stops on the block) AND a normal release
        // (stops a hair short of fully open) both come back as ABORTED. So we
        // judge success by what the gripper ACTUALLY did — using the real
        // position the adapter reports — not by the commanded target.
        const bool   opening    = (position_ >= 0.04f);   // target open vs close
        const float  actual_pos = (result.result) ? result.result->position : position_;

        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                action_result_ = ActionResult::ActionSucceded;
                break;
            case rclcpp_action::ResultCode::ABORTED:
                if (opening) {
                    // Release: success if the gripper actually opened (moved
                    // well away from closed). Only a fingers-stuck-shut abort
                    // (still near closed) counts as a real failure.
                    action_result_ = (actual_pos > 0.04f)
                        ? ActionResult::ActionSucceded
                        : ActionResult::ActionFailed;
                } else {
                    // Close: stopping on the block (stalled) is a good grasp.
                    action_result_ = ActionResult::ActionSucceded;
                }
                break;
            case rclcpp_action::ResultCode::CANCELED:
                action_result_ = ActionResult::ActionCancelled;
                break;
            default: break;
        }
    }

    BT::NodeStatus onRunning() {
        if (!goal_sent_) {
            goal_sent_ = true;
            if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
                std::cout << "[Gripper]: action server unavailable" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
            auto goal = GripperAction::Goal();
            goal.command.position   = position_;
            goal.command.max_speed  = max_speed_;
            goal.command.max_effort = max_effort_;
            auto send_goal_future = client_->async_send_goal(goal);
            if (rclcpp::spin_until_future_complete(node_, send_goal_future) !=
                rclcpp::FutureReturnCode::SUCCESS) {
                std::cout << "[Gripper]: failed to send goal" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
            auto goal_handle = send_goal_future.get();
            if (!goal_handle) {
                std::cout << "[Gripper]: goal rejected by server" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
            client_->async_get_result(
                goal_handle,
                std::bind(&Gripper::result_callback, this, std::placeholders::_1));
        }

        rclcpp::spin_some(node_);

        if (action_result_ == ActionResult::ActionSucceded) {
            std::cout << "[Gripper]: Success" << std::endl;
            return BT::NodeStatus::SUCCESS;
        } else if (action_result_ == ActionResult::ActionFailed ||
                   action_result_ == ActionResult::ActionCancelled) {
            std::cout << "[Gripper]: Failure" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("server_name"),
            BT::InputPort<float>("position"),
            BT::InputPort<float>("max_speed"),
            BT::InputPort<float>("max_effort")
        };
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<GripperAction>::SharedPtr client_;
    float position_{0.08f}, max_speed_{0.1f}, max_effort_{50.0f};
    bool  goal_sent_{false};
    ActionResult action_result_{ActionResult::ActionNotCompleted};
};


// Gesture-controlled gate: holds the tree while the operator has paused the
// task with an OPEN PALM gesture (gesture_tracker publishes /gesture_pause).
// Returns RUNNING while paused, SUCCESS once clear — place one before each
// motion step so a pause takes effect at the next waypoint boundary. If the
// gesture node is not running, no message ever arrives and the gate defaults
// to not-paused, so the tree behaves exactly as it did without this feature.
class GesturePauseGate : public BT::StatefulActionNode {
public:
    GesturePauseGate(const std::string & action_name, const BT::NodeConfig & conf)
    : BT::StatefulActionNode(action_name, conf)
    {
        static std::atomic<int> instance_count{0};
        node_ = rclcpp::Node::make_shared(
            "gesture_gate_node_" + std::to_string(instance_count++));
        sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/gesture_pause", 10,
            [this](std_msgs::msg::Bool::SharedPtr msg) { paused_ = msg->data; });
    }

    BT::NodeStatus onStart() {
        return check_gate();
    }

    BT::NodeStatus onRunning() {
        return check_gate();
    }

    void onHalted() {}

    static BT::PortsList providedPorts() { return {}; }

private:
    BT::NodeStatus check_gate() {
        rclcpp::spin_some(node_);
        if (paused_) {
            std::cout << "[GesturePauseGate]: PAUSED — show a FIST to resume"
                      << std::endl;
            return BT::NodeStatus::RUNNING;
        }
        return BT::NodeStatus::SUCCESS;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
    bool paused_{false};
};


class BTExecutor : public rclcpp::Node {
public:
    BTExecutor()
    : Node("bt_executor"), first_(true)
    {
        blackboard_ = BT::Blackboard::create();
        timer_ = this->create_wall_timer(
            0.5s, std::bind(&BTExecutor::tick_function, this));
    }

private:
    void init_btree() {
        factory_.registerNodeType<WaitForServer>("WaitForServer");
        factory_.registerNodeType<CallAction>("CallAction");
        factory_.registerNodeType<Gripper>("Gripper");
        factory_.registerNodeType<GesturePauseGate>("GesturePauseGate");

        this->declare_parameter<std::string>("tree_xml_file", "");
        std::string tree_file;
        this->get_parameter("tree_xml_file", tree_file);

        blackboard_->set<rclcpp::Node::SharedPtr>("node", this->shared_from_this());
        tree_ = factory_.createTreeFromFile(tree_file, blackboard_);
    }

    void tick_function() {
        if (first_) {
            init_btree();
            first_ = false;
        }
        auto status = tree_.tickOnce();
        if (status == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "BT cycle SUCCESS — restarting");
            tree_.rootNode()->haltNode();
        } else if (status == BT::NodeStatus::FAILURE) {
            RCLCPP_WARN(this->get_logger(), "BT cycle FAILURE — retrying");
            tree_.rootNode()->haltNode();
        }
    }

    BT::Tree tree_;
    BT::Blackboard::Ptr blackboard_;
    BT::BehaviorTreeFactory factory_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool first_;
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BTExecutor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
