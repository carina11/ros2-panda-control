#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>
#include <map>
#include <memory>
#include <string>

class GripperAction : public BT::SyncActionNode
{
public:
  // Default ctor delegates to node-accepting ctor (creates unique node if none provided)
  GripperAction(const std::string& name, const BT::NodeConfiguration& config)
    : GripperAction(name, config, nullptr)
  {}

  // Ctor that accepts a shared rclcpp::Node to avoid duplicate publishers
  GripperAction(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config), node_(node)
  {
    if (!node_) {
      // unique node name per instance to avoid publisher collisions
      std::string node_name = "bt_gripper_" + std::to_string(reinterpret_cast<uintptr_t>(this));
      node_ = rclcpp::Node::make_shared(node_name);
    }

    gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "gripper");
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("finger1"),        // Wert für panda_finger_joint1
             BT::InputPort<double>("finger2"),        // Wert für panda_finger_joint2
             BT::InputPort<double>("gripper_time") }; // Planning time in seconds
  }

  BT::NodeStatus tick() override
  {
    auto finger1 = getInput<double>("finger1");
    auto finger2 = getInput<double>("finger2");
    if (!finger1 || !finger2)
    {
      RCLCPP_ERROR(node_->get_logger(), "finger1 and finger2 must be provided!");
      return BT::NodeStatus::FAILURE;
    }

    double f1 = *finger1;
    double f2 = *finger2;

    // gripper_time lesen und setzen
    double gripper_time = 3.0; // default
    if (auto gt = getInput<double>("gripper_time")) gripper_time = *gt;
    gripper_group_->setPlanningTime(gripper_time);

    RCLCPP_INFO(node_->get_logger(), "Setting gripper fingers: finger1=%f, finger2=%f, planning_time=%f", f1, f2, gripper_time);
    std::map<std::string, double> gripper_joints;
    gripper_joints["panda_finger_joint1"] = f1;
    gripper_joints["panda_finger_joint2"] = f2;
    gripper_group_->setJointValueTarget(gripper_joints);
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    if (gripper_group_->plan(gripper_plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "Gripper planning failed!");
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(node_->get_logger(), "Executing gripper move...");
    auto result = gripper_group_->move();
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(node_->get_logger(), "Gripper move executed successfully.");
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Gripper move failed with code: %d", result.val);
    }
    return BT::NodeStatus::SUCCESS;
  }

  // helper to allow factory registration with a shared node
  static void RegisterWithFactory(BT::BehaviorTreeFactory &factory, rclcpp::Node::SharedPtr node)
  {
    factory.registerBuilder<GripperAction>("GripperAction",
      [node](const std::string &name, const BT::NodeConfiguration &cfg) {
        return std::make_unique<GripperAction>(name, cfg, node);
      });
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
};