#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class GripperAction : public BT::SyncActionNode
{
public:
  GripperAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), node_(rclcpp::Node::make_shared("bt_gripper"))
  {
    gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "gripper");
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("finger1"),  // Wert für panda_finger_joint1
             BT::InputPort<double>("finger2") }; // Wert für panda_finger_joint2
  }

  BT::NodeStatus tick() override
  {
    auto finger1 = getInput<double>("finger1");
    auto finger2 = getInput<double>("finger2");

    if (finger1 && finger2)
    {
      RCLCPP_INFO(node_->get_logger(), "Setting gripper fingers: finger1=%f, finger2=%f", *finger1, *finger2);
      std::map<std::string, double> gripper_joints;
      gripper_joints["panda_finger_joint1"] = *finger1;
      gripper_joints["panda_finger_joint2"] = *finger2;
      gripper_group_->setJointValueTarget(gripper_joints);
      moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
      if (gripper_group_->plan(gripper_plan) != moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_ERROR(node_->get_logger(), "Gripper planning failed!");
        return BT::NodeStatus::FAILURE;
      }
      gripper_group_->move();
      RCLCPP_INFO(node_->get_logger(), "Gripper moved successfully.");
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "No finger1 or finger2 provided!");
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
};