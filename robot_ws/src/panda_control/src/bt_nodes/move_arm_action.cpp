#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>  // Für std::vector<double>

class MoveArmAction : public BT::SyncActionNode
{
public:
  MoveArmAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), node_(rclcpp::Node::make_shared("bt_move_arm"))
  {
    arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm");
    arm_group_->setPoseReferenceFrame("panda_link0");
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::vector<double>>("joint_angles") };
  }

  BT::NodeStatus tick() override
  {
    auto joint_angles = getInput<std::vector<double>>("joint_angles");
    
    // Arm-Bewegung
    if (joint_angles)
    {
      RCLCPP_INFO(node_->get_logger(), "Setting arm joints: %f, %f, %f, %f, %f, %f, %f", (*joint_angles)[0], (*joint_angles)[1], (*joint_angles)[2], (*joint_angles)[3], (*joint_angles)[4], (*joint_angles)[5], (*joint_angles)[6]);
      std::map<std::string, double> joint_values;
      joint_values["panda_joint1"] = (*joint_angles)[0];
      joint_values["panda_joint2"] = (*joint_angles)[1];
      joint_values["panda_joint3"] = (*joint_angles)[2];
      joint_values["panda_joint4"] = (*joint_angles)[3];
      joint_values["panda_joint5"] = (*joint_angles)[4];
      joint_values["panda_joint6"] = (*joint_angles)[5];
      joint_values["panda_joint7"] = (*joint_angles)[6];

      arm_group_->setJointValueTarget(joint_values);
      moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
      if (arm_group_->plan(joint_plan) != moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_ERROR(node_->get_logger(), "Arm planning failed!");
        return BT::NodeStatus::FAILURE;
      }
      arm_group_->move();
      RCLCPP_INFO(node_->get_logger(), "Arm moved successfully.");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "No joint_angles provided!");
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
};