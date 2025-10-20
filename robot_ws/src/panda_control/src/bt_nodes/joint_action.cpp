#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class MoveJointAction : public BT::SyncActionNode
{
public:
  MoveJointAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), node_(rclcpp::Node::make_shared("bt_move_joint"))
  {
    arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm");
    arm_group_->setPoseReferenceFrame("panda_link0");
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<int>("joint_index"),    // Z.B. 6 für panda_joint7 (0-basiert)
             BT::InputPort<double>("joint_value") }; // Neuer Wert in Radiant
  }

  BT::NodeStatus tick() override
  {
    auto joint_index = getInput<int>("joint_index");
    auto joint_value = getInput<double>("joint_value");

    if (joint_index && joint_value)
    {
      // Hole aktuelle Joint-Werte
      auto current_state = arm_group_->getCurrentState();
      std::vector<double> joint_values;
      current_state->copyJointGroupPositions(arm_group_->getName(), joint_values);

      // Ändere nur den gewünschten Joint
      if (*joint_index >= 0 && static_cast<size_t>(*joint_index) < joint_values.size())
      {
        joint_values[*joint_index] = *joint_value;
        RCLCPP_INFO(node_->get_logger(), "Moving joint %d to %f", *joint_index, *joint_value);

        arm_group_->setJointValueTarget(joint_values);
        moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
        if (arm_group_->plan(joint_plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
          RCLCPP_ERROR(node_->get_logger(), "Joint planning failed!");
          return BT::NodeStatus::FAILURE;
        }
        arm_group_->move();
        RCLCPP_INFO(node_->get_logger(), "Joint moved successfully.");
        return BT::NodeStatus::SUCCESS;
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Invalid joint_index: %d", *joint_index);
        return BT::NodeStatus::FAILURE;
      }
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "No joint_index or joint_value provided!");
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
};