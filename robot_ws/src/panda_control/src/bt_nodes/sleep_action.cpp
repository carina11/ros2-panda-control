#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>  // Für std::chrono

class SleepAction : public BT::SyncActionNode
{
public:
  SleepAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), node_(rclcpp::Node::make_shared("bt_sleep"))
  {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("duration") };  // Dauer in Sekunden (z.B. 2.0 für 2 Sekunden)
  }

  BT::NodeStatus tick() override
  {
    auto duration = getInput<double>("duration");
    if (duration)
    {
      RCLCPP_INFO(node_->get_logger(), "Sleeping for %f seconds", *duration);
     rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(*duration)));
      RCLCPP_INFO(node_->get_logger(), "Sleep finished");
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "No duration provided!");
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
};