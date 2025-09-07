#include "rclcpp/rclcpp.hpp"
#include "panda_control_msgs/srv/move_to_pose.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <moveit/move_group_interface/move_group_interface.h>


using std::placeholders::_1;
using std::placeholders::_2;

class MoveToPoseServer
{
public:
  MoveToPoseServer()
  {
    node_ = rclcpp::Node::make_shared("move_to_pose_server");
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm");
    move_group_interface_->setPoseReferenceFrame("panda_link0");

    service_ = node_->create_service<panda_control_msgs::srv::MoveToPose>(
      "move_to_pose",
      std::bind(&MoveToPoseServer::handle_service, this, _1, _2)
    );
    RCLCPP_INFO(node_->get_logger(), "MoveToPose service is ready.");
  }

    // Provide access to the node so it can be spun
    rclcpp::Node::SharedPtr get_node() { return node_; }

private:
  void handle_service(
    const std::shared_ptr<panda_control_msgs::srv::MoveToPose::Request> request,
    std::shared_ptr<panda_control_msgs::srv::MoveToPose::Response> response)
  {
    const auto& pose = request->target_pose;
    move_group_interface_->setPoseTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
    if (success)
    {
      // Execute the planned trajectory
      move_group_interface_->move();
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Planning failed");
    }
    RCLCPP_INFO(node_->get_logger(), "Received target pose: [%.2f, %.2f, %.2f]",
                pose.position.x, pose.position.y, pose.position.z);

    // Simulate processing
    response->success = true;
    response->message = "Moved to the target pose successfully.";

    RCLCPP_INFO(node_->get_logger(), "MoveToPose response sent.");
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<panda_control_msgs::srv::MoveToPose>::SharedPtr service_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto move_to_pose_server = std::make_shared<MoveToPoseServer>();
  executor.add_node(move_to_pose_server->get_node());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
