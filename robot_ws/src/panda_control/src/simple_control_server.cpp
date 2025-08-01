#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>
#include <functional>
#include <cmath>
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class ArmPosePublisher
{
public:
  ArmPosePublisher()
  {
    // Create a shared node instance
    node_ = rclcpp::Node::make_shared("pose_stamped_publisher");

    // Create a publisher for PoseStamped messages
    publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("panda/pose", 10);

    // Create the MoveGroupInterface using the node instance
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm");

    node_->declare_parameter("height", 100);
    node_->declare_parameter("width", 100);

    int height, width;

    node_->get_parameter("height", height);
    node_->get_parameter("width", width);
   

    joint_state_subscriber = node_->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10,
        [this](sensor_msgs::msg::JointState::SharedPtr msg){
            for (size_t i = 0; i < msg->name.size(); i++){
                joint_values[msg->name[i]] = msg->position[i];
                RCLCPP_INFO(node_->get_logger(), "Joint: %s | Position: %.3f",
                    msg->name[i].c_str(), msg->position[i]);
            }
        }
    );

    set_initial_pose();

    control_subscriber = node_->create_subscription<geometry_msgs::msg::PoseStamped>("control", 10, 
        [this, height, width](geometry_msgs::msg::PoseStamped::SharedPtr msg){
            // RCLCPP_WARN(node_->get_logger(), "Hello");
            
            move_to_point(msg);

            // std::map<std::string, double> goal_values;
            // std::string command_horizontal, command_vertical; 
            // if (msg->pose.position.x < width / 2){
            //   command_horizontal = "left";
            // }else{
            //   command_horizontal = "right";
            // }
            // if (msg->pose.position.y < height / 2){
            //   command_vertical = "up";
            // }else{
            //   command_vertical = "down";
            // }

            // if(command_horizontal == "left"){
            //     goal_values["panda_joint1"] = joint_values["panda_joint1"] + (5.0 * M_PI / 180.0);
            // }else if(command_horizontal == "right"){
            //     goal_values["panda_joint1"] = joint_values["panda_joint1"] - (5.0 * M_PI / 180.0);
            // } 
            // if(command_vertical == "up") {
            //     goal_values["panda_joint6"] = joint_values["panda_joint6"] + (5.0 * M_PI / 180.0);
            // }else if(command_vertical == "down") {
            //     goal_values["panda_joint6"] = joint_values["panda_joint6"] - (5.0 * M_PI / 180.0);
            // }
            // // goal_values["panda_joint1"] = ;
            // goal_values["panda_joint2"] = -65.0 * M_PI / 180.0;
            // goal_values["panda_joint3"] = 0.0 * M_PI / 180.0;
            // goal_values["panda_joint4"] = -95.0 * M_PI / 180.0;
            // goal_values["panda_joint5"] = 0.0 * M_PI / 180.0;
            // // goal_values["panda_joint6"] = 120.0 * M_PI / 180.0;
            // goal_values["panda_joint7"] = 45.0 * M_PI / 180.0;
            // set_joints(goal_values);
        }
    );
  }

  // Provide access to the node so it can be spun
  rclcpp::Node::SharedPtr get_node() { return node_; }

private:
  void set_initial_pose() {
    std::map<std::string, double> goal_values;
    goal_values["panda_joint1"] = 0.0;
    goal_values["panda_joint2"] = -65.0 * M_PI / 180.0;
    goal_values["panda_joint3"] = 0.0 * M_PI / 180.0;
    goal_values["panda_joint4"] = -95.0 * M_PI / 180.0;
    goal_values["panda_joint5"] = 0.0 * M_PI / 180.0;
    goal_values["panda_joint6"] = 120.0 * M_PI / 180.0;
    goal_values["panda_joint7"] = 45.0 * M_PI / 180.0;
    set_joints(goal_values);
  }

  void move_to_point(geometry_msgs::msg::PoseStamped::SharedPtr msg){
    geometry_msgs::msg::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = msg->pose.position.x;
    target_pose1.position.y = msg->pose.position.y;
    target_pose1.position.z = msg->pose.position.z;
    move_group_interface_->setPoseTarget(target_pose1);
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
  }

  void set_joints(std::map<std::string, double> goal_values){
    move_group_interface_->setJointValueTarget(goal_values);

    // Plan the motion to the target joint values
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
  }

  // Members holding the node, publisher, timer, and MoveGroupInterface
  rclcpp::Node::SharedPtr node_;
  std::map<std::string, double> joint_values;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr control_subscriber;


  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;


  // Create our ArmPosePublisher instance
  auto arm_pose_publisher = std::make_shared<ArmPosePublisher>();
  executor.add_node(arm_pose_publisher->get_node());
  executor.spin();


  // Spin the node owned by our instance
//   rclcpp::spin(arm_pose_publisher->get_node());

  rclcpp::shutdown();
  return 0;
}
