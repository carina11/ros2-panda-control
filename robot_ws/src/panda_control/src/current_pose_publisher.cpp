#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class PosePublisherNode
{
public:
  PosePublisherNode()
  {
    node_ = rclcpp::Node::make_shared("pose_publisher_node");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 10);

    timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PosePublisherNode::publishCurrentPose, this));
  }

  rclcpp::Node::SharedPtr get_node() { return node_; }

private:
  void publishCurrentPose()
  {
    try
    {
      // Replace these frame names as needed
      const std::string base_frame = "panda_link0";
      const std::string ee_frame = "panda_hand_tcp";

      geometry_msgs::msg::TransformStamped transform =
          tf_buffer_->lookupTransform(base_frame, ee_frame, tf2::TimePointZero);

      geometry_msgs::msg::PoseStamped pose;
      pose.header = transform.header;
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;

      pose_pub_->publish(pose);
      RCLCPP_DEBUG(node_->get_logger(), "Published TF-based current pose");
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(node_->get_logger(), "TF lookup failed: %s", ex.what());
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto pose_publisher_node = std::make_shared<PosePublisherNode>();

  rclcpp::spin(pose_publisher_node->get_node());
  rclcpp::shutdown();
  return 0;
}
