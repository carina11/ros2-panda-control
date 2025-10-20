#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <vector>
#include <map>
#include <memory>
#include <string>

class MoveArmAction : public BT::SyncActionNode
{
public:
  // Default ctor delegates to node-accepting ctor (creates unique node if none provided)
  MoveArmAction(const std::string &name, const BT::NodeConfiguration &config)
    : MoveArmAction(name, config, nullptr)
  {}

  // Ctor that accepts a shared rclcpp::Node to avoid duplicate publishers
  MoveArmAction(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config), node_(node)
  {
    if (!node_) {
      // unique node name per instance to avoid publisher collisions
      std::string node_name = "bt_move_arm_" + std::to_string(reinterpret_cast<uintptr_t>(this));
      node_ = rclcpp::Node::make_shared(node_name);
    }

    // declare params with defaults (can be set via launch)
    node_->declare_parameter("velocity_scale", 0.1);
    node_->declare_parameter("accel_scale", 0.1);
    node_->declare_parameter("move_time", 2.0);
    node_->declare_parameter("gripper_time", 3.0);

    node_->get_parameter("velocity_scale", default_vel_);
    node_->get_parameter("accel_scale", default_acc_);
    node_->get_parameter("move_time", default_move_time_);
    node_->get_parameter("gripper_time", default_gripper_time_);

    // MoveIt MoveGroupInterfaces
    arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm");
    arm_group_->setPoseReferenceFrame("panda_link0");
    gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "gripper");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<double>>("joint_angles"),
      BT::InputPort<double>("velocity_scale"),   // optional override 0..1
      BT::InputPort<double>("accel_scale"),      // optional override 0..1
      BT::InputPort<double>("move_time"),        // optional override seconds (planning time)
      BT::InputPort<double>("gripper_f1"),       // optional: gripper target joint 1
      BT::InputPort<double>("gripper_f2"),       // optional: gripper target joint 2
      BT::InputPort<double>("gripper_time")      // optional per-gripper override seconds
    };
  }

  BT::NodeStatus tick() override
  {
    // Read inputs once
    auto joint_angles = getInput<std::vector<double>>("joint_angles");
    auto port_vel = getInput<double>("velocity_scale");
    auto port_acc = getInput<double>("accel_scale");
    auto port_move_time = getInput<double>("move_time");
    auto gf1 = getInput<double>("gripper_f1");
    auto gf2 = getInput<double>("gripper_f2");
    auto port_grip_time = getInput<double>("gripper_time");

    if (!joint_angles) {
      RCLCPP_ERROR(node_->get_logger(), "MoveArmAction: No joint_angles provided");
      return BT::NodeStatus::FAILURE;
    }
    if (joint_angles->size() < 7) {
      RCLCPP_ERROR(node_->get_logger(), "MoveArmAction: joint_angles must contain 7 values");
      return BT::NodeStatus::FAILURE;
    }

    // resolve motion params: ports override node params
    double vel = default_vel_;
    double acc = default_acc_;
    double move_time = default_move_time_;
    if (port_vel) vel = *port_vel;
    if (port_acc) acc = *port_acc;
    if (port_move_time) move_time = *port_move_time;

    arm_group_->setMaxVelocityScalingFactor(vel);
    arm_group_->setMaxAccelerationScalingFactor(acc);
    arm_group_->setPlanningTime(move_time);

    // set arm joint targets
    std::map<std::string, double> joint_values;
    joint_values["panda_joint1"] = (*joint_angles)[0];
    joint_values["panda_joint2"] = (*joint_angles)[1];
    joint_values["panda_joint3"] = (*joint_angles)[2];
    joint_values["panda_joint4"] = (*joint_angles)[3];
    joint_values["panda_joint5"] = (*joint_angles)[4];
    joint_values["panda_joint6"] = (*joint_angles)[5];
    joint_values["panda_joint7"] = (*joint_angles)[6];

    arm_group_->setJointValueTarget(joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    auto plan_res = arm_group_->plan(arm_plan);
    if (plan_res != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "MoveArmAction: Arm planning failed");
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "MoveArmAction: Arm plan OK, executing...");
    auto exec_res = arm_group_->execute(arm_plan);
    if (exec_res != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "MoveArmAction: Arm execute failed");
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(node_->get_logger(), "MoveArmAction: Arm moved successfully");

    // optional gripper execution if both ports provided
    if (gf1 && gf2) {
      double f1 = *gf1;
      double f2 = *gf2;

      RCLCPP_INFO(node_->get_logger(), "MoveArmAction: Executing gripper f1=%f f2=%f", f1, f2);

      std::map<std::string, double> gripper_joints;
      gripper_joints["panda_finger_joint1"] = f1;
      gripper_joints["panda_finger_joint2"] = f2;
      gripper_group_->setJointValueTarget(gripper_joints);

      // gripper params: reuse arm scaling unless separate provided
      double g_vel = vel;
      double g_acc = acc;
      double g_time = default_gripper_time_;
      if (port_grip_time) g_time = *port_grip_time;

      gripper_group_->setMaxVelocityScalingFactor(g_vel);
      gripper_group_->setMaxAccelerationScalingFactor(g_acc);
      gripper_group_->setPlanningTime(g_time);

      moveit::planning_interface::MoveGroupInterface::Plan grip_plan;
      auto grip_plan_res = gripper_group_->plan(grip_plan);
      if (grip_plan_res != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "MoveArmAction: Gripper planning failed");
        return BT::NodeStatus::FAILURE;
      }
      RCLCPP_INFO(node_->get_logger(), "MoveArmAction: Gripper plan OK, executing...");
      auto grip_exec_res = gripper_group_->execute(grip_plan);
      if (grip_exec_res != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "MoveArmAction: Gripper execute failed");
        return BT::NodeStatus::FAILURE;
      }
      RCLCPP_INFO(node_->get_logger(), "MoveArmAction: Gripper moved successfully");
    }

    return BT::NodeStatus::SUCCESS;
  }

  // helper to allow factory registration with a shared node
  static void RegisterWithFactory(BT::BehaviorTreeFactory &factory, rclcpp::Node::SharedPtr node)
  {
    factory.registerBuilder<MoveArmAction>("MoveArmAction",
      [node](const std::string &name, const BT::NodeConfiguration &cfg) {
        return std::make_unique<MoveArmAction>(name, cfg, node);
      });
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;

  double default_vel_{0.1};
  double default_acc_{0.1};
  double default_move_time_{2.0};
  double default_gripper_time_{3.0};
};

// No global code here - class only. Register in bt_main