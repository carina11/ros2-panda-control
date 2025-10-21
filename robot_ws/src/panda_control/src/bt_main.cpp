#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/blackboard.h>
#include <panda_control_msgs/srv/start_sequence.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <vector>
#include <string>
#include <memory>
#include <cmath> // für M_PI

#include "bt_nodes/move_arm_action.cpp"
#include "bt_nodes/sleep_action.cpp"
#include "bt_nodes/gripper_action.cpp"
#include "bt_nodes/joint_action.cpp"  


class BTService : public rclcpp::Node
{
public:
  BTService() : Node("bt_main")
  {
    // Declare parameters
    this->declare_parameter("bt_xml_file", "sequence1.xml");

    // Service to start sequence
    service_ = this->create_service<panda_control_msgs::srv::StartSequence>(
      "start_sequence", std::bind(&BTService::handle_request, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "BT Service ready. Call /start_sequence with sequence_id");
  }

private:
  void handle_request(const std::shared_ptr<panda_control_msgs::srv::StartSequence::Request> request,
                      std::shared_ptr<panda_control_msgs::srv::StartSequence::Response> response)
  {
    try {
      // Select XML based on sequence_id
      std::string xml_file;
      switch (request->sequence_id) {
        case 1: xml_file = "/robot_ws/install/panda_control/share/panda_control/bt/seq1_complete.xml"; break;
        case 2: xml_file = "/robot_ws/install/panda_control/share/panda_control/bt/seq2_complete.xml"; break;
        case 3: xml_file = "/robot_ws/install/panda_control/share/panda_control/bt/seq3_basic.xml"; break;
        case 4: xml_file = "/robot_ws/install/panda_control/share/panda_control/bt/seq4_kine.xml"; break;
        case 5: xml_file = "/robot_ws/install/panda_control/share/panda_control/bt/seq5_kine.xml"; break;
        case 6: xml_file = "/robot_ws/install/panda_control/share/panda_control/bt/seq6_prox.xml"; break;
        case 7: xml_file = "/robot_ws/install/panda_control/share/panda_control/bt/seq7_prox.xml"; break;
        default: xml_file = "/robot_ws/install/panda_control/share/panda_control/bt/seq1_complete.xml"; break;
      }

      // Factory ohne shared node
          BT::BehaviorTreeFactory factory;
          factory.registerNodeType<MoveArmAction>("MoveArmAction");
          factory.registerNodeType<SleepAction>("SleepAction");
          factory.registerNodeType<GripperAction>("GripperAction");
          factory.registerNodeType<MoveJointAction>("MoveJointAction");
      // Create tree
      auto tree = factory.createTreeFromFile(xml_file);
      auto blackboard = tree.rootBlackboard();

      // Define joint poses and set blackboard (nach tree-Erstellung)
      std::vector<double> start_joints = {0.0*M_PI/180.0, -45*M_PI/180.0, 0.0, -135*M_PI/180.0, 0.0, 90*M_PI/180.0, 45*M_PI/180.0};
      blackboard->set("start_joints", start_joints);
        
      std::vector<double> rotate_left_joints = {90*M_PI/180.0, -45*M_PI/180.0, 0.0, -135*M_PI/180.0, 0.0, 90*M_PI/180.0, 45*M_PI/180.0};
      blackboard->set("rotate_left_joints", rotate_left_joints);

      //NICKEN 
      std::vector<double> nod_joints = {90*M_PI/180.0, -80*M_PI/180.0, 0.0, -160*M_PI/180.0, 0.0, 130*M_PI/180.0, 45*M_PI/180.0};
      blackboard->set("nod_joints", nod_joints);

      std::vector<double> nod_joints2 = {90*M_PI/180.0, -80*M_PI/180.0, 0.0, -160*M_PI/180.0, 0.0, 120*M_PI/180.0, 45*M_PI/180.0};
      blackboard->set("nod_joints2", nod_joints2);
      //NICKEN VORBEI

      //BEND FORWARD
      std::vector<double> bend_forward_joints = {0.0, 30*M_PI/180.0, 0.0, -130*M_PI/180.0, 0.0, 160*M_PI/180.0, 45*M_PI/180.0};
      blackboard->set("bend_forward_joints", bend_forward_joints);
      //This object
      std::vector<double> this_object = {0.0, 15*M_PI/180.0, 0.0, -130*M_PI/180.0, 0.0, 150*M_PI/180.0, 45*M_PI/180.0};
      blackboard->set("this_object", this_object);

      //tAKE OBJECT
      std::vector<double> take_object_joints = {0.0, 30*M_PI/180.0, 0.0, -130*M_PI/180.0, 0.0, 160*M_PI/180.0, -45*M_PI/180.0};
      blackboard->set("take_object_joints", take_object_joints);

      //TAKE OBJECT Kinesics
      std::vector<double> take_object_kinesics = {90*M_PI/180.0, 15*M_PI/180.0, 15*M_PI/180.0, -100*M_PI/180.0, -60*M_PI/180.0, 135*M_PI/180.0, 80*M_PI/180.0};
      blackboard->set("take_object_kinesics", take_object_kinesics);

      //Take OBJECT Kinesics erweitert 
      std::vector<double> take_object_kinesics_erweitert = {90*M_PI/180.0, 12*M_PI/180.0, 12*M_PI/180.0, -110*M_PI/180.0, -45*M_PI/180.0, 145*M_PI/180.0, 96*M_PI/180.0};
      blackboard->set("take_object_kinesics_erweitert", take_object_kinesics_erweitert);

      std::vector<double> take_object_kinesics_node = {90*M_PI/180.0, 12*M_PI/180.0, 12*M_PI/180.0, -110*M_PI/180.0, -45*M_PI/180.0, 145*M_PI/180.0, 86*M_PI/180.0};
      blackboard->set("take_object_kinesics_node", take_object_kinesics_node);
      //other
      std::vector<double> basic_handover_joints = {90*M_PI/180.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0};
      blackboard->set("basic_handover_joints", basic_handover_joints);

      std::vector<double> basic_handover_joints_gedreht = {90*M_PI/180.0, 0.0, 0.0, -1.57, 0.0, 1.57, 45*M_PI/180.0};
      blackboard->set("basic_handover_joints_gedreht", basic_handover_joints_gedreht);

      std::vector<double> proxemics_nearer = {90*M_PI/180.0, 5*M_PI/180.0, 10*M_PI/180.0, -135*M_PI/180.0, 0.0, 165*M_PI/180.0, 45*M_PI/180.0};
      blackboard->set("proxemics_nearer", proxemics_nearer);
      std::vector<double> proxemics_handover = {90*M_PI/180.0, 20*M_PI/180.0, 10*M_PI/180.0, -115*M_PI/180.0, -10*M_PI/180.0, 160*M_PI/180.0, 45*M_PI/180.0};
      blackboard->set("proxemics_handover", proxemics_handover);
     

      // Tick once
      BT::NodeStatus status = tree.tickRoot();
      if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Sequence %d executed successfully", request->sequence_id);
        response->success = true;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Sequence %d failed with status: %s", request->sequence_id, BT::toStr(status).c_str());
        response->success = false;
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in sequence %d: %s", request->sequence_id, e.what());
      response->success = false;
    }
  }

  rclcpp::Service<panda_control_msgs::srv::StartSequence>::SharedPtr service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BTService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}