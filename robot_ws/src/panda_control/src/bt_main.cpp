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
        case 1: xml_file = "/robot_ws/install/panda_control/share/panda_control/bt/seq1_basic.xml"; break;
        case 2: xml_file = "/robot_ws/install/panda_control/share/panda_control/bt/seq2_kine_all.xml"; break;
        case 3: xml_file = "/robot_ws/install/panda_control/share/panda_control/bt/seq3_kine_mix1.xml"; break;
        case 4: xml_file = "/robot_ws/install/panda_control/share/panda_control/bt/seq4_kine_mix2.xml"; break;
        case 5: xml_file = "/robot_ws/install/panda_control/share/panda_control/bt/seq5_prox_all.xml"; break;
        case 6: xml_file = "/robot_ws/install/panda_control/share/panda_control/bt/seq6_prox_mix1.xml"; break;
        case 7: xml_file = "/robot_ws/install/panda_control/share/panda_control/bt/seq7_handover_mix.xml"; break;
        case 8: xml_file = "/robot_ws/install/panda_control/share/panda_control/bt/seq8_handover_mix2.xml"; break;
        default: xml_file = "/robot_ws/install/panda_control/share/panda_control/bt/seq1_basic.xml"; break;
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

      // //--------BASICS----------------------------------------------------------------------------------------------------------------------------
      std::vector<double> start_joints = {0.0*M_PI/180.0, -45*M_PI/180.0, 0.0, -135*M_PI/180.0, 0.0, 90*M_PI/180.0, 45*M_PI/180.0};
      blackboard->set("start_joints", start_joints);

      std::vector<double> sleepmode = {0*M_PI/180.0, -45*M_PI/180.0, 0.0, -150*M_PI/180.0, 0.0, 13*M_PI/180.0, 45*M_PI/180.0};
      blackboard->set("sleepmode", sleepmode);

      std::vector<double> rotate_left_joints = {90*M_PI/180.0, -45*M_PI/180.0, 0.0, -135*M_PI/180.0, 0.0, 90*M_PI/180.0, 45*M_PI/180.0};
      blackboard->set("rotate_left_joints", rotate_left_joints);

      std::vector<double> basic_handover_joints = {90*M_PI/180.0, 0.0, 0.0, -90*M_PI/180.0, 0.0, 90*M_PI/180.0, 45*M_PI/180.0};
      blackboard->set("basic_handover_joints", basic_handover_joints);

      //  std::vector<double> basic_handover_joints_gedreht = {90*M_PI/180.0, 0.0, 0.0, -90*M_PI/180.0, 0.0, 90*M_PI/180.0, 0.0};
      // blackboard->set("basic_handover_joints_gedreht", basic_handover_joints_gedreht);

      std::vector<double> breath = {91*M_PI/180.0, -72*M_PI/180.0, -3*M_PI/180.0, -147*M_PI/180.0, 0.0, 190*M_PI/180.0, 43*M_PI/180.0};
      blackboard->set("breath", breath);

      std::vector<double> gaze = {90*M_PI/180.0, -70*M_PI/180.0, 0.0, -145*M_PI/180.0, 0.0, 160*M_PI/180.0, 45*M_PI/180.0};
      blackboard->set("gaze", gaze);

      
      //A) KINESICS------------------------------------------------------------------------------------------------------------------------------------------------------

      // A.1) PREHANDOVER---------------------
      // //a) Nod----------------- 
      std::vector<double> nod_joints = {90*M_PI/180.0, -70*M_PI/180.0, 0.0, -145*M_PI/180.0, 0.0, 135*M_PI/180.0, 45*M_PI/180.0};      
      blackboard->set("nod_joints", nod_joints);


      // // b) Irritation------------ 
      //Only Gripper tilts 
      std::vector<double> irritation_head = {90*M_PI/180.0, -70*M_PI/180.0, 0.0, -145*M_PI/180.0, -60.0*M_PI/180.0, 155*M_PI/180.0, 45*M_PI/180.0};
      blackboard->set("irritation_head", irritation_head);
      //Including Joint3 for whole Robot to move 
      std::vector<double> irritation_full = {90*M_PI/180.0, -70*M_PI/180.0, 10.0*M_PI/180.0, -145*M_PI/180.0, -40.0*M_PI/180.0, 155*M_PI/180.0, 50*M_PI/180.0};   
      blackboard->set("irritation_full", irritation_full);


      // //c) Head Shake--------
      std::vector<double> head_shake_right = {90*M_PI/180.0, -70*M_PI/180.0, 10.0*M_PI/180.0, -145*M_PI/180.0, -20.0*M_PI/180.0, 160*M_PI/180.0, 45*M_PI/180.0};
      blackboard->set("head_shake_right", head_shake_right);
      std::vector<double> head_shake_left = {90*M_PI/180.0, -70*M_PI/180.0, -10.0*M_PI/180.0, -145*M_PI/180.0, 20.0*M_PI/180.0, 160*M_PI/180.0, 45*M_PI/180.0};
      blackboard->set("head_shake_left", head_shake_left);
      // // Head Shake with Gripper-----

      std::vector<double> head_shake_gripper_right = {90*M_PI/180.0, -70*M_PI/180.0, 0.0, -145*M_PI/180.0, 0.0*M_PI/180.0, 160*M_PI/180.0, 20*M_PI/180.0};
      blackboard->set("head_shake_gripper_right", head_shake_gripper_right);
      std::vector<double> head_shake_gripper_left = {90*M_PI/180.0, -70*M_PI/180.0, 0.0, -145*M_PI/180.0, 0.0*M_PI/180.0, 160*M_PI/180.0, 70*M_PI/180.0};
      blackboard->set("head_shake_gripper_left", head_shake_gripper_left);


      //A.2) Take Object Kinesics----------
      std::vector<double> search_joints_right = {-20*M_PI/180.0, -45*M_PI/180.0, 0.0, -135*M_PI/180.0, 0.0, 90*M_PI/180.0, 45*M_PI/180.0};
      blackboard->set("search_joints_right", search_joints_right);

      std::vector<double> search_joints_left = {20*M_PI/180.0, -45*M_PI/180.0, 0.0, -135*M_PI/180.0, 0.0, 90*M_PI/180.0, 45*M_PI/180.0};
      blackboard->set("search_joints_left", search_joints_left);

      //This object
      std::vector<double> this_object = {0.0, -45*M_PI/180.0, 0.0, -145*M_PI/180.0, 0.0, 100*M_PI/180.0, 45*M_PI/180.0};
      blackboard->set("this_object", this_object);
      // Rotate Gripper to show initative to take object
      std::vector<double> take_object_joints = {0.0, -45*M_PI/180.0, 0.0, -135*M_PI/180.0, 0.0, 100*M_PI/180.0, -45*M_PI/180.0};
      blackboard->set("take_object_joints", take_object_joints);

     
      // //-----HANDOVER PHASE------

      // //---Approaching
      
      // //handover positions
      std::vector<double> kinesics_nearer = {90*M_PI/180.0, -10*M_PI/180.0, 0*M_PI/180.0, -125*M_PI/180.0, 0.0, 120*M_PI/180.0, 45*M_PI/180.0};
      blackboard->set("kinesics_nearer", kinesics_nearer);
      std::vector<double> kinesics_handover = {90*M_PI/180.0, 30*M_PI/180.0, 0*M_PI/180.0, -82*M_PI/180.0, 0*M_PI/180.0, 185*M_PI/180.0, 45*M_PI/180.0};
      blackboard->set("kinesics_handover", kinesics_handover);

      // //Signaling Positions
      // // Transfer Signals
      std::vector<double> beckoning_gesture = {90*M_PI/180.0, 40*M_PI/180.0, 0*M_PI/180.0, -64*M_PI/180.0, 0*M_PI/180.0, 180*M_PI/180.0, 45*M_PI/180.0  };
      blackboard->set("beckoning_gesture", beckoning_gesture);

      std::vector<double> palm_presenting = {90*M_PI/180.0, 30*M_PI/180.0, 0*M_PI/180.0, -83*M_PI/180.0, -15*M_PI/180.0, 184*M_PI/180.0, 100*M_PI/180.0};
      blackboard->set("palm_presenting", palm_presenting);

      std::vector<double> mini_lift = {90*M_PI/180.0, 30*M_PI/180.0, 0*M_PI/180.0, -79*M_PI/180.0, 0*M_PI/180.0, 185*M_PI/180.0, 45*M_PI/180.0};
      blackboard->set("mini_lift", mini_lift);
      // Search Human
      std::vector<double> search_human = {90*M_PI/180.0, 30*M_PI/180.0, 0*M_PI/180.0, -82*M_PI/180.0, 0*M_PI/180.0, 185*M_PI/180.0, 13*M_PI/180.0};
      blackboard->set("search_human", search_human);
      std::vector<double> found_human = {90*M_PI/180.0, 28*M_PI/180.0, -17*M_PI/180.0, -82*M_PI/180.0, 0*M_PI/180.0, 185*M_PI/180.0, 50*M_PI/180.0};
      blackboard->set("found_human", found_human);

      // //Withhold Signal
      std::vector<double> halt = {90*M_PI/180.0, -10*M_PI/180.0, 0*M_PI/180.0, -125*M_PI/180.0, 0.0, 70*M_PI/180.0, 45*M_PI/180.0};
      blackboard->set("halt", halt);

      // //Post-Handover 

      std::vector<double> nod_after_kin_handover = {90*M_PI/180.0, 30*M_PI/180.0, 0*M_PI/180.0, -78*M_PI/180.0, 0*M_PI/180.0, 185*M_PI/180.0, 45*M_PI/180.0};
      blackboard->set("nod_after_kin_handover", nod_after_kin_handover);



      // //B)PROXEMICS---------------------------------------------------------------------------------------------------------------------------------------------------------------------

      // //B.1) PREHANDOVER----------------------------------------------------------------------------------------------------------------------------------------
      
      // //Gaze 

      std::vector<double> gaze_head = {0*M_PI/180.0, -40*M_PI/180.0, 0*M_PI/180.0, -130*M_PI/180.0, 70*M_PI/180.0, 90*M_PI/180.0, -45*M_PI/180.0};
      blackboard->set("gaze_head", gaze_head);
      // //B.2) HANDOVER PHASE------------------

      // //-----APPROACHING-PHASE--------
      std::vector<double> give_object_prox1 = {90*M_PI/180.0, 15*M_PI/180.0, 15*M_PI/180.0, -100*M_PI/180.0, -60*M_PI/180.0, 135*M_PI/180.0, 95*M_PI/180.0};
      blackboard->set("give_object_prox1", give_object_prox1);

      // //Approach Receiver
      std::vector<double> give_object_prox2 = {90*M_PI/180.0, 12*M_PI/180.0, 15*M_PI/180.0, -95*M_PI/180.0, -70*M_PI/180.0, 142*M_PI/180.0, 80*M_PI/180.0};
      blackboard->set("give_object_prox2", give_object_prox2);

       std::vector<double> give_object_prox_nod = {90*M_PI/180.0, 12*M_PI/180.0, 15*M_PI/180.0, -95*M_PI/180.0, -70*M_PI/180.0, 130*M_PI/180.0, 80*M_PI/180.0};
       blackboard->set("give_object_prox_nod", give_object_prox_nod);

      // //Straight_line from rotate left
      std::vector<double> give_object_prox_straight = {90*M_PI/180.0, 12*M_PI/180.0, 5*M_PI/180.0, -100*M_PI/180.0, 0*M_PI/180.0, 160*M_PI/180.0, 45*M_PI/180.0};
      blackboard->set("give_object_prox_straight", give_object_prox_straight);

      // //Arm Bowed
      std::vector<double> before_half_straight_arm = {50*M_PI/180.0, 30*M_PI/180.0, 50*M_PI/180.0, -76*M_PI/180.0, 15*M_PI/180.0, 110*M_PI/180.0, 60*M_PI/180.0};
      blackboard->set("before_half_straight_arm", before_half_straight_arm);

      std::vector<double> half_straight_arm = {50*M_PI/180.0, 30*M_PI/180.0, 60*M_PI/180.0, -76*M_PI/180.0, 35*M_PI/180.0, 140*M_PI/180.0, 60*M_PI/180.0};
      blackboard->set("half_straight_arm", half_straight_arm);

      // std::vector<double> straight_arm = {90*M_PI/180.0, 35*M_PI/180.0, 7*M_PI/180.0, -70*M_PI/180.0, -10*M_PI/180.0, 175*M_PI/180.0, -3*M_PI/180.0};
      // blackboard->set("straight_arm", straight_arm);
      
      // //Physical Handover--------
      
      // //Beckoning from Half ARM 
      std::vector<double> beckoning_from_half = {65*M_PI/180.0, 38*M_PI/180.0, 52*M_PI/180.0, -70*M_PI/180.0, 8*M_PI/180.0, 143*M_PI/180.0, 65*M_PI/180.0};
      blackboard->set("beckoning_from_half", beckoning_from_half);
      // //Follow Human From half_straight_arm position
      std::vector<double> follow_arm_half = {54*M_PI/180.0, 39*M_PI/180.0, 60*M_PI/180.0, -74*M_PI/180.0, 15*M_PI/180.0, 141*M_PI/180.0, 60*M_PI/180.0};
      blackboard->set("follow_arm_half", follow_arm_half);

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