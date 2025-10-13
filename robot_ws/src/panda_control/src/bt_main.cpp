#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/xml_parsing.h>
#include <rclcpp/rclcpp.hpp>
#include <panda_control_msgs/srv/start_sequence.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include "bt_nodes/move_arm_action.cpp"
#include "bt_nodes/sleep_action.cpp"
#include "bt_nodes/gripper_action.cpp"

class BTService : public rclcpp::Node
{
public:
  BTService() : Node("bt_service")
  {
    service_ = create_service<panda_control_msgs::srv::StartSequence>(
      "start_sequence",
      std::bind(&BTService::handle_request, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "BT Service bereit. Rufe mit 'ros2 service call /start_sequence panda_control_msgs/srv/StartSequence \"sequence_id:<id>\"' auf.");
  }

private:
  void handle_request(const std::shared_ptr<panda_control_msgs::srv::StartSequence::Request> request,
                      std::shared_ptr<panda_control_msgs::srv::StartSequence::Response> response)
  {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<MoveArmAction>("MoveArmAction");
    factory.registerNodeType<SleepAction>("SleepAction");
    factory.registerNodeType<GripperAction>("GripperAction");
    // Bestimme XML-Datei basierend auf sequence_id
    std::string xml_file;
    if (request->sequence_id == 1) {
      xml_file = "/robot_ws/install/panda_control/share/panda_control/bt/sequence1.xml";
    } else if (request->sequence_id == 2) {
      xml_file = "/robot_ws/install/panda_control/share/panda_control/bt/sequence2.xml";
    } else if (request->sequence_id == 3) {
      xml_file = "/robot_ws/install/panda_control/share/panda_control/bt/sequence3.xml";
    } else {
      response->success = false;
      response->message = "Unbekannte Sequence-ID: " + std::to_string(request->sequence_id);
      RCLCPP_ERROR(get_logger(), "Unbekannte Sequence-ID: %d", request->sequence_id);
      return;
    }

    // Lade und ticke den Tree
    try {
      auto tree = factory.createTreeFromFile(xml_file);

      //Startpostion (0, -45, 0, 135, 0, 90, 45)
      std::vector<double> start_joints = {0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854};
      tree.blackboard_stack[0]->set("start_joints", start_joints);

      //Nicken
      std::vector<double> nod_joints = {0.0, -0.7854, 0.0, -2.3562, 0.0, 1.7453, 0.7854};
      tree.blackboard_stack[0]->set("nod_joints", nod_joints);

      // Pick und Place!
      //Startposition um 90 Grad gedreht nach rechts (90, -45, 0, -135, 0, 90, 45)
      std::vector<double> rotate_left_joints = {1.5708, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854};
      tree.blackboard_stack[0]->set("rotate_left_joints", rotate_left_joints);
      
      // Vorne runter (90,20,15,-130,-10,150,90)
      std::vector<double> bend_forward_joints = {1.5708, 0.3491, 0.2618, -2.2689, -0.1745, 2.6180, 1.5708};
      tree.blackboard_stack[0]->set("bend_forward_joints", bend_forward_joints);

      // Proxemics: Näher kommen
      std::vector<double> proxemics_nearer = {0.0, -0.244, 0.175, -2.042, 0.052, 1.797, 1.012};
      tree.blackboard_stack[0]->set("proxemics_nearer", proxemics_nearer);

      //Proxemics: Übergabe
      std::vector<double> proxemics_handover = {0.000, 0.262, -0.157, -1.361, 0.157, 2.164, 0.785};
      tree.blackboard_stack[0]->set("proxemics_handover", proxemics_handover);

      //Kinesics
      //Endgelenk Übergabe nach proxemics_handover
      std::vector<double> endjoint = {0.000, 0.262, -0.157, -1.361, 0.157, 2.356, 0.785};

      tree.tickRoot();

      response->success = true;
      response->message = "Sequence " + std::to_string(request->sequence_id) + " erfolgreich ausgeführt.";
      RCLCPP_INFO(get_logger(), "Sequence %d ausgeführt.", request->sequence_id);
    } catch (const std::exception& e) {
      response->success = false;
      response->message = "Fehler beim Ausführen der Sequence: " + std::string(e.what());
      RCLCPP_ERROR(get_logger(), "Fehler: %s", e.what());
    }
  }

  rclcpp::Service<panda_control_msgs::srv::StartSequence>::SharedPtr service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BTService>());
  rclcpp::shutdown();
  return 0;
}