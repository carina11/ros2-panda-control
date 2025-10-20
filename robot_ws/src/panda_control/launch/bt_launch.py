import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    
    robot_description_package = 'panda_description'
    urdf_file = os.path.join(get_package_share_directory("panda_description"), 'urdf', 'panda.urdf')
    srdf_file = os.path.join(get_package_share_directory("panda_moveit_config"), 'srdf', 'panda.srdf')  
    # Convert the URDF and SRDF files using xacro (if needed)
    robot_description = {'robot_description': open(urdf_file).read()}
    robot_description_semantic = {'robot_description_semantic': open(srdf_file).read()}
   
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='panda_control',
            executable='bt_main',
            name='bt_main',
            output='screen',
              parameters=[
                {'robot_description': open(urdf_file).read()},
                {'robot_description_semantic': open(srdf_file).read()},
                 ]
        )
    ])