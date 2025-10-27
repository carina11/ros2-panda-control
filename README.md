# ROS2-PANDA-CONTROL

## Installation
1. Clone this repo  
`git clone git@github.com:carina11/ros2-panda-control.git`  
2. Build the docker image  
`docker compose build`  
If docker is not installed on your system, please refer to this page and install  
(https://docs.docker.com/engine/install/ubuntu/)  

## How to launch the simulation environment for the panda robot
1. Start the docker image   
`docker compose up -d`
2. Give a permission to start software that needs to access your graphics
(magic)  
`xhost +`
3. Open terminator (or bash) inide docker  
`docker exec -it panda-control terminator`  
4. Start gazebo and rviz for panda robot  
`ros2 launch panda ign.launch.py`  
 

## How to launch / use the service to control the panda robot
- Launch a service to control the rob
'ros2 launch panda_control panda_control.launch.py otic arm   
` 
- Call the service 
```
ros2 service call /move_to_pose panda_control_msgs/srv/MoveToPose "target_pose:
  position:
    x: 0.5
    y: 0.0
    z: 0.5
  orientation:
    x: 1.0
    y: 0.0
    z: 0.0
    w: 0.0"
```


```
## How to launch the Behavior Tree 
- Start simulation: `ros2 launch panda ign.launch.py`
- Start BT: `ros2 launch panda_control bt_launch.py`
- Rufe die gewünsche Sequence auf: `ros2 service call /start_sequence panda_control_msgs/srv/StartSequence "sequence_id: 1`
--> ersetze die 1 durch die gewünscht ID 