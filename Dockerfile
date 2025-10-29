# Use the official ROS2 Humble base image
FROM ros:humble
ENV ROBOT_WS=/robot_ws
ENV ROS_DISTRO=humble

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    terminator \
    libzmq3-dev \
    rapidjson-dev \
    git \
    libgtest-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    clang-format \
    clang-tidy \
    sudo \
    vim \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install websockets

# Build the BehaviorTree.CPP


#### Visualisation with foxglove  ####
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*


WORKDIR ${ROBOT_WS}
COPY robot_ws/src /robot_ws/src

# Install dependencies
#RUN apt update && rosdep init && rosdep update && rosdep install -y -r -i --rosdistro ${ROS_DISTRO} --from-paths /robot_ws/src


#Install ROS2 franka library for real live franka
RUN apt-get update && apt-get install -y \
    ros-dev-tools \
    ros-${ROS_DISTRO}-libfranka \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-backward-ros \
    ros-${ROS_DISTRO}-controller-interface \
    ros-humble-ros-ign-bridge \
    ros-humble-moveit \
    ros-humble-moveit-servo \
    ros-humble-ign-ros2-control \
    ros-humble-ros-ign-gazebo \
    ros-humble-ros-gz \
    ros-humble-sdformat-urdf \
    ros-humble-joint-state-publisher-gui \
    ros-humble-ros2controlcli \
    ros-humble-controller-interface \
    ros-humble-hardware-interface-testing \
    ros-humble-ament-cmake-clang-format \
    ros-humble-ament-cmake-clang-tidy \
    ros-humble-controller-manager \
    ros-humble-ros2-control-test-assets \
    libignition-gazebo6-dev \
    libignition-plugin-dev \
    ros-humble-hardware-interface \
    ros-humble-control-msgs \
    ros-humble-backward-ros \
    ros-humble-generate-parameter-library \
    ros-humble-realtime-tools \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-broadcaster \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-kinematics \
    ros-humble-moveit-planners-ompl \
    ros-humble-moveit-ros-visualization \
    ros-humble-joint-trajectory-controller \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-rviz2 \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

RUN cd ${ROBOT_WS}/src && \
    git clone https://github.com/frankarobotics/franka_ros2.git && \
    vcs import . < franka_ros2/franka.repos --recursive --skip-existing && \
    rosdep update && \
    rosdep install --from-paths . --ignore-src --rosdistro humble -y --skip-keys "ament_cmake_clang_format ament_cmake_clang_tidy" && \
    rm -rf /var/lib/apt/lists/*


#RUN . /opt/ros/humble/setup.sh && colcon build



###########################---------------Clean ROS2----------------###########################


RUN cd $ROBOT_WS && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    sudo apt-get update && \
    rosdep update && rosdep install --from-path . --ignore-src -r -y && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/* && \
    colcon build --symlink-install

RUN sudo apt update && \
    sudo apt install -y ros-humble-image-tools && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/* 

RUN echo '. /opt/ros/$ROS_DISTRO/setup.sh' >> ~/.bashrc && \
    echo '. $ROBOT_WS/install/setup.bash' >> ~/.bashrc


# Source the workspace
RUN echo "source /robot_ws/install/setup.bash" >> /root/.bashrc


###########################---------------To avoid permission issue with copying ----------------###########################
# change UID and GID depending on system ( example:superMITI is 1007)
ARG UNAME=user
ARG UID=1000 
ARG GID=1000
RUN groupadd -g $GID -o $UNAME
RUN useradd -m -u $UID -g $GID -o -s /bin/bash $UNAME
RUN usermod -aG sudo $UNAME
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN chown -R $UNAME $ROBOT_WS
USER $UNAME


# Set the entrypoint
ENTRYPOINT ["/bin/bash", "-c", "source /robot_ws/install/setup.bash && bash"]

# Default command
CMD ["bash"]
