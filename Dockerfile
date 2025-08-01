# Use the official ROS2 Humble base image
FROM ros:humble-ros-core-jammy
ENV ROBOT_WS=/robot_ws

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
    && rm -rf /var/lib/apt/lists/*


# Build the BehaviorTree.CPP

WORKDIR ${ROBOT_WS}
COPY robot_ws/src /robot_ws/src
# Install dependencies
RUN apt update && rosdep install -y -r -i --rosdistro ${ROS_DISTRO} --from-paths /robot_ws/src
RUN . /opt/ros/humble/setup.sh && colcon build

# Source the workspace
RUN echo "source /robot_ws/install/setup.bash" >> /root/.bashrc

# Set the entrypoint
ENTRYPOINT ["/bin/bash", "-c", "source /robot_ws/install/setup.bash && bash"]

# Default command
CMD ["bash"]
