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
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install websockets

# Build the BehaviorTree.CPP
# Install BehaviorTree.CPP
RUN git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git && \
    cd BehaviorTree.CPP && \
    git checkout v3.8 && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && \
    make -j$(nproc) && \
    make install && \
    ldconfig

# Set environment variables
ENV CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# Create and build the ROS2 workspace
WORKDIR ${ROBOT_WS}
COPY robot_ws/src /robot_ws/src
# Install dependencies
RUN apt update && rosdep init && rosdep update && rosdep install -y -r -i --rosdistro ${ROS_DISTRO} --from-paths /robot_ws/src
RUN . /opt/ros/humble/setup.sh && colcon build

# Source the workspace
RUN echo "source /robot_ws/install/setup.bash" >> /root/.bashrc

# Set the entrypoint
ENTRYPOINT ["/bin/bash", "-c", "source /robot_ws/install/setup.bash && bash"]

# Default command
CMD ["bash"]
