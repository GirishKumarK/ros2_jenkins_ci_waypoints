FROM osrf/ros:humble-desktop

# Install Gazebo and ROS2 dependencies
RUN apt-get update && apt-get install -y \
    # Gazebo Classic
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    # Robot Description
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    # Transforms
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    # Navigation/Localization
    ros-humble-robot-localization \
    && rm -rf /var/lib/apt/lists/*

# Create ROS 2 workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/

# Copy package files
COPY ./src /ros2_ws/src/

# Build workspace with colcon (symlink-install for mesh/resource file access)
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd /ros2_ws && colcon build --symlink-install"

# Source workspace on container start (for interactive shells)
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Start a bash shell when the container starts
CMD ["bash"]