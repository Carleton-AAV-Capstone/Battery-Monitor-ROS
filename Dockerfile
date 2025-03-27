FROM ros:humble-ros-base

# install ros package
RUN apt-get update && apt-get install -y \
      ros-${ROS_DISTRO}-demo-nodes-cpp \
      ros-${ROS_DISTRO}-demo-nodes-py && \
    rm -rf /var/lib/apt/lists/*


# Move the files to the appropriate place
COPY battery_monitor ~/ros2_ws/src

# Build
RUN cd ~/ros2_ws
RUN colcon build

# Run
CMD source /opt/ros/humble/setup.bash && source install/local_setup.bash && ros2 run battery_monitor battery_monitor
