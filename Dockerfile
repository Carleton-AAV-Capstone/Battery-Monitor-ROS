FROM ros:humble-ros-base

# install ros package
RUN apt-get update && apt-get install -y \
      ros-${ROS_DISTRO}-demo-nodes-cpp \
      ros-${ROS_DISTRO}-demo-nodes-py && \
    rm -rf /var/lib/apt/lists/*


# Move the files to the appropriate place
COPY battery_monitor ~/ros2_ws/src

# Run
#CMD cd ~/ros2_ws && colcon build && source /opt/ros/humble/setup.bash && source install/local_setup.bash && ros2 run battery_monitor battery_monitor
