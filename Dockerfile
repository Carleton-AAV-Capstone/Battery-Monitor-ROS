FROM ros:humble-ros-base

# install ros package
RUN apt-get update && apt-get install -y \
      ros-${ROS_DISTRO}-demo-nodes-cpp \
      ros-${ROS_DISTRO}-demo-nodes-py && \
    rm -rf /var/lib/apt/lists/*


# Move the files to the appropriate place
COPY battery_monitor ~/ros2_ws/src
COPY ros_entrypoint.sh /

ENTRYPOINT ["/bin/sh", "/ros_entrypoint.sh"]
CMD ["--help"]
