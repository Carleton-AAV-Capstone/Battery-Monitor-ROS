FROM microros/base:humble

WORKDIR /uros_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
&&  . install/local_setup.sh \
&&  ros2 run micro_ros_setup create_agent_ws.sh \
&&  ros2 run micro_ros_setup build_agent.sh \
&&  rm -rf log/ build/ src/

COPY ./battery_monitor /uros_ws

ENTRYPOINT ["/bin/sh", "/ros_entrypoint.sh"]
CMD ["--help"]
