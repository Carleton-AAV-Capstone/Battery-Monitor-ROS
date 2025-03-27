source "/opt/ros/humble/setup.bash" --

#Build 
cd /uros_ws
colcon build

# Run
. /install/local_setup.bash
ros2 run battery_monitor battery_monitor

wait
