source "/opt/ros/humble/setup.bash" --

#Build 
cd ~/ros2_ws
colcon build
# Run
source "install/local_setup.bash" --
ros2 run battery_monitor battery_monitor
