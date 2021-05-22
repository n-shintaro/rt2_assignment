#!/bin/bash
gnome-terminal --tab --title="ros1" -- bash -c "source /opt/ros/noetic/setup.bash;source /root/ros_ws/devel/setup.bash; cd ros_ws; roslaunch rt2_assignment1 sim_python.launch "
gnome-terminal --tab --title="bridge" -- bash -c "sleep 2;source /root/ros_ws/devel/setup.bash; source /root/ros2_ws/install/setup.bash; cd ros2_ws;ros2 run ros1_bridge dynamic_bridge"
gnome-terminal --tab --title="ros2" -- bash -c "sleep 2;source /opt/ros/foxy/setup.bash; source /root/ros2_ws/install/local_setup.bash; cd ros2_ws;ros2 launch rt2_assignment1 sim_comp_launch.py"