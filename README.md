# Research Track 2 - assignment 1

## OverView

In the branch ros2, the cpp nodes (state_machine and random_position_server) are written for ROS2 as components. By using the ros1_bridge, they can communicate with ROS nodes and simulation in Gazebo.



## Requirements

- Ubuntu 20.04
- ROS noetic
- ROS2 foxy
- install ros1_bridge (https://github.com/ros2/ros1_bridge)
- install gnome-terminal

## Compile

### Step1 : Build the ROS packages

```
source /root/ros_ws/devel/setup.bash
cd ros_ws
catkin_make
```

### Step2: Build the ROS2 packages without ros1_bridge

```
source /root/ros2_ws/install/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-skip ros1_bridge
```

### Step3 : Build the ros1_bridge

```
source /root/ros_ws/devel/setup.bash
source /root/ros2_ws/install/setup.bash
cd ros2_ws
colcon build --packages-select ros1_bridge --cmake-force-configure
```



## Run

### Step1 : run the ros scripts (in the shell of ROS)

```
roslaunch rt2_assignment1 sim_python.launch 
```

### Step2: run ros1_bridge (in the shell of ROS and ROS2)

```
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

### Step3 : run ros2  (in the shell of ROS2)

```
ros2 launch rt2_assignment1 sim_comp_launch.py 
```

