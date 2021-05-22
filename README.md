# Research Track 2 - first assignment
## OverView

- The Branch contains the same package as the branch main in ROS, but with the go_to_point node modelled as a ROS action server, instead of a “simple” server.

- Given that, the robot FSM node should now implement mechanisms for possibly cancelling the goal,
  when the related user command is received



## Q1

### Dependencies

his software is built on the Robotic Operating System ([ROS]), which needs to be install first and create the workspace. 

- Ubuntu 20.04
- ROS noetic
- gazebo



### Compiling and Running

Build catkin_workspace

```
cd ros_ws
catkin_make
```

launch go_to_point.py and user_interface.py with gazebo

```
launch rt2_assignment1 sim.launch
```





