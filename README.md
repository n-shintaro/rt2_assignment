# Research Track 2 - first assignment
## OverView

### Description of the content of the package

- go_to_point.py
  - receive the goal position (/go_to_point)
  - control the velocity depending on the position
  - send the velocity (/cmd_vel)
- user_interface.py
  - user interface (command line)
  - when the user put "start", the robot starts to move
  - when the user put "stop", the robot stops
- position_service.cpp
  - when service is received from state machine, it returns the random position
- state_machine.cpp
  - when service is recived from user interface, it sends the service to position_service. 
  - it sends the random position as the goal to go_to_point

#### Topic 

- /cmd_vel
  - Type: gemetry_msgs/Twist
  - Publisher Node: /go_to_point
  - Subscriber Node: /gazebo or /vrep
- /odom
  - Type: nav_msgs/Odometry
  - Publisher Node: /gazebo or /vrep
  - Subscriber Node: /go_to_point



#### Service

- /go_to_point
  - Client Node: go_to_point
  - Server Node: state_machine
  - Type : Position
- /position_server
  - Client Node: state_machine
  - Server Node: random_position_server
  - Type:  RandomPosition
- /user_interface
  - Client Node: user_interface
  - Server Node: state_machine
  - Type : Command
  
  



#### custom service

- rt2_assignment1/RandomPosition

  - request
- float32 x_max
    - float32 x_min
    - float32 y_max
    - float32 y_min
  - response
  - float32 x
    - float32 y
    - float32 theta
- rt2_assignment1/Position
- request
    - float32 x
    - float32 y
    - float32 theta
  - response
    - bool ok
- rt2_assignment1/Position
  - request
    - string command
  - response
    - bool ok



## Q2

### Dependencies

his software is built on the Robotic Operating System ([ROS]), which needs to be install first and create the workspace. 

- Ubuntu 20.04
- ROS noetic
- gazebo



### Compiling and Running

Build catkin_workspace

```
cd cakin_workspace
catkin_make
```

launch go_to_point.py and user_interface.py with gazeb

```
launch rt2_assignment1 sim_python.launch
```





## Q 3

### Dependencies

This software is built on the Robotic Operating System ([ROS]), which needs to be install first and create the workspace. 

- Ubuntu 20.04
- ROS noetic
- CoppeliaSim V 4.2.0



### Setting of CoppeliaSim

#### download

URI

http://www.coppeliarobotics.com/downloads.html



#### ROS integration

Vrep should be already6 integrated with ROS. You just need to launch the ROS master before running the V-REP (CoppeliaSim) software. 

If there is any problem in building the plugin, you will need to recompile it by yourself: you
can download it from here

```
# install xsltproc
Run apt-get install xsltproc

# install xmlschema
pip3 install xmlschema

git clone https://github.com/CoppeliaRobotics/simExtROS.git
```



In order to build the packages, navigate to the catkin_ws folder and type:

```
export COPPELIASIM_ROOT_DIR=~/path/to/coppeliaSim/folder
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Vrep should be executed when the ROS master is already running

```
roscore
```



```
cd /install folder of Coppeliasim
./coppeliasim.sh
```



Copy the devel/lib/libsimExtROS.so file to the CoppeliaSim installation folder.



### Compiling and Running

Build catkin_workspace

```
cd cakin_workspace
catkin_make
```

launch all the nodes without gazebo simulator

```
roroslaunch rt2_assignment1 sim_vrep.launch
```



start CoppeliaSim

```
cd /install folder of Coppeliasim
./coppeliasim.sh
```



file->open scene->go to the poinner_ctrl folder of rt2_assignment package-> pioneerROS_rt2.ttt

and run the simulator







## 　Conclusion and Further investigation

We describe system’s limitations and possible improvements.

Move_base package provides us a lot of topic (reference: http://wiki.ros.org/move_base). For example, "move_base/status"  provides status information on the goals that are sent to the move_base action. If status==3, it means that the goal was achieved successfully. In my case, when I judge whether the robot reach the goal, I calculate the distance between the target position and the current position. But if I use this topic, I can judge more easily and correctly. And In my case, when I judge whether the robot reaches the goal, we don't consider the angle of orientation. I can improve like below. After distance between the target position and the current position is lower than the threshold, we can set the threshold for the angle of orientation and make the robot rotate on the spot. 

My code has a mistake that when state is changed from 1 (or 2) to 3 (follow the wall) , move_base, state3 node publishes cmd_vel. But move_base still publishes cmd_vel in order to go to the target position which was set before. In order to remove the previous target position, I found the websites which say that the topic "/move_base/cancel" and actionlib can cancel the goal.

SLAM algorithm create the mapping while moving and estimating the current position. Therefore, at the beginning, there are possibility the robot plan the path which cannot be passed through.
So If we already know the map, it is better to use amcl which estimate the current position by pattern matching with pre-existing maps and laser data. In this case, the robot already know the environment so it can solve this issue.  We already created the map data (saved in the "map" folder) so I want to use amcl, too.

The robot plan the path by Dijsksta but this algorithm explore all of the possible grid node. So it takes a long time to calculate when there are a lot of nodes. There is the other algorithm called A* which use heuristics so this calculate faster than Dijskstra.

In this environment, there are not dynamic obstacle like humans but our algorithm is not suitable for moving in the dynamic environment. Dynamic obstacle is moving unlike static obstacle so they cannot be written on the map. So the robot detect the position of dynamic obstacle and predict the motion of dynamic obstacle in real time. Further improvement and studies are needed in order for the robot to move in the dynamic environment.