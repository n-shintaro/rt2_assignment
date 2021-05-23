# Research Track 2 - first assignment
## OverView

### the robot movement

- the robot moves to the random position when user demand the robot to move.
  - when the user type 1 in the shell, the robot starts to move
  - when the user type 0 in the shell, the robot try to stop (in the action branch, the robot stops immediately. Otherwise, the robot can be stopped only when it reaches a target)

### nodes

There are four main nodes

- GotoPoint node
  - drive the robot towards the random position in space (x,y) and with a certain angle (theta)
- User interface node
  - receive the user request
  - ask the state_machine to send the command to the robot
- Position server node
  - reply with random values for x, y, and theta, where x and y should be limited between some min and max values
- State machine node
  - give the possibility to start or stop the robot behaviour when the user requests



### Requirements of assignment

#### Q1) Action branch

- state machine node should now implement mechanisms for possibly canceling the goal, when user type stop (0) command in the shell



#### Q2) ROS2 branch

- cpp nodes are written for ROS as components
- by using the ros1_bridge, they can be interfaces with the ROS nodes and with simulation in Gazebo.



#### Q3) Main branch

- use Vrep in stead of Gazebo

- Vrep communicate with ROS (not ROS2)

  

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

launch go_to_point.py and user_interface.py with gazebo

```
roslaunch rt2_assignment1 sim_python.launch
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
roslaunch rt2_assignment1 sim_vrep.launch
```



start CoppeliaSim

```
cd /install folder of Coppeliasim
./coppeliasim.sh
```



file->open scene->go to the vrep folder of rt2_assignment package-> pioneerROS_rt2.ttt

and run the simulator


