#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/Position.h"
#include "rt2_assignment1/RandomPosition.h"
#include <rt2_assignment1/MotionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

bool start = false; // start becomes true when the use command start

int mode=4;
// mode=1: start
// mode=2: reach the goal
// mode=3 :interrupt goal
// mode=4 : not to change

/*
    when the user request start, mode=1.
    otherwise mode=3
*/
bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    ROS_INFO("user_interface");
    if (req.command == "start"){
    	mode = 1;
      ROS_INFO("start");
    }
    else {
    	mode=3;
      ROS_INFO("stop");
    }
    return true;
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   // action client
   actionlib::SimpleActionClient<rt2_assignment1::MotionAction> ac("/go_to_point", true);

  // max and min of random position
   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
    while(!ac.waitForServer(ros::Duration(5.0))){
     ROS_INFO("Please wait for action server to come up");
   }

   while(ros::ok()){
   	ros::spinOnce();

   	if (mode==1){ // when user command start, send the random position as the goal to the robot
        client_rp.call(rp); // get random position
        //send the goal to the go_to_point
        rt2_assignment1::MotionGoal goal;

        goal.actual_target.x = rp.response.x;
        goal.actual_target.y = rp.response.y;
        goal.actual_target.theta = rp.response.theta;
        std::cout << "\nGoing to the position: x= " << goal.actual_target.x << " y= " <<goal.actual_target.y << " theta = " <<goal.actual_target.theta << std::endl;
        ROS_INFO("Sending goal");
        ac.sendGoal(goal); //send the goal to the robot
        mode=4;

   	}

    /* action ended  when
         - the robot reach the goal
         - the action is canceled
    */
    else if(mode==2){
        actionlib::SimpleClientGoalState goal_state = ac.getState();
        if(goal_state == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("Hooray, target reached!");
            mode=1; //go to the next target
        }
        else if(goal_state == actionlib::SimpleClientGoalState::PREEMPTED){
          ROS_DEBUG("Goal is canceled");
          mode = 4;
        }
        else{
          ROS_DEBUG("can't reach the goal");
          mode = 4;
        }
    }
    /*
    when the user request stop, action is canceled.
    */
    else if(mode==3){
        ac.cancelGoal(); // goal is canceled
        ROS_INFO("Goal is canceled!");
        mode=4;
    }
   }
   return 0;
}
