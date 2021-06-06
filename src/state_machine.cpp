#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/Position.h"
#include "rt2_assignment1/RandomPosition.h"
#include <rt2_assignment1/MotionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <std_msgs/Float32.h>

bool start = false; // start becomes true when the use command start

int mode=4;
int goal_number=0;
int cancel_number=0;
double start_time=0;
double reached_time=0;

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

void doneCllbck(const actionlib::SimpleClientGoalState& goal_state,
                const rt2_assignment1::MotionResultConstPtr& result){
  mode = 2; /* Goal reached state */
}

void activeCllbck(){return;}

void feedbackCllbck(const rt2_assignment1::MotionFeedbackConstPtr& feedback){
  //ROS_INFO("FEEDBACK: %s", feedback->status.c_str());
  return;
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   ros::Publisher reached_time_pub = n.advertise<std_msgs::Float32>("/reached_time", 10);
   // action client
   actionlib::SimpleActionClient<rt2_assignment1::MotionAction> ac("/go_to_point", true);

   rt2_assignment1::RandomPosition rp;
   // max and min of random position
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
    while(!ac.waitForServer(ros::Duration(5.0))){ 
     ROS_INFO("Waiting for the go_to_point action server to come up");
   }

   while(ros::ok()){
   	ros::spinOnce();
    // std::cout << "\n mode= " << mode<<std::endl;
   	if (mode==1){ // when user command start, send the random position as the goal to the robot
        client_rp.call(rp); // get random position
        //send the goal to the go_to_point
        rt2_assignment1::MotionGoal goal;

        goal.actual_target.x = rp.response.x;
        goal.actual_target.y = rp.response.y;
        goal.actual_target.theta = rp.response.theta;
        std::cout << "\nGoing to the position: x= " << goal.actual_target.x << " y= " <<goal.actual_target.y << " theta = " <<goal.actual_target.theta << std::endl;
        ROS_INFO("Sending goal");
        ac.sendGoal(goal, &doneCllbck, &activeCllbck, &feedbackCllbck);
        start_time=ros::Time::now().toSec();

        mode=4;
        // else
        //     ROS_INFO("The base failed to reach the target for some reason");
   	}
    /*
    when the user request stop, action is canceled.
    */
    else if(mode==3){
        std::cout << "\n mode= " << mode<<std::endl;
        ac.cancelGoal();
        cancel_number++;
        ROS_INFO("Goal is canceled!");
        std::cout << "\n cancel_number= " << cancel_number<<std::endl;
        n.setParam("cancel_param",cancel_number);
        mode=4;
    }
    /* action ended  when
         - the robot reach the goal
         - the action is preempted
    */
    else if(mode==2){
        actionlib::SimpleClientGoalState goal_state = ac.getState();
        if(goal_state == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("Hooray, target reached!");
            reached_time=ros::Time::now().toSec();

            std_msgs::Float32 end_time;
            end_time.data=reached_time-start_time;
            reached_time_pub.publish(end_time);
            goal_number++;
            std::cout << "\n goal_number= " << goal_number<<std::endl;
            n.setParam("goal_param",goal_number);
            mode=1; //go to the next target
        }
        else if(goal_state == actionlib::SimpleClientGoalState::PREEMPTED){
          ROS_DEBUG("Goal canceled");
          mode = 4;
        }
        else{
          ROS_DEBUG("Fail to reach the goal");
          mode = 4;
        }

    }
   }
   return 0;
}
