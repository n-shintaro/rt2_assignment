#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/Position.h"
#include "rt2_assignment1/RandomPosition.h"
#include <rt2_assignment1/MotionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

bool start = false;

bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start"){
    	start = true;
    }
    else {
    	start = false;
    }
    return true;
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   ros::ServiceClient client_p = n.serviceClient<rt2_assignment1::Position>("/go_to_point");
   actionlib::SimpleActionClient<rt2_assignment1::MotionAction> ac("/reaching_goal", true);

   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
   rt2_assignment1::Position p;

   while(ros::ok()){
   	ros::spinOnce();
   	if (start){
        client_rp.call(rp);
        p.request.x = rp.response.x;
        p.request.y = rp.response.y;
        p.request.theta = rp.response.theta;
        
        std::cout << "\nGoing to the position: x= " << p.request.x << " y= " <<p.request.y << " theta = " <<p.request.theta << std::endl;
        rt2_assignment1::MotionGoal goal;

        // client_p.call(p);
        std::cout << "Position reached" << std::endl;
        goal.actual_target.x = p.request.x;
        goal.actual_target.y = p.request.y;
        goal.actual_target.theta = p.request.theta;
        ROS_INFO("Sending goal");
        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Hooray, target reached!");
        else
            ROS_INFO("The base failed to reach the target for some reason");
   	}
   }
   return 0;
}
