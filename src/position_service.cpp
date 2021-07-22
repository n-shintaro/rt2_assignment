/**
 * \file position_service.cpp
 * \brief This files creates a service server to create the random position
 * \author Shintaro Nakaoka
 * \version 0.1
 * \date 22/07/2021
 * 
 * \details
 * 
 * Services : <BR>
 * 		/position_server
 * 
 *  Description :
 *  
 *  This node define the position server. When the service is requested,
 *  my random function generates the random position between the max and the min.
 *
*
 */

#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"


/**
*    \brief generate the random number
*
*    \param M (double): the upper bound
*    \param N (double): the lower bound
*/

double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }

/*
Generate the rando
callback function of /position_server
M: min
N: max
*/

bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}

/*
server (/position_server) which generates the random x,y,theta
*/

int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}
