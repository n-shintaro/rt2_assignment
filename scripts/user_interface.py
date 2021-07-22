# # @package rt2_assignment1
# 	\file go_to_point.py
# 	\brief this file define the node of user interface to ask the user the command of the robot.
# 	\author Shintaro Nakaoka
# 	\date 21/07/2021

# 	\details
#
# 	\Client:<BR>
# 	    \user_interface

import rospy
import time
from rt2_assignment1.srv import Command

##
#    \brief main function: ask the user whether it starts or stops the robot.
#     if input=1, the robot starts to move. else if input=0, the robot stops.
#
def main():
    rospy.init_node('user_interface')
    ui_client = rospy.ServiceProxy('/user_interface', Command)
    time.sleep(10)
    rate = rospy.Rate(20)
    x = int(input("\nPress 1 to start the robot "))
    while not rospy.is_shutdown():
        if (x == 1):
            ui_client("start")
            x = int(input("\nPress 0 to stop the robot "))
        else:
            print("The robot stopped.")
            ui_client("stop")
            x = int(input("\nPress 1 to start the robot "))
            
if __name__ == '__main__':
    main()