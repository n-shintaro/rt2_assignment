import rospy
import time
from rt2_assignment1.srv import Command

def main():
    rospy.init_node('user_interface')
    rospy.wait_for_service('/user_interface')
    ui_client = rospy.ServiceProxy('/user_interface', Command)
    time.sleep(5)
    rate = rospy.Rate(20)
    x = int(input("\nPress 1 to start the robot "))
    while not rospy.is_shutdown():
        if (x == 1):
            print("Robot starting")
            ui_client("start")
            x = int(input("\nPress 0 to stop the robot "))
        else:
            print("Robot stopping")
            x = int(input("\nPress 1 to start the robot "))
            ui_client("stop")
#-#-#-#-#
            
if __name__ == '__main__':
    main()