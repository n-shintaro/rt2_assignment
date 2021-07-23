"""
.. module:: user_interface
:platform: Unix
:synopsis: Python module for the user Interface
.. moduleauthor:: Shintaro Nakaoka shintaro0311@keio.jp
This file contain the description of the movement of robot and
drive the robot towards the random position in space (x,y) and with a certain angle (theta)
Publishers to:<BR>
 	\cmd_vel

Subscribers to:<BR>
    \odom

Action Server:<BR>
 	\go_to_point
"""

#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point, Pose,Pose2D
from nav_msgs.msg import Odometry
from tf import transformations
from rt2_assignment1.srv import Position
import math
import actionlib
import actionlib.msg
import rt2_assignment1.msg

# robot state variables
position_ = Point()
"""Point: actual robot position
"""
yaw_ = 0
position_ = 0
pose_ = Pose()
state_ = 0
pub_ = None

# parameters for control
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0 
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

# publisher
pub = None

# action_server
act_s = None

def clbk_odom(msg):
    """
    Callback function of odometry
    and receive the position and orientation from the robot

    Args:
      msg (Odometry): the new odometry message
    """
    global position_
    global pose_
    global yaw_

    # position
    position_ = msg.pose.pose.position
    pose_ = msg.pose.pose

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def change_state(state):
    """
    update the state

    Args:
      state(int): new state
    
    Returns:
        normalized angle
    """
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)


def normalize_angle(angle):
    """
    normalize the angle between [-pi, pi]

    Args:
      angle(float): the current angle
    
    Returns:
        normalized angle
    """
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def fix_yaw(des_pos):
    """
    orient the robot in the current position

    Args:
    des_pos(float): the desired position
    """
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(1)


def go_straight_ahead(des_pos):
    """
    move the robot toward the goal

    Args:
    des_pos(float): the desired position
    """
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub_.publish(twist_msg)
    else: # state change conditions
        #print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(0)

def fix_final_yaw(des_yaw):
    """
    orient the robot toward the final yaw

    Args:
    des_yaw(float): the desired yaw
    """
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(3)
        
def done():
    """
    stop the robot and set the robot linear and angluar vel to 0

    """
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)
    
def go_to_point(goal):
    """
    state machine: depending on the state, change the behaviour of the robot until goal is preempted.
    
    Args:
    goal(float): the desired position
    """
    global act_s
    desired_position = Point()
    desired_position.x = goal.actual_target.x
    desired_position.y =  goal.actual_target.y
    des_yaw = goal.actual_target.theta
    change_state(0)
    rate = rospy.Rate(20)
    # create messages that are used to publish feedback/result
    feedback = rt2_assignment1.msg.MotionFeedback()
    result = rt2_assignment1.msg.MotionResult()
    # when the action ended when
    # - the robot reaches the goal
    # - goal is canceled
    finished_flag=False
    success=True
    while not rospy.is_shutdown() and not finished_flag:
        # an action client to request that the current goal execution be cancelled
        if act_s.is_preempt_requested():
            rospy.loginfo('Goal was preempted')
            # signals that the action has been preempted by user request
            act_s.set_preempted()
            print('state='+str(state_))
            success = False
            done()
            change_state(-1)
            finished_flag=True

        elif state_ == 0:
            feedback.stat = "Fixing the yaw"
            feedback.actual_pose = pose_
            fix_yaw(desired_position)
        elif state_ == 1:
            feedback.stat = "Go straight ahead"
            feedback.actual_pose = pose_
            go_straight_ahead(desired_position)
        elif state_ == 2:
            feedback.stat = "fix the final state!"
            feedback.actual_pose = pose_
            fix_final_yaw(des_yaw)
        elif state_ == 3:
            feedback.stat = "Target reached!"
            feedback.actual_pose = pose_
            done()
            finished_flag=True
        # feedback is published
        act_s.publish_feedback(feedback)
        rate.sleep()
    #Once the action has finished the action notifies the action
    #client that the action is complete by setting succeeded.
    if success:
        feedback.stat="the robot reaches the goal"
        result.reached=success
        act_s.set_succeeded(result)
    return True


def main():
    """
    main function: define the publisher and subscriber
    """
    global pub_, act_s
    rospy.init_node('go_to_point')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    # service = rospy.Service('/go_to_point', Position, go_to_point)

    #an action server is created.
    act_s = actionlib.SimpleActionServer(
        '/go_to_point', rt2_assignment1.msg.MotionAction, go_to_point, auto_start=False)
    act_s.start()
    rospy.spin()

if __name__ == '__main__':
    main()
