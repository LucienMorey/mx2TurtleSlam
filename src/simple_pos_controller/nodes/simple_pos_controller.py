#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios
 
x = 0.0
y = 0.0 
theta = 0.0

WAFFLE_MAX_LIN_VEL = 0.22
WAFFLE_MAX_ANGLE_VEL = 2.84
 
def getKey():
    if os.name == 'nt':
        return msvcrt.getch()
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def newOdom(msg):
    global x
    global y
    global theta
 
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
 
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])



if __name__=="__main__":

    if os.name!= 'nt':
        settings = termios.tcgettattr(sys.stdin)
    
    rospy.init_node("speed_controller")
 
    sub = rospy.Subscriber("/odom", Odometry, newOdom)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
     
    speed = Twist()
     
    r = rospy.Rate(4)
     
    goal = Point()
    goal.x = 5
    goal.y = 0
     
    while not rospy.is_shutdown():
        inc_x = goal.x -x
        inc_y = goal.y -y
     
        angle_to_goal = atan2(inc_y, inc_x)
     
        if abs(angle_to_goal - theta) > 0.1:
            speed.linear.x = 0.0
            speed.angular.z = 0.3
        elif abs(inc_x >= 0.1):
            speed.linear.x = inc_x*0.5
            speed.angular.z = 0.0
        else:
            speed.linear.x = 0.0
            speed.angular.z = 0.0
     
        pub.publish(speed)
        r.sleep()
