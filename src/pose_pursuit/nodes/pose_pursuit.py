#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Twist, PoseWithCovarianceStamped, PoseArray
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import numpy as np

class PosePursuit:
    def __init__(self):

		# initialise ROS node
		rospy.init_node('pose_pursuit')

		# publisher
		self.velocity_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

		# subscriber
		self.odom_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)
		self.pose_goal_sub = rospy.Subscriber('target_pose', Pose, self.pose_goal_callback)

		# current x, y, a position
		self.x = 0
		self.y = 0
		self.theta = 0

		# controller gains
		self.Kp_rho = 0.1
		self.Kp_alpha = 0.2
		self.Kp_beta = -0.2

		# velocity to output
		self.velocity = Twist()

		# pose array for path message
		self.pose_array = []

    def update_current_pose_goal(self):
        print("ping")
        self.x_goal = self.pose_array[0].position.x
        self.y_goal = self.pose_array[0].position.y
    	rot_q = (
            self.pose_array[0].orientation.x,
            self.pose_array[0].orientation.y,
            self.pose_array[0].orientation.z,
            self.pose_array[0].orientation.w)

    	(roll_goal, pitch_goal, self.theta_goal) = euler_from_quaternion(rot_q)

    	# delete the current target from the array after we set it
    	# next loop the next element will be at the front
    	del self.pose_array[0]

	def amcl_pose_callback(self, msg):
		self.x = msg.pose.pose.position.x
    	self.y = msg.pose.pose.position.y
    	rot_q = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)

    	(roll_current, pitch_current, self.theta_current) = euler_from_quaternion(rot_q)
    

	def pose_array_goal_callback(self, msg):
		# path PoseArray from path planner
		# poses are loaded into list
		for pose in msg.poses:
			self.pose_array.append(pose)

        
	def pose_goal_callback(self, msg):
		self.x_goal = msg.position.x
		self.y_goal = msg.position.y
    	rot_q = (
    			msg.orientation.x,
            	msg.orientation.y,
            	msg.orientation.z,
            	msg.orientation.w
            	)
    	(roll_goal, pitch_goal, self.theta_goal) = euler_from_quaternion(rot_q)

	def move_to_pose(self):
    	# print("rho is:", (rho) )
    	# print("heading error is", (theta_goal-theta_current))
    	# print("alpha is:",alpha)
    	# print("beta is", beta)
		x_diff = self.x_goal - self.x
    	y_diff = self.y_goal - self.y

    	rho = (np.sqrt(x_diff**2 + y_diff**2))

    	# Restrict alpha and beta (angle differences) to the range
    	# [-pi, pi] to prevent unstable behavior e.g. difference going
    	# from 0 rad to 2*pi rad with slight turn

    	alpha = (np.arctan2(y_diff, x_diff) - self.theta_current + np.pi) % (2 * np.pi) - np.pi

    	beta = (self.theta_goal - self.theta_current - alpha + np.pi) % (2 * np.pi) - np.pi

    	if ((rho > 0.05) or (abs(self.theta_goal - self.theta_current) > 0.0825)):       
        
        	self.velocity.linear.x = Kp_rho * rho
        	self.velocity.angular.z = Kp_alpha * alpha + Kp_beta * beta

        	if self.velocity.linear.x > 0.22:
        		self.velocity.linear.x = 0.22

        	if self.velocity.angular.z > 1.82:
        		self.velocity.angular.z = 1.82
        	elif self.velocity.angular.z < -1.82:
        		self.velocity.angular.z = -1.82

        	if alpha > np.pi / 2 or alpha < -np.pi / 2:
        		self.velocity.linear.x = -self.velocity.linear.x
    	else:
        	self.velocity.linear.x = 0.0
        	self.velocity.angular.z = 0.0
        	# if len(pose_array) > 0:
        	#     update_current_pose_goal()



if __name__ == '__main__':

    
    try:
        while not rospy.is_shutdown():
            
            pose_pursuit = PosePursuit()
            pose_pursuit.move_to_pose()
            
    except Exception as fail:
        print(fail)