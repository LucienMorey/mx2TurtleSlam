#ifndef TURTLEBOT3_MOVE_H_
#define TURTLEBOT3_MOVE_H_

#include <math.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <tf/LinearMath/Transform.h>

// define some names
#define ODOM_SUBSCRIBER_NAME			"odom"
#define VELOCITY_PUBLISHER_NAME			"cmd_vel"
#define NODE_NAME						"velocity_control"

// proportional and derivative gains for the velocity control
#define ANGLE_P							0.05
#define ANGLE_D							0.01
#define LINEAR_P						0.15
#define LINEAR_D						0.01

//
#define POSE_ERROR_MARGIN				0.1

class turtlebot3_move {
	public:
		nav_msgs::Odometry odom;

		// current position and yaw of the turtlebot
		// subscribe to the odom topic to get these 
		float pose[2];
		double yaw;

		// pose and yaw targets
		// get from user input
		float pose_target[2];
		float yaw_target;

		// get the error between the current pose and the target pose.
		float pose_error[2];
		float yaw_error;

		// velocity output from the controller
		// publish this as a cmd_vel message for the turtlebot
		float cmd_vel_[6];

		unsigned long time_sec;
		unsigned long time_sec_old;


		// class constructor
		turtlebot3_move();

        // class deconstructor
        ~turtlebot3_move ();

        // creates and publishes message
		void create_msg ();
 
		// odometry subscriber callback function
        void odom_callback (const nav_msgs::Odometry::ConstPtr& current_odom);

        void velocity_control ();

        // check if we have reached the desired pose
        // only checks if the X,Y position is correct, not the yaw
        bool reached_pose ();

	private:
		ros::NodeHandle nh;
		ros::Publisher string_test;
        ros::Subscriber current_odom;
};


#endif //TURTLEBOT3_MOVE_H
