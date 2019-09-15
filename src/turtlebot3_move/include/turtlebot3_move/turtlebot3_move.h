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
#define POSE_TARGET_SUBSCRIBER_NAME		"target_pose"

// proportional and derivative gains for the velocity control
#define KP_LINEAR						0.005
#define KP_ANGULAR						0.005

#define KI_LINEAR						0.005
#define KI_ANGULAR						0.005

#define KD_LINEAR						0.005
#define KD_ANGULAR						0.005

#define U_MAX							0.26


//
#define POSE_ERROR_MARGIN				0.1

class turtlebot3_move {
	public:
		nav_msgs::Odometry odom;
		geometry_msgs::Pose pose_target_msg;

		// current position and yaw of the turtlebot
		// subscribe to the odom topic to get these 
		double pose[3];
		double pose_old[3];

		// pose and yaw targets
		// get from user input
		double pose_target[3];

		double abs_pose_target[3];

		// get the error between the current pose and the target pose.
		double pose_error[3];
		double pose_error_old[3];

		unsigned long time_sec;
		unsigned long time_sec_old;


		// class constructor
		turtlebot3_move();

        // class deconstructor
        ~turtlebot3_move ();

        // creates and publishes message
		void create_msg ();
 
		// odometry subscriber callback function
        void odom_callback (const nav_msgs::Odometry::ConstPtr& current_odom_msg);

        // pose subscriber callback function
        void pose_target_callback(const geometry_msgs::Pose::ConstPtr& pose_target_msg_);

        void velocity_control ();

        // check if we have reached the desired pose
        // only checks if the X,Y position is correct, not the yaw
        bool reached_pose ();

        // move the bot forward a given distance
        void linear_move_x ();

	private:
		ros::NodeHandle nh;
		ros::Publisher cmd_velocity;
        ros::Subscriber current_odom;
        ros::Subscriber pose_target_sub;

        // controller terms
		float ui, up, ud, ui_old, u;
};


#endif //TURTLEBOT3_MOVE_H
