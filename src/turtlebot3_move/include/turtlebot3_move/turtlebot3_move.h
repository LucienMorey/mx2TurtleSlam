#ifndef TURTLEBOT3_MOVE_H_
#define TURTLEBOT3_MOVE_H_

#include <math.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>

#define ODOM_SUBSCRIBER_NAME			"odom"
#define VELOCITY_PUBLISHER_NAME			"cmd_vel"
#define NODE_NAME						"velocity_control"

// proportional and derivative gains for the velocity control
#define ANGLE_P							5
#define ANGLE_D							2
#define LINEAR_P						5
#define LINEAR_D						2

class turtlebot3_move {
	public:
		float pose[3];
		float pose_target[3];
		float yaw;
		float yaw_target;

		float pose_error[3];
		float yaw_error;


		// class constructor
		turtlebot3_move();

        // class deconstructor
        ~turtlebot3_move ();

        // creates and publishes message
		void create_msg ();

		// odometry subscriber callback function
        void odom_callback (const nav_msgs::Odometry::ConstPtr& current_odom);

        void velocity_control ();

	private:
		ros::NodeHandle nh;
		ros::Publisher string_test;
        ros::Subscriber current_odom;
};


#endif //TURTLEBOT3_MOVE_H
