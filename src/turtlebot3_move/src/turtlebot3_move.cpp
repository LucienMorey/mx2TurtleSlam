#include "turtlebot3_move/turtlebot3_move.h"



turtlebot3_move::turtlebot3_move() {
    // initialise publisher
    string_test = nh.advertise<std_msgs::String>("test_topic", 10);
    ROS_INFO("Publisher initialised with name: %s", "test_topic");

    // initialise subscriber
    current_odom = nh.subscribe(ODOM_SUBSCRIBER_NAME, 10, &turtlebot3_move::odom_callback, this);
    ROS_INFO("Subscriber initialised with name: %s", ODOM_SUBSCRIBER_NAME);

    pose_target_sub = nh.subscribe(POSE_TARGET_SUBSCRIBER_NAME, 10, &turtlebot3_move::pose_target_callback,this);
    ROS_INFO("Subscriber initialised with name: %s", POSE_TARGET_SUBSCRIBER_NAME);

} 



// class deconstructor
turtlebot3_move::~turtlebot3_move () {}



void turtlebot3_move::create_msg () {
    // initiallise a string message
    std_msgs::String msg;

    // get the current ros time in seconds
    unsigned long t = ros::Time::now().toSec();

    // create a stringstream to build the message
    std::stringstream s;
    s << "time:" << t;


    // load the string into the message
    msg.data = s.str();

    // publish the message
    string_test.publish(msg);
}



void turtlebot3_move::odom_callback (const nav_msgs::Odometry::ConstPtr& current_odom_msg) {
    for(int i = 0; i<3;i++)
    	pose_old[i] = pose[i];

    odom = *current_odom_msg;

    // load the msg into an internal odometry variable
    pose[0] = odom.pose.pose.position.x;
    pose[1] = odom.pose.pose.position.y;

    // create these temp variables to get euler conversion
    double r, p;

    // create a Quaternion to hold the geometry_msgs/quaternion data so that we can make a matrix
    tf::Quaternion q (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w);

    // make a matrix with the current orientration quarternion
    tf::Matrix3x3 m (q);

    // convert to euler roll pitch yaw, we only need the yaw here
    m.getRPY(r,p,pose[2]);
}

// get the desired movement as a pose message
// contains a position point and orientation quaternion
void turtlebot3_move::pose_target_callback (const geometry_msgs::Pose::ConstPtr& pose_target_msg_) {
	// store the pose target locally
	pose_target_msg = * pose_target_msg_;

	pose_target[0] = pose_target_msg.position.x;
	pose_target[1] = pose_target_msg.position.y;
	ROS_INFO("pose_target[0]: %f", pose_target[0]);
	double r, p;

	tf::Quaternion q (pose_target_msg.orientation.x, pose_target_msg.orientation.y, pose_target_msg.orientation.z, pose_target_msg.orientation.w);

	tf::Matrix3x3 m (q);

	m.getRPY(r,p,pose_target[2]);

	// abs_pose_target is the target position in absolute reference frame
    // determined by adding the linear move distance to the direction we are facing
    abs_pose_target[0] = pose[0] + pose_target[0]*cos(pose[2]);
    abs_pose_target[1] = pose[1] + pose_target[0]*sin(pose[2]);
    abs_pose_target[2] = pose[2] + pose_target[2];
    ROS_INFO("abs_pose_target[0]: %f", abs_pose_target[0]);

}



bool turtlebot3_move::reached_pose() {
    // get the cartesian distance to the target pose
    unsigned int dist = sqrt(pow(pose_error[0],2) + pow(pose_error[1],2));

    // if we are close enough then reached pose is true
    return (dist < POSE_ERROR_MARGIN) ? true : false;
}


void turtlebot3_move::velocity_control() {
	// get current time
    time_sec = ros::Time::now().toSec();

    pose_error[0] = sqrt(pow((abs_pose_target[0]-pose[0]),2) + pow((abs_pose_target[1]-pose[1]),2));
    //ROS_INFO("Error: %f", pose_error[0]);
    pose_error[1] = pose_target[2]- pose[2];
  	ROS_INFO("Error: %f", pose_error[1]);
  	
    up = KP_LINEAR * pose_error[0];
  	// integral term
  	ui = ui_old  + KI_LINEAR * pose_error[0];

  	// derivative term
	ud = KD_LINEAR * ((pose_error[0] - pose_error_old[0])/(time_sec-time_sec_old));

	// total = p + i + d
	u = up + ui + ud;

	// // saturation and anti-wind up
	// if (u > u_max_)
	//     u = u_max_;
	// else if (u < u_min_)
	//       u = u_min_;
	// else
	// {
	//     // Integral value is stored for the next step only if
	//     // control value is not saturated(anti-wind up)
	//     ui_old_ = ui;
	// }
	ui_old = ui;

	pose_error_old[0] = pose_error[0];
	time_sec_old = time_sec;
	
}

int main (int argc, char **argv) {
    // initialise the node in ros
    // argc and argv are terminal arguments we dont need
    // name the node - must be unique
    ros::init(argc, argv, NODE_NAME);
    ROS_INFO("Node Initialised with name: %s", NODE_NAME);

    // change the logger level
    // this allows ROS_INFO() to output to the console
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    // initialise a turtlebot3 object
	turtlebot3_move tb3move;

    // rate at which we want to loop in Hz
    // this keeps track of how long since last rate.sleep
    // also controls publishing frequency
    ros::Rate rate(1);

    // check the node hasnt been closed and ros is alive
    while (ros::ok()) {
        tb3move.velocity_control();
        // generate a new message and publish it
        tb3move.create_msg();
        // handle ros events and messages
        ros::spinOnce();

        // sleep for a while
        rate.sleep();
    }
}



// VELOCITY CONTROLLER
// NOT CURRENTLY WORKING
// USING LINEAR_MOVE AND ANGULAR MOVE






