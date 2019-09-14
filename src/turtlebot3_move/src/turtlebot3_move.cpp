#include "turtlebot3_move/turtlebot3_move.h"



turtlebot3_move::turtlebot3_move() {
    // initialise publisher
    string_test = nh.advertise<std_msgs::String>("test_topic", 10);
    ROS_INFO("Publisher initialised with name: %s", "test_topic");

    // initialise subscriber
    current_odom = nh.subscribe("ODOM_SUBSCRIBER_NAME", 10, &turtlebot3_move::odom_callback, this);
    ROS_INFO("Subscriber initialised with name: %s", ODOM_SUBSCRIBER_NAME);

    pose_target_msg = nh.subscribe("POSE_TARGET_SUBSCRIBER_NAME", 10, &turtlebot3_move::pose_target_callback,this);
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
    m.getRPY(r,p,yaw);
}

// get the desired movement as a pose message
// contains a position point and orientation quaternion
void turtlebot3_move::pose_target_callback (const geometry_msgs::Pose::ConstPtr& pose_target_msg) {
	// store the pose target locally
	pose_target = * pose_target_msg;

	double r, p;

	tf::Quaternion q (pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w);

	tf::Matrix3x3 m (q);

	m.getRPY(r,p,yaw_target);

	pose_target.x
}



bool turtlebot3_move::reached_pose() {
    // get the cartesian distance to the target pose
    unsigned int dist = sqrt(pow(pose_error[0],2) + pow(pose_error[1],2));

    // if we are close enough then reached pose is true
    return (dist < POSE_ERROR_MARGIN) ? true : false;
}


void turtlebot3_move::linear_move_x() {
	float e_x = pose[0] + 	;
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
/*

void turtlebot3_move::velocity_control() {
    // get current time
    time_sec = ros::Time::now().toSec();
    
    // calculate position error
    // target position - current position
    pose_error[0] = pose_target[0] - pose[0];
    pose_error[1] = pose_target[1] - pose[1];

    if(!reached_pose()){
        // we have not reached the X,Y location yet
        // calculate yaw error
        // drive towards point
    }
    else {
        // we have reached the X,Y
        // calculate yaw error to desired orientation
        // rotate
    }

    yaw_error = yaw_target - yaw;
}


*/