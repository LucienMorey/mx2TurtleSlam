#include "turtlebot3_move/turtlebot3_move.h"



turtlebot3_move::turtlebot3_move() {
    // initialise publisher
    string_test = nh.advertise<std_msgs::String>("test_topic", 10);
    ROS_INFO("Publisher initialised with name: %s", ODOM_SUBSCRIBER_NAME);

    // initialise subscriber
    current_odom = nh.subscribe("odom", 10, &turtlebot3_move::odom_callback, this);
    ROS_INFO("Subscriber initialised with name: %s", VELOCITY_PUBLISHER_NAME);
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



void turtlebot3_move::odom_callback (const nav_msgs::Odometry::ConstPtr& current_odom) {
    // load the msg into an internal odometry variable
    pose[0] = current_odom->pose.pose.position.x;
    pose[1] = current_odom->pose.pose.position.y;
    pose[2] = current_odom->pose.pose.position.z;

}

void turtlebot3_move::velocity_control() {
    // calculate position error
    // target position - current position
    pose_error[0] = pose_target[0] - pose[0];
    pose_error[1] = pose_target[1] - pose[1];
    pose_error[2] = pose_target[2] - pose[2];

    // calculate yaw error
    // positive yaw is to the left
    yaw_error     = yaw_target - yaw;

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

        ROS_INFO("POSX: %f", tb3move.odom.pose.pose.position.x);

        // handle ros events and messages
        ros::spinOnce();

        // sleep for a while
        rate.sleep();
    }
}
