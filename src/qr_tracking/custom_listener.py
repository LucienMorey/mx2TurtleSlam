#!/usr/bin/env python
import rospy
from beginner_tutorials.msg import QR
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
import copy



victim_pose_array = PoseArray()
victim_pose_array.header.frame_id = "map"
current_pose = Pose()

def callback_QR(data):
    
    global victim_pose_array, current_pose
    pose_length = len(victim_pose_array.poses)
    print("entered callback")

    
    if (pose_length == 0 or ((abs(victim_pose_array.poses[-1].position.x-current_pose.position.x) > 0.30) or (abs(victim_pose_array.poses[-1].position.y-current_pose.position.y) > 0.30))):
        victim_pose = copy.deepcopy(current_pose)
        victim_pose_array.poses.append(victim_pose)
        print("Added element @ X: {}\tY: {}\n" .format(victim_pose.position.x, victim_pose.position.y))



def callback_POSE(data):

    # this is the current AMCL pose of the robot
    global current_pose


    # update current pose
    current_pose.position.x = data.pose.pose.position.x
    current_pose.position.y = data.pose.pose.position.y
    
    current_pose.orientation.x = data.pose.pose.orientation.x
    current_pose.orientation.y = data.pose.pose.orientation.y
    current_pose.orientation.z = data.pose.pose.orientation.z
    current_pose.orientation.w = data.pose.pose.orientation.w
    
    
if __name__ == '__main__':
    

    # start node
    rospy.init_node('QR_listener')

    # publish @ 1Hz
    rate = rospy.Rate(1)

    # initialise subscriber
    rospy.Subscriber("QR_chatter", QR, callback_QR)
    pose_sub = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, callback_POSE)

    # initialise publisher
    array_publisher = rospy.Publisher('victim_pose_array', PoseArray, queue_size=5)



    try:
        while not rospy.is_shutdown():
            # publish the pose_array
            array_publisher.publish(victim_pose_array)

            rate.sleep()

    except Exception as e:
        print(e)
''''''