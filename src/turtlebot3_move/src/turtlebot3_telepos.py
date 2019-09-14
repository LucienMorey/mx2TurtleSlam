import rospy
from geometry_msgs.msg import Pose
import sys

msg = """

CTRL-C to quit
"""

e = """
Communications Failed
"""

def getLine():
   	line = sys.stdin.getline().split()
    return line

if __name__=="__main__":

    rospy.init_node('turtlebot3_telepos')
    pub = rospy.Publisher('target_pose', Pose, queue_size=10)

    try:
        print msg
        while(1):
            line = getLine()
            
           	if (len(line) != 3)
           		print e_f
           		break

            pub.publish(twist)

    except:
        print e

    finally:
        pose = Pose()
        pose.position.x = 0; pose.position.y = 0; pose.position.z = 0
        pose.orientation.x; pose.orientation.y; pose.orientation.z; pose.orientation.w
        pub.publish(pose)