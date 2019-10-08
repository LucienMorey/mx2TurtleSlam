#!/usr/bin/env python
# subscribe to occupancy grid msg
# occupancy grid published by map_server map_server
# output occupancy grid as a text file

import rospy
from nav_msgs.msg import OccupancyGrid

def print_map(msg):
	f = open('map.txt', 'a+')
	f.truncate(0)

	print("Found map!")

	for y in range(msg.info.height -1, 0, -1):
		for x in range(msg.info.width):
			i = msg.info.width*y + x
			if(x < msg.info.width - 1):
				c = to_trinary(msg.data[i])		#check type
				f.write(c)
			else:
				f.write("\r\n")
	
	f.close()
	rospy.signal_shutdown("quit")

def map_callback(msg):
	map = msg
	print("map %s"  %type(map))
	print("map.data %s" %type(map.data))
	for i in range(len(map.data)):
		a = map.data[i]
		#print(a)
		#map.data[i] = to_trinary(a)
	print_map(map)

	rospy.signal_shutdown("quit")



def to_trinary(p):
	if (p == 100):
		return str(1)
	elif (p == 0):
		return str(0)
	else:
		return str(2)

if __name__=="__main__":
    rospy.init_node('map_writer')
    map_sub = rospy.Subscriber('map', OccupancyGrid, map_callback)
    map_pub = rospy.Publisher("normalised_map", OccupancyGrid, queue_size = 10)

    rospy.spin()
