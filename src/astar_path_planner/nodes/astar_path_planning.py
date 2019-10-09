#!/usr/bin/env python
# Path planning node returns a list of poses forming a path
# Subscibes to OccupancyGrid
# Run map_server with target map to publish OccupancyGrid messages
# Publishes PoseArray with name "path"
# subscribes to amcl_pose for starting pose

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped


from tf.transformations import quaternion_from_euler
import matplotlib.pyplot as plt
import math
import numpy

#import to stop matplot warnings in terminal
import warnings
import matplotlib.cbook
warnings.filterwarnings("ignore",category=matplotlib.cbook.mplDeprecation)

# Global variables
map_origin      = Pose()
map_res         = 0         # m/nodes
ox, oy          = [], []    # obstacle maps
gx, gy          = 0, 0      # goal x,y
sx, sy          = 0, 0      # start position for planning
robot_radius    = 0.25       # meters
show_animation  = True
planning_status = False     # currently planning?

class astar_planner:

    def __init__(self, ox, oy, reso, rr):
        self.reso = reso
        self.rr = rr
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, pind):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.pind = pind

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)

    def planning(self, sx, sy, gx, gy):
        print("Planning started")

        global planning_status
        planning_status = True

        nstart = self.Node(self.calc_xyindex(sx, self.minx),
                           self.calc_xyindex(sy, self.miny), 0.0, -1)
        ngoal = self.Node(self.calc_xyindex(gx, self.minx),
                          self.calc_xyindex(gy, self.miny), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(nstart)] = nstart

        while 1:
            if len(open_set) == 0:
                print("Open set is empty...")
                break

            c_id = min(open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(ngoal, open_set[o]))
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.minx),
                         self.calc_grid_position(current.y, self.miny), "xc")
                if len(closed_set.keys()) % 100 == 0:
                    plt.pause(0.001)

            if current.x == ngoal.x and current.y == ngoal.y:
                print("Goal Found!")
                ngoal.pind = current.pind
                ngoal.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)


                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(ngoal, closed_set)

        return rx, ry

    def calc_final_path(self, ngoal, closedset):
        # generate final course
        rx, ry = [self.calc_grid_position(ngoal.x, self.minx)], [
            self.calc_grid_position(ngoal.y, self.miny)]
        pind = ngoal.pind
        while pind != -1:
            n = closedset[pind]
            rx.append(self.calc_grid_position(n.x, self.minx))
            ry.append(self.calc_grid_position(n.y, self.miny))
            pind = n.pind

        global planning_status
        planning_status = False        

        return rx[::-1], ry[::-1]

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.sqrt((n1.x - n2.x) ** 2 + (n1.y - n2.y) ** 2)
        return d

    def calc_grid_position(self, index, minp):

        pos = index * self.reso + minp
        return pos

    def calc_xyindex(self, position, min_pos):
        return int(round((position - min_pos) / self.reso))

    def calc_grid_index(self, node):
        return (node.y - self.miny) * self.xwidth + (node.x - self.minx)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.minx)
        py = self.calc_grid_position(node.y, self.miny)

        if px < self.minx:
            return False
        elif py < self.miny:
            return False
        elif px >= self.maxx:
            return False
        elif py >= self.maxy:
            return False

        # collision check
        if self.obmap[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.minx = int(round(min(ox)))
        self.miny = int(round(min(oy)))
        self.maxx = int(round(max(ox)))
        self.maxy = int(round(max(oy)))

        self.xwidth = int(round((self.maxx - self.minx) / self.reso))
        self.ywidth = int(round((self.maxy - self.miny) / self.reso))

        self.obmap = [[False for i in range(self.ywidth)]for i in range(self.xwidth)]
        count = 0
        #print(int(round(self.rr)))
        for x, y in zip(ox, oy):
            ix = x - self.minx
            iy = y - self.miny
            try:
                self.obmap[ix][iy] = True
                count += 1
                for r in range(int(round(self.rr))):
                    for dy in range(-r, r):
                        for dx in range(-r, r):
                            try:
                                if not self.obmap[ix+dx][iy+dy]:
                                    self.obmap[ix+dx][iy+dy] = True
                                    count += 1
                                    print("count: {:<12}\r" .format(count)),
                            except IndexError:
                                pass
            except IndexError:
                pass
        
        print("\nAdded {} nodes to the vitual obstacle map". format(count))


    def verify_target_pose(self, gx, gy):
        if (gx < self.minx) or (gx > self.maxx) or (gy < self.miny) or (gy > self.maxy):
            return False
   
        elif self.obmap[int(round(gx - self.minx))][int(round(gy - self.miny))]:
            return False

        else:
            return True

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion

'''
####################################
###########End of Class#############
####################################
'''


def print_map(msg):
	f = open('map.txt', 'a+')
	f.truncate(0)

	print("Found map!")

	for y in range(msg.info.height -1, 0, -1):
		for x in range(msg.info.width):
			i = msg.info.width*y + x
			if(x < msg.info.width - 1):
				c = to_trinary(msg.data[i])
				f.write(c)
			else:
				f.write("\r\n")

	f.close()
	rospy.signal_shutdown("quit")



def to_trinary(p):
    # change the occupancy grid values to something more convenient
	if (p == 100):
		return str(1)
	elif (p == 0):
		return str(0)
	else:
		return str(2)



def map_callback(msg):
    # create obstacle arrays from map message
    map = msg
    count = 0

    global ox, oy, map_res, map_origin

    # setting this to a larger value will reduce the number of nodes
    map_res = map.info.resolution
    map_origin = map.info.origin

    for y in range(map.info.height):
        for x in range(map.info.width):
            i = y * map.info.width + x
            if (to_trinary(map.data[i]) == '1'):
                ox.append(x)
                oy.append(y)
                count += 1
    
    print("Map found!\nHeight: {}\nWidth: {}\nResolution: {}" .format(map.info.height, map.info.width, map.info.resolution))
    print("Found {} obstacle points\n" .format(count))
    pose_amcl_sub = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)



def list_to_pose(x_list, y_list):

    # create pose array from x and y lists
    pose_array_msg = PoseArray()
    old_xypose = Pose()

    # iterate through path lists
    for i in range(len(x_list)):
        xypose = Pose()
        xypose.position.x, xypose.position.y = node_to_pose(x_list[i], y_list[i])

        if (i + 1 < len(x_list)):
            # heading is the direction to the next pose
            dx = x_list[i+1] - x_list[i]
            dy = y_list[i+1] - y_list[i]

            # atan2 takes into account quadrants to return an angle beyond pi/2
            theta = (numpy.pi / 2) -math.atan2(dx, dy) 
            xypose.orientation.x, xypose.orientation.y, xypose.orientation.z, xypose.orientation.w= quaternion_from_euler(0, 0, theta)

        # only add this pose to the message if the angle changes
        # removes redundant pose destinations
        if(xypose.orientation != old_xypose.orientation):
            pose_array_msg.poses.append(xypose)
            old_xypose = xypose
    
    # message needs a frame of reference
    pose_array_msg.header.frame_id = "map"
    pose_pub.publish(pose_array_msg)

    # confirmation
    print("Published {} path poses\n" .format(len(pose_array_msg.poses)))



def node_to_pose(x, y):
    x_pose = map_origin.position.x + map_res * x
    y_pose = map_origin.position.y + map_res * y

    return x_pose, y_pose



def pose_to_node(x_pose, y_pose):
    x = (x_pose - map_origin.position.x)/map_res
    y = (y_pose - map_origin.position.y)/map_res

    return x, y


def pose_target_callback(msg):

    # target pose callback
    # recieves msg from pose publisher node
    # if we are not currently planning, then set the new goal
    global planning_status
    global gx, gy

    try:
        plt.close()
    except:
        pass

    if not planning_status:
        gx, gy = pose_to_node(msg.pose.position.x, msg.pose.position.y)
        print("\nGoal found!")
        print("Goal Pose: {}, {}" .format(msg.pose.position.x, msg.pose.position.y))
        print("Goal Node: {}, {}\n" .format(round(gx), round(gy)))
    else:
        print("Still planning, cannot update goal")




def amcl_pose_callback(msg):
    global sx, sy
    sx, sy = pose_to_node(msg.pose.pose.position.x, msg.pose.pose.position.y)



if __name__=="__main__":
    rospy.init_node('map_writer')
    map_sub = rospy.Subscriber('map', OccupancyGrid, map_callback)
    pose_target_sub = rospy.Subscriber("move_base_simple/goal", PoseStamped, pose_target_callback)
    
    pose_pub = rospy.Publisher("path", PoseArray, queue_size = 10)

    while 1:
        # if start, goal, and map have been recieved
        if(ox and gx and sx):
            
            robot_radius /= (map_res)
    
            astar = astar_planner(ox, oy, 1, robot_radius)

            # wait for a new goal to start planning again
            while 1:
                while not astar.verify_target_pose(gx,gy):
                    print("Waiting for valid target \r"),  

                if show_animation:
                    plt.plot(ox, oy, ".k")
                    plt.plot(sx, sy, "og")
                    plt.plot(gx, gy, "xb")
                    plt.grid(True)
                    plt.axis("equal") 

                rx, ry = astar.planning(sx, sy, gx, gy)
                list_to_pose(rx, ry)

               
                if show_animation:
                    plt.plot(rx, ry, "-r")
                    plt.show()

                gx, gy = 0, 0
               
