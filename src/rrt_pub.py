#!/usr/bin/env python


# This program generates a simple rapidly exploring random tree (RRT) in a square region on Rviz.
#
# This program is adapted from the code written by Steve LaValle in May 2011


import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from geometry_msgs.msg import Point

import math
import random

#constants
XDIM = 10.0
YDIM = 10.0
WINSIZE = [XDIM, YDIM]
EPSILON = 7.0
NUMNODES = 5000

PI = math.pi

rospy.init_node('path_publisher')
#marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=1000)
marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1000)

rate = rospy.Rate(30)
f = 0.0
markerArray = MarkerArray()
#points_list = []


def dist(p1,p2):
    return math.sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

def step_from_to(p1,p2):
    if dist(p1,p2) < EPSILON:
        return p2
    else:
        theta = math.atan2(p2[1]-p1[1],p2[0]-p1[0])
        return p1[0] + EPSILON*math.cos(theta), p1[1] + EPSILON*math.sin(theta)



count = 0
lista_2_pontos = []
nodes = []

#nodes.append((XDIM/2.0,YDIM/2.0)) # Start in the center
nodes.append((0.0,0.0)) # Start in the corner
while not rospy.is_shutdown(): 
	
	if count < NUMNODES:

	

		points = Marker()
		points.header.frame_id = "/my_frame"
		points.header.stamp = rospy.Time.now()


		points.ns = "points_and_lines"

		points.action = points.ADD

		points.pose.orientation.w = 1.0

		points.id = 1

		#points.type = points.POINTS
		points.type = points.LINE_LIST

		# POINTS markers use x and y scale for width/height respectively
		points.scale.x = 0.01
		#points.scale.y = 1.0

		# Points are green
		points.color.g = 1.0
		points.color.a = 1.0

		rand = (random.random() - 0.5)*XDIM, (random.random() - 0.5)*YDIM
		nn = nodes[0]
		for p in nodes:
			if dist(p,rand) < dist(nn,rand):
				nn = p

		newnode = step_from_to(nn,rand)
		nodes.append(newnode)

		# nn
		p1 = Point()
		p1.x = nn[0]
		p1.y = nn[1]
		p1.z = 0.0

		lista_2_pontos.append(p1)

		# newnode
		p2 = Point()
		p2.x = newnode[0]
		p2.y = newnode[1]
		p2.z = 0.00

		lista_2_pontos.append(p2)


		points.points = lista_2_pontos

		marker_pub.publish(points)

	
	rate.sleep()
	count = count + 1

""" 
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
"""

