#!/usr/bin/env python

# rosrun path_markers path_publisher
# rosrun rviz rviz 

import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from geometry_msgs.msg import Point

import math
from math import sqrt,cos,sin,atan2
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
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

def step_from_to(p1,p2):
    if dist(p1,p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
        return p1[0] + EPSILON*cos(theta), p1[1] + EPSILON*sin(theta)



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


	# x = i
	# y = math.sin(2*PI*i/10)
	# ponto = [x,y]
	# pontos.append(ponto)

	# x1 = x
	# y1 = y
	# list2 = []
	# for (x,y) in pontos:
	# 	p = Point() 
	# 	p.x = x
	# 	p.y = y
	# 	p.z = 0
	# 	list2.append(p)


		points.points = lista_2_pontos

		marker_pub.publish(points)

		#
		#markerArray.markers.append(points)

		

		#marker_pub.publish(markerArray)
	
	rate.sleep()
	count = count + 1


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

"""
def print_points(self, points, frame):
	def __init__(self):
        self.pub = rospy.Publisher(frame, Marker, queue_size = 100)
        rospy.sleep(1)

    rate = rospy.Rate(5)
    triplePoints = []
     #transform from x,y points to x,y,z points
    for (x,y) in points:
        p = Point() 
        p.x = x
        p.y = y
        p.z = 0
        triplePoints.append(p)

    iterations = 0
    while not rospy.is_shutdown() and iterations <= 10:
        pub = rospy.Publisher(frame, Marker, queue_size = 100)
        marker = Marker()
        marker.header.frame_id = "/map"

        marker.type = marker.POINTS
        marker.action = marker.ADD
        marker.pose.orientation.w = 1

        marker.points = triplePoints;
        t = rospy.Duration()
        marker.lifetime = t
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4
        marker.color.a = 1.0
        marker.color.r = 1.0

        #pub.publish(marker)
        self.pub.publish(marker)
        
        iterations += 1
        rate.sleep()


def main():



"""
############################################################################
# # rrt.py
# This program generates a simple rapidly
# exploring random tree (RRT) in a rectangular region.
#
# Written by Steve LaValle
# May 2011
############################################################################
""" 
import sys, random, math, pygame
from pygame.locals import *
from math import sqrt,cos,sin,atan2

#constants
XDIM = 640
YDIM = 480
WINSIZE = [XDIM, YDIM]
EPSILON = 7.0
NUMNODES = 5000

def dist(p1,p2):
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

def step_from_to(p1,p2):
    if dist(p1,p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
        return p1[0] + EPSILON*cos(theta), p1[1] + EPSILON*sin(theta)

def main():
    #initialize and prepare screen
    pygame.init()
    screen = pygame.display.set_mode(WINSIZE)
    pygame.display.set_caption('RRT      S. LaValle    May 2011')
    white = 255, 240, 200
    black = 20, 20, 40
    screen.fill(black)

    nodes = []

    nodes.append((XDIM/2.0,YDIM/2.0)) # Start in the center
#    nodes.append((0.0,0.0)) # Start in the corner

    for i in range(NUMNODES):
	rand = random.random()*640.0, random.random()*480.0
	nn = nodes[0]
        for p in nodes:
	   if dist(p,rand) < dist(nn,rand):
	      nn = p
	newnode = step_from_to(nn,rand)
 	nodes.append(newnode)
	pygame.draw.line(screen,white,nn,newnode)
        pygame.display.update()
        #print i, "    ", nodes

        for e in pygame.event.get():
	   if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
	      sys.exit("Leaving because you requested it.")
	

# if python says run, then we should run
if __name__ == '__main__':
    main()
"""
############################################################################
############################################################################
############################################################################