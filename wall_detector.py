#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point 
from std_msgs.msg import ColorRGBA
from laser_line_extraction.msg import LineSegmentList
import numpy as np
import matplotlib.pyplot as plt 

FRAME_ID = 'laser'

refR = 1.5
refright = -np.pi
refleft = np.pi

pub = rospy.Publisher('visualization_marker', Marker, queue_size =1)


def n(val):
	return ((val+np.pi)%(2*np.pi)-np.pi)

def callbackLOld(msg):
	distances = np.array(msg.ranges)	
	if not len(distances):
		return	
	thetas = -2*np.pi*np.array(range(len(distances)))/(len(distances)-1)
	# Remove nans	
	thetas = thetas[~np.isnan(distances)]
	distances = distances[~np.isnan(distances)]
	# Convert to cartesian
	x = distances * np.cos(thetas)
	y = distances * np.sin(thetas)
	# filter angle range
	cond = np.logical_and(-7*np.pi/4 <thetas, thetas<-5*np.pi/4 )
	x = x[cond]
	y = y[cond]
	# Regression
	X = np.vstack((x,np.ones_like(x))).T
	theta=np.linalg.inv(X.T.dot(X)).dot(X.T).dot(y)
	print(theta)

def callback(msg):
	data = msg.line_segments
	L = len(data)
	if L:
		### creation of markers
		M = Marker()
		M.ns = 'targets'
		M.action = 0
		M.id = 0
		M.type = 7
		M.pose.orientation.w = 1.0
		M.scale.x = 0.1
		M.scale.y = 0.1
		M.scale.z = 0.1
		# ~ M.color.r = 0.
		# ~ M.color.g = 0.
		# ~ M.color.b = 1.
		# ~ M.color.a = 1.
		
		eR = np.zeros(L)
		X = np.zeros(L)
		Y = np.zeros(L)
		for i in range(L):
			p = Point()
			c = ColorRGBA()
			eR[i] = data[i].radius - refR
			if eR[i] > 0:
				x = np.cos(data[i].angle) * eR[i]
				y = np.sin(data[i].angle) * eR[i]  
			else:
				x = np.cos(n(data[i].angle+180)) * np.abs(eR[i])
				y = np.sin(n(data[i].angle+180)) * np.abs(eR[i])  
			X[i] = x
			Y[i] = y
			p.x = x
			p.y = y
			p.z = 0.0
			c.r = 0.
			c.g = 0.
			c.b = 1.
			c.a = 1.0
			M.points.append(p)
			M.colors.append(c)
			
		minR = np.argmin(np.abs(eR))
		q = ColorRGBA()
		q.r = 0.
		q.g = 1.
		q.b = 0.
		q.a = 1.0
		M.colors[minR] = q
		
		M.header.frame_id = FRAME_ID
		M.header.stamp = rospy.Time.now()
		pub.publish(M)
		
	
	#print(X,Y,(X[minR],Y[minR]))
	
	 

rospy.init_node('scan_lines')
sub = rospy.Subscriber('/line_segments', LineSegmentList, callback, queue_size=1)
rospy.spin()
