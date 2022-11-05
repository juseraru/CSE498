#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Point32
from geometry_msgs.msg import Polygon
from std_msgs.msg import ColorRGBA
from laser_line_extraction.msg import LineSegmentList
import numpy as np
import matplotlib.pyplot as plt 
import matplotlib.animation as animation

FRAME_ID = 'lidar_frame'
topic = '/LiDAR/LD06'

#pub = rospy.Publisher('wall', Point32, queue_size =1)
pubdata = rospy.Publisher('data',Polygon,queue_size=1)



def n(val):
	return ((val+np.pi)%(2*np.pi)-np.pi)

def callback(msg):
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
	m, inter = theta
	p = Point32()
	p.x = np.arctan(m)
	p.y = inter
	p.z = 0.0
	#pub.publish(p)
	pol = Polygon()
	pol.points.append(p)
	for i in range(len(x)):
		p = Point32()
		p.x = x[i]
		p.y = y[i]
		p.z = 0.0
		pol.points.append(p)
	pubdata.publish(pol)
	print(pol.points[0].x,pol.points[0].y)

	
def main():
	rospy.init_node('walls')
	sub = rospy.Subscriber(topic, LaserScan, callback, queue_size=1)
	#plt.show(block=True)
	rospy.spin()

if __name__ == '__main__':
	main()
