#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np

topic = '/LiDAR/LD06'

pubdata = rospy.Publisher('data',Float32MultiArray,queue_size=1)

## laser angle
left = 1/2 * np.pi
right = 3/2 * np.pi
front = 0.0000001
back = np.pi
direc = {'left':left, 'back':back, 'right':right, 'front':front}

cone_angle = np.pi/8


def n(val, f=True):
	if f:
		return ((val+np.pi)%(2*np.pi)-np.pi)
	else:
		return val

def callback(msg):
	distances = np.array(msg.ranges)	
	if not len(distances):
		return	
	thetas = 2*np.pi*np.array(range(len(distances)))/(len(distances)-1)
	# Remove nans	
	thetas = thetas[~np.isnan(distances)]
	distances = distances[~np.isnan(distances)]
	# Convert to cartesian
	x = distances * np.cos(thetas)
	y = distances * np.sin(thetas)
	
	p = Float32MultiArray()
	p.data = [0,0,0,0]
	f = False
	for i, (k,v) in enumerate(direc.items()):	
		# filter angle range on the left
		if k == 'front':
			thetas = n(thetas)
			cond = np.logical_and(n(v-cone_angle) <thetas, thetas<n(v+cone_angle))
			xc = x[cond]
			yc = y[cond]
		else:
			cond = np.logical_and(v-cone_angle <thetas, thetas<v+cone_angle)
			xc = x[cond]
			yc = y[cond]
			
		# Regression
		X = np.vstack((xc,np.ones_like(xc))).T
		w = np.linalg.inv(X.T.dot(X)).dot(X.T).dot(yc)
		if k == 'front' or k == 'back':
			p.data[i] = -w[1]/(w[0]+1e-6)
		else:
			p.data[i] = w[1]  ### distance to wall
	
	### Left,Back,Right,Front ###
	pubdata.publish(p)
	print(p.data)

	
def main():
	rospy.init_node('wall_detector')
	sub = rospy.Subscriber(topic, LaserScan, callback, queue_size=1)
	rospy.spin()

if __name__ == '__main__':
	main()
