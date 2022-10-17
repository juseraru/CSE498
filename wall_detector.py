#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt 


DRAW = True


	


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
	
	print(theta)
	
	
	if DRAW:
		plt.plot(x,y, '.')
		plt.plot([0,1],[0,0], '-')
		plt.plot([0,0],[0,1], '-')
		
		u = np.linspace(-1,1,20)
		v = u*theta[0] + theta[1]
		plt.plot(u,v)
		
		plt.grid()
		plt.show()
	
		

rospy.init_node('scan_values')
sub = rospy.Subscriber('/base_scan', LaserScan, callback, queue_size=1)
rospy.spin()
