#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Point, Polygon
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import numpy as np
import matplotlib.pyplot as plt 
import matplotlib.animation as animation

FRAME_ID = 'lidar_frame'
topic = 'data'

pubMarker = rospy.Publisher('visualization_marker', Marker,queue_size=1)

# ~ failed attempt matplotlib 
# ~ fig = plt.figure()
# ~ ax = fig.add_subplot(1,1,1)
# ~ ax.set_xlim(-13,13)
# ~ ax.set_ylim(-13,13)
# ~ drawx = np.linspace(-13,13,100)
# ~ mG = 0
# ~ interG = 0
# ~ drawy = mG*drawx + interG
# ~ ax.plot(drawx, drawy)
# ~ mG = msg.points[0].x
# ~ interG = msg.points[0].y
# ~ drawy = mG*drawx + interG
# ~ ax.cla()
# ~ ax.plot(drawx,drawy)
# ~ plt.pause(0.001)

def draw(msg):
	m = msg.points[0].x
	interc = msg.points[0].y
	M = Marker()
	M.ns = 'wall'
	M.action = 0
	M.id = 0
	M.type = 0  # arrow
	M.pose.orientation.w = 1.0
	M.scale.x = 0.05
	M.scale.y = 0.05
	M.scale.z = 0.0
	M.color.a = 1.0
	M.color.b = 0.0
	M.color.g = 0.0
	M.color.r = 1.0
	p = Point()
	p.x = 7
	p.y = -m*(7) - interc
	p.z = 0.0
	M.points.append(p)
	p = Point()
	p.x = -7
	p.y = -m*(-7) - interc
	p.z = 0.0
	M.points.append(p)
	M.header.frame_id = FRAME_ID
	M.header.stamp = rospy.Time.now()
	pubMarker.publish(M)
	
def main():
	rospy.init_node('viz')
	sub = rospy.Subscriber(topic, Polygon, draw, queue_size=1)
	rospy.spin()
	
if __name__ == '__main__':
	main()
