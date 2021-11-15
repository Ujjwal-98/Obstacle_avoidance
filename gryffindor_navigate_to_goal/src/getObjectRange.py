#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

pub = rospy.Publisher("obs_pose", Twist, queue_size = 5)


'''
Mahdi Ghanei and Ujjwal Gupta
'''

def callback(msg):

	min_dist = np.inf
	for i in range(360):
		if msg.ranges[i] <= min_dist and msg.ranges[i] > 0.0:
			min_dist = msg.ranges[i]
			index = i
		
	#print('dist', avg_dist, '\n')
	if index < 180:
		look_ahead_angle = index - 5
	else: 
		look_ahead_angle = index + 5
	
	if look_ahead_angle > 359:
		look_ahead_angle = look_ahead_angle - 360
	if look_ahead_angle < 0:
		look_ahead_angle = look_ahead_angle + 360
	
	look_ahead_dist = msg.ranges[look_ahead_angle]
	pose = Twist()
	pose.angular.z = index
	pose.linear.x = min_dist
	pose.linear.y = look_ahead_dist
	pub.publish(pose)

     	
def obs_range():  #obstacle range
     	
	rospy.init_node('laser_subscriber', anonymous=True)
	rospy.Subscriber('/scan', LaserScan, callback)
	
   	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
 
if __name__ == '__main__':
	np.set_printoptions(precision=3)
	#stop = Twist()
	obs_range()
	

	# if rospy.is_shutdown():
	# 	pub.publish(stop)
