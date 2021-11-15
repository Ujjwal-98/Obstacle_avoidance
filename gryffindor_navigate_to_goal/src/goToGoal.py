#!/usr/bin/env python
from logging import error
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from time import time, sleep
import sys
import signal
from nav_msgs.msg import Odometry
# from scipy.spatial.transform import Rotation

'''
Mahdi Ghanei and Ujjwal Gupta
'''
# robot velocity topic
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 5)

########################
### global variables ###
########################
t_old = time()
ei = 0.0  # integral error
ei_2 = 0.0

robot_pose = (0, 0, 0) # x, y, yaw
path = 0			   # segment of path


old_yaw = None
yaw_dist = None
dt = None
encounter_path1 = 0
encounter_path2 = 0




def callback(data):
	'''read desired angular data and publish to robot
	'''

	global path
	global yaw_0

	global encounter_path1
	global encounter_path2


	waypoints = [(1.5+0.1, 0), (1.5+0.1, 1.4+0.1), (0, 1.4+0.1)]

	### read the obstacle pose
	obs_angle = data.angular.z
	if obs_angle > 180:
		obs_angle = obs_angle - 360
	obs_dist = data.linear.x
	look_ahead_dist = min(data.linear.y, 0.5)



    ##################################
	########## Switching mode ############
	##################################

	ww = waypoints[path]
	# scale down waypoints
	w = (ww[0], ww[1])

	theta_d = np.arctan2(w[1]-robot_pose[1], w[0]-robot_pose[0])		# vector robot to next_waypoint

	theta_r = robot_pose[2]												# current robot angle
	dist = np.sqrt((w[0]-robot_pose[0])**2 + (w[1]-robot_pose[1])**2)	# distance to next waypoint

	##################################
	########## Controller ############
	##################################

	################################
	### publish to robot

	if True:
		algorithm = 'waypoint'
		if obs_dist < 0.3 and abs(obs_angle) < 90 and (algorithm=='boundary'):
			print('obstacle detected at: {:.3f}, {:.3f}'.format(obs_dist, obs_angle))


			if obs_angle > 0:
				obs_angle_d =  85*np.pi/180
				
			else:
				obs_angle_d = -85*np.pi/180
				
			dist_d = 0.3
			mode = "avoid"
			print('path {} - obs_dist {:.3}, realtive_angle{:.3f} '.format(path, look_ahead_dist, obs_angle))
			print('estimated position:{:.3f} {:.3f},  {:.3f}'.format(robot_pose[0], robot_pose[1], robot_pose[2]))
			vel, omega = controller(obs_angle*np.pi/180, obs_angle_d, look_ahead_dist, dist_d, mode)
			omega = - omega
			print('vel, omega: ({:.3f}, {:.3f})\n'.format(vel, omega))


		if obs_dist < 0.35 and abs(obs_angle) < 90 and (algorithm=='waypoint'):
			
			obs_angle = -obs_angle
			sign_value = np.sign(obs_angle)
			if sign_value == 0:
				sign_value = 1

			lookahead_angle = sign_value*np.pi/3 + obs_angle*(np.pi/180)
			wp = getPoseGlobal(l=0.6, th=lookahead_angle)
			theta_d = np.arctan2(wp[1]-robot_pose[1], wp[0]-robot_pose[0])
			dist = np.sqrt((wp[0]-robot_pose[0])**2 + (wp[1]-robot_pose[1])**2)
			print('intermediate waypoint following')
			print('obstacle detected at: {:.2f}, {:.2f}'.format(obs_dist, obs_angle))
			print('virutal waypoint ({:.2f}, {:.2f}, {:.2f})'.format(wp[0], wp[1], lookahead_angle))
			print('path {} - dist {:.3}, realtive_angle{:.3f} '.format(path, dist, theta_d))
			print('estimated position:{:.3f} {:.3f},  {:.3f}'.format(robot_pose[0], robot_pose[1], robot_pose[2]))

			if path == 1 and encounter_path1<8:
				vel, omega = controller(theta_r, theta_d, dist=0, dist_d=0, mode='follow')
				encounter_path1 += 1
			
			elif path == 2 and encounter_path2<8:
				vel, omega = controller(theta_r, theta_d, dist=0, dist_d=0, mode='follow')
				encounter_path2 += 1
			else:
				vel, omega = controller(theta_r, theta_d, dist, dist_d=0, mode='follow')
			print('vel, omega: ({:.3f}, {:.3f})\n'.format(vel, omega))



		else:
			mode = "follow"
			print('path {} - dist {:.3}, realtive_angle{:.3f} '.format(path, dist, theta_d))
			print('estimated position:{:.3f} {:.3f},  {:.3f}'.format(robot_pose[0], robot_pose[1], robot_pose[2]))
			dist_d = 0
			vel, omega = controller(theta_r, theta_d, dist, dist_d, mode)
			print('vel, omega: ({:.3f}, {:.3f})\n'.format(vel, omega))


		



	if path == 2 and dist < 0.05:
		pubRobot(0,0)
		print('\n\n\n****************************************Reached goal!', path)
		sleep(100.0)
		#sys.exit(0)

	if dist < 0.01:
		path = path + 1
		pubRobot(0,0)
		print('\n\n\n****************************************path changed!', path)
		sleep(10.0)

	pubRobot(vel, omega)


def getPoseGlobal(l, th):
	p0 = robot_pose
 	T = [[np.cos(p0[2]), -np.sin(p0[2]), p0[0]],
		 [np.sin(p0[2]), np.cos(p0[2]), p0[1]],
		 [0, 0, 1 ]
		]

	p =  np.array([l*np.cos(th), l*np.sin(th), 1]).T
	p_global = np.matmul(T, p)

	return p_global



def pubRobot(vel, omega):
	pose = Twist()
	pose.angular.z = omega
	pose.linear.x = vel
	pub.publish(pose)


def controller(theta_r, theta_d, dist, dist_d, mode):
	''' controller to go to the waypoint
	'''
	global ei
	global ei_2
	global t_old
	global dt

	dt = time()- t_old



	### angle controller
	kp = 0.5
	kd = .2
	ki = 0.1
	ei_max = 1.0

	error_angle = theta_r - theta_d
	if error_angle > np.pi:
		error_angle = error_angle - 2*np.pi
	if error_angle < -np.pi:
		error_angle = error_angle + 2*np.pi

	ep = kp*(error_angle)
	ed = kd*(error_angle)/dt
	ei = ki*(error_angle) + ei
	ei = min(ei, ei_max)

	omega = ep + ed + ei
	

	### distance controller
	if mode == "follow":
		kp_2 = 0.15 # 1.0
		kd_2 = 0.05 #0.1
		ki_2 = 0.0
		ei_2_max = 1.0
	else: 					# obstacle
		kp_2 = 0.5 # 1.0
		kd_2 = 0.25 #0.1
		ki_2 = 0.0
		ei_2_max = 1.0



	ep_2 = kp_2*(dist - dist_d)
	ed_2 = kd_2*(dist - dist_d)/dt
	ei_2 = ki_2*(dist - dist_d) + ei_2
	ei_2 = min(ei_2, ei_2_max)

	vel = ep_2 + ed_2 + ei_2

	t_old = time()

	return vel, -omega




def callback2(data):
	'''get the pose of the robot in global frame
	'''
	global robot_pose

	x = data.pose.pose.position.x
	y = data.pose.pose.position.y

	### calculate yaw
	q1 = data.pose.pose.orientation.x
	q2 = data.pose.pose.orientation.y
	q3 = data.pose.pose.orientation.z
	q4 = data.pose.pose.orientation.w

	siny_cosp = 2 * (q4 * q3 + q1 * q2)
	cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3)
	yaw = np.arctan2(siny_cosp, cosy_cosp)



	robot_pose = (x, y, yaw)


def goToGoal():
	'''read omega and publish to robot
	'''
	#signal.signal(signal.SIGINT, signal_handler)
	rospy.init_node('goToGoal', anonymous=True)

	print('arugment', sys.argv[1])
	zero_vel =  int(sys.argv[1])

	if zero_vel>0:
		oldtime = time()
		while (time() - oldtime) < 5:
			pubRobot(0,0)
		print('published zero')
	
	else:
		rospy.Subscriber("/obs_pose", Twist, callback, queue_size = 1)
		rospy.Subscriber("/odom", Odometry, callback2, queue_size = 1)	


	rospy.spin()

	
if __name__ == '__main__':
	goToGoal()

	stop = Twist()
	if rospy.is_shutdown():
		pub.publish(stop)
