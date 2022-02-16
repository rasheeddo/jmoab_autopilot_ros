#!/usr/bin/env python

import rospy
import rospkg
import rosparam
import yaml
import dynamic_reconfigure.client
import os

import numpy as np
from numpy import pi
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Int32MultiArray, Float32MultiArray, Int8, Bool
import geometry_msgs.msg
from geometry_msgs.msg import Polygon, PolygonStamped, Point32
import tf2_ros
import time
from simple_pid import PID
import argparse

#                                   Y
#                                   |
#                                   | 
#                               |   |                   |<stop_dist>|
#                               |<-----robot_length---->|           .
#                               |   |                   |           .
#                               |   |                   |
#                               |   |                   |           .
#                               A___|___________________B\          .
#                               |   |       ^           |  \        .
#                               |   |       |           |    \      .
#                               |   |       |           |      \    .
#                               |   |       |           |        \  .
#                               |   |   robot_width     |          \ 
#   ----------------------------|---O-------------------|---------- T--------X
#                               |   |       |           |          / 
#                               |   |       |           |        / 
#                               |   |       |           |      /
#                               |   |       |           |    /
#                               |   |       |           |  /
#                               C___|_______V___________D/


class GreenhouseNav:

	def __init__(self, param_file):

		rospy.init_node("greenhouse_nav_node", anonymous=True)

		self.left_wall_scan_pub = rospy.Publisher("/left_wall_scan", LaserScan, queue_size=1)
		self.right_wall_scan_pub = rospy.Publisher("/right_wall_scan", LaserScan, queue_size=1)
		self.front_wall_scan_pub = rospy.Publisher("/front_wall_scan", LaserScan, queue_size=1)
		self.left_wall_scan = LaserScan()
		self.right_wall_scan = LaserScan()
		self.front_wall_scan = LaserScan()

		self.sbus_cmd_pub = rospy.Publisher("/sbus_cmd", Int32MultiArray, queue_size=10)
		self.sbus_cmd = Int32MultiArray()

		self.foot_pub = rospy.Publisher("/footprint", PolygonStamped, queue_size=1)
		self.front_stop_pub = rospy.Publisher("/front_stop_zone", PolygonStamped, queue_size=1)
		self.back_stop_pub = rospy.Publisher("/back_stop_zone", PolygonStamped, queue_size=1)

		self.video_trig_pub = rospy.Publisher("/video_trigger", Bool, queue_size=1)
		self.video_trig_msg = Bool()

		self.pg = PolygonStamped()
		self.pg_front_stop = PolygonStamped()
		self.pg_back_stop = PolygonStamped()

		self.br = tf2_ros.TransformBroadcaster()
		self.t = geometry_msgs.msg.TransformStamped()

		################################ ROS Parameters #####################################
		sv_node = "greenhouse_nav_server_node"
		# https://answers.ros.org/question/169866/load-yaml-with-code/
		# load yaml file to rosparam server without running server on python
		f = open(param_file, 'r')
		yamlfile = yaml.load(f)
		rosparam.upload_params("/", yamlfile)

		## get parameter from rosparam server that we just loaded above
		self.wall_scan_ang = rosparam.get_param(sv_node+"/wall_scan_ang")
		self.front_stop_dist = rosparam.get_param(sv_node+"/front_stop_dist")
		self.right_dist_in_lane = rosparam.get_param(sv_node+"/right_dist_in_lane")
		self.left_dist_in_lane = rosparam.get_param(sv_node+"/left_dist_in_lane")
		self.max_start_str = rosparam.get_param(sv_node+"/max_start_str")
		self.min_start_str = rosparam.get_param(sv_node+"/min_start_str")
		self.str_mid = rosparam.get_param(sv_node+"/str_mid")
		self.wf_str_adj = rosparam.get_param(sv_node+"/wf_str_adj")
		self.lc_str_adj = rosparam.get_param(sv_node+"/lc_str_adj")
		self.rh_str_adj = rosparam.get_param(sv_node+"/rh_str_adj")
		self.ut_str_adj = rosparam.get_param(sv_node+"/ut_str_adj")
		self.wf_thr = rosparam.get_param(sv_node+"/wf_thr")
		self.lc_thr = rosparam.get_param(sv_node+"/lc_thr")
		self.wf_setpoint = rosparam.get_param(sv_node+"/wf_setpoint")
		self.wf_p = rosparam.get_param(sv_node+"/wf_p")
		self.wf_i = rosparam.get_param(sv_node+"/wf_i")
		self.wf_d = rosparam.get_param(sv_node+"/wf_d")
		self.lc_setpoint = rosparam.get_param(sv_node+"/lc_setpoint")
		self.lc_p = rosparam.get_param(sv_node+"/lc_p")
		self.lc_i = rosparam.get_param(sv_node+"/lc_i")
		self.lc_d = rosparam.get_param(sv_node+"/lc_d")
		self.rh_p = rosparam.get_param(sv_node+"/rh_p")
		self.rh_i = rosparam.get_param(sv_node+"/rh_i")
		self.rh_d = rosparam.get_param(sv_node+"/rh_d")
		self.ut_p = rosparam.get_param(sv_node+"/ut_p")
		self.ut_i = rosparam.get_param(sv_node+"/ut_i")
		self.ut_d = rosparam.get_param(sv_node+"/ut_d")

		################################ Dynamic Reconfigure #####################################
		print("Wait for server node...")
		try:
			## This we will get live data from rqt_reconfigure
			self.client = dynamic_reconfigure.client.Client(sv_node, timeout=2, config_callback=self.param_callback)
			print("Open rqt_reconfigure to tune paramters realtime")
		except rospy.ROSException:
			print("Server node is not alive, load parameters from GpsWaypoints.yaml file")
			pass

		########################### Robot polygon shape ##########################
		# Physical params
		self.robot_length = 0.455
		self.robot_width = 0.476
		self.from_back_to_lidar = 0.355

		# Behavior params
		self.stop_dist = self.front_stop_dist	#1.50    #0.25
		self.avoid_length = 0.50 # 1.0
		self.avoid_width = 0.75

		# AB is robot length
		# BD is robot width
		# AC_O is lidar offset from back side

		self.AB = self.robot_length
		self.BD = self.robot_width
		self.AC_O = self.from_back_to_lidar

		# Footprint coords
		self.A = (-self.AC_O, self.robot_width/2)
		self.B = (self.robot_length-self.AC_O, self.robot_width/2)
		self.C = (-self.AC_O, -self.robot_width/2)
		self.D = (self.robot_length-self.AC_O, -self.robot_width/2)
		# Front Stop and Avoid zone coords
		self.T = ((self.robot_length-self.AC_O+self.stop_dist), 0)

		# Robot's footprint is ABCD
		# O is where the lidar is placing
		# robot is facing forward on X-direction
		# BDT and ACW are the STOP_ZONE
		## Stop zone of triangle shape
		## Front
		# slope and y-intercept of first quadant
		self.m1 = (self.T[1] - self.B[1])/(self.T[0] - self.B[0])
		self.b1 = self.B[1] - self.m1*(self.B[0])
		# slope and y-intercept of second quadant
		self.m2 = (self.T[1] - self.D[1])/(self.T[0] - self.D[0])
		self.b2 = self.D[1] - self.m2*(self.D[0])


		########################### Wall scan repeater ##########################
		self.ranges_list = None
		self.angle_list = None
		self.angle_min = None
		self.angle_max = None
		self.angle_increment = None
		self.intensities = None

		### Wall detect params
		## Left
		self.left_wall_detect_deg = self.wall_scan_ang	# 15.0
		self.pcs_on_pie = 1.0/(self.left_wall_detect_deg/360.0)
		self.shifted_from_mid = int(2019.0/self.pcs_on_pie)		# ranges_len is constant as 2019
		self.leftWall_mid_idx = 1515 - self.shifted_from_mid	#	1515 				# left side, 3/4 of 2019
		self.leftWall_left_shifted_idx = self.leftWall_mid_idx - self.shifted_from_mid
		self.leftWall_right_shifted_idx = self.leftWall_mid_idx + self.shifted_from_mid

		## Right 
		self.right_wall_detect_deg = self.left_wall_detect_deg
		self.rightWall_mid_idx = 505 + self.shifted_from_mid	#	505 	# 1/4 of 2019
		self.rightWall_left_shifted_idx = self.rightWall_mid_idx - self.shifted_from_mid
		self.rightWall_right_shifted_idx = self.rightWall_mid_idx + self.shifted_from_mid
		## Front
		# self.front_wall_detect_deg = 10.0
		# pcs_on_pie_F = 1.0/(self.front_wall_detect_deg/360.0)
		# self.shifted_from_mid_F = int(int(2019.0/pcs_on_pie_F)/2.0)
		# self.frontWall_mid_idx = 1010
		# self.frontWall_left_shifted_idx = self.frontWall_mid_idx - self.shifted_from_mid_F
		# self.frontWall_right_shifted_idx = self.frontWall_mid_idx + self.shifted_from_mid_F

		########################### Robot Parameters ##########################
		self.sbus_steering_mid = 1024
		self.sbus_throttle_mid = 1024	#1024

		self.update_local_param()

		########################### PID Parameters ##########################
		#####################
		## PID wall follow ##
		#####################
		# self.kp_wf = 200.0
		# self.ki_wf = 0.0
		# self.kd_wf = 0.00
		# self.setpoint_wf = 0.5	#0.75	#self.half_lane_width 	# 0.28

		self.pid_wf = PID(self.wf_p, self.wf_i, self.wf_d, setpoint=self.wf_setpoint)
		self.pid_wf.tunings = (self.wf_p, self.wf_i, self.wf_d)
		self.pid_wf.sample_time = 0.001
		self.wf_out_range = 100.0
		self.pid_wf.output_limits = (-self.wf_out_range, self.wf_out_range)
		self.pid_wf.auto_mode = True

		#####################
		## PID lane change ##
		#####################
		# self.kp_lc = 150.0
		# self.ki_lc = 0.0
		# self.kd_lc = 0.00
		# self.setpoint_lc = 0.5	#0.75	#self.half_lane_width 	# 0.28

		self.pid_lc = PID(self.lc_p, self.lc_i, self.lc_d, setpoint=self.lc_setpoint)
		self.pid_lc.tunings = (self.lc_p, self.lc_i, self.lc_d)
		self.pid_lc.sample_time = 0.001
		self.lc_out_range = 100.0
		self.pid_lc.output_limits = (-self.lc_out_range, self.lc_out_range)
		self.pid_lc.auto_mode = False

		################
		## PID Repose ##
		################
		# self.kp_rp = 10.0
		# self.ki_rp = 0.0
		# self.kd_rp = 0.00
		self.rh_setpoint = 0.0

		self.pid_rh = PID(self.rh_p, self.rh_i, self.rh_d, setpoint=self.rh_setpoint)
		self.pid_rh.tunings = (self.rh_p, self.rh_i, self.rh_d)
		self.pid_rh.sample_time = 0.001
		self.rh_out_range = 100.0
		self.pid_rh.output_limits = (-self.rh_out_range, self.rh_out_range)
		self.pid_rh.auto_mode = False

		################
		## PID U-Turn ##
		################
		# self.kp_ut = 5.0
		# self.ki_ut = 0.0
		# self.kd_ut = 0.00
		self.ut_setpoint = 0.0

		self.pid_ut = PID(self.ut_p, self.ut_i, self.ut_d, setpoint=self.ut_setpoint)
		self.pid_ut.tunings = (self.ut_p, self.ut_i, self.ut_d)
		self.pid_ut.sample_time = 0.001
		self.ut_out_range = 100.0
		self.pid_ut.output_limits = (-self.ut_out_range, self.ut_out_range)
		self.pid_ut.auto_mode = False

		##################### Robot Stop zone ########################
		self.FRONT_STOP = False

		self.X = np.array([])
		self.Y = np.array([])
		self.PX = np.array([])
		self.PY = np.array([])

		#################### Lidar checking distance #####################
		self.left_dist_per = 0.0 # initial value

		self.left_scan_trig_idx = 1300

		self.mode = "WallFollow"
		self.shelf_counter = 0
		self.cart_mode = 1
		self.prev_cart_mode = 1
		self.video_trig_flag = False

		######################### Subscriber node ##########################
		rospy.Subscriber("/atcart_scan", LaserScan, self.scan_callback)
		# rospy.Subscriber("/jmoab_imu_raw", Imu, self.imu_callback)
		rospy.Subscriber("/jmoab_compass", Float32MultiArray, self.imu_callback)
		rospy.Subscriber("/atcart_mode", Int8, self.atcart_mode_callback)

		########################### Loop ###################################

		self.run()

		rospy.spin()

	def param_callback(self, config):
		self.wall_scan_ang = config["wall_scan_ang"]
		self.front_stop_dist = config["front_stop_dist"]
		self.right_dist_in_lane = config["right_dist_in_lane"]
		self.left_dist_in_lane = config["left_dist_in_lane"]
		self.max_start_str = config["max_start_str"]
		self.min_start_str = config["min_start_str"]
		self.str_mid = config["str_mid"]
		self.wf_str_adj = config["wf_str_adj"]
		self.lc_str_adj = config["lc_str_adj"]
		self.rh_str_adj = config["rh_str_adj"]
		self.ut_str_adj = config["ut_str_adj"]
		self.wf_thr = config["wf_thr"]
		self.lc_thr = config["lc_thr"]
		self.wf_setpoint = config["wf_setpoint"]
		self.wf_p = config["wf_p"]
		self.wf_i = config["wf_i"]
		self.wf_d = config["wf_d"]
		self.lc_setpoint = config["lc_setpoint"]
		self.lc_p = config["lc_p"]
		self.lc_i = config["lc_i"]
		self.lc_d = config["lc_d"]
		self.rh_p = config["rh_p"]
		self.rh_i = config["rh_i"]
		self.rh_d = config["rh_d"]
		self.ut_p = config["ut_p"]
		self.ut_i = config["ut_i"]
		self.ut_d = config["ut_d"]

		self.pid_wf.tunings = (self.wf_p, self.wf_i, self.wf_d)
		self.pid_lc.tunings = (self.lc_p, self.lc_i, self.lc_d)
		self.pid_rh.tunings = (self.rh_p, self.rh_i, self.rh_d)
		self.pid_ut.tunings = (self.ut_p, self.ut_i, self.ut_d)

		self.update_local_param()

	def update_local_param(self):
		## Steering
		self.sbus_steering_max_DB = self.max_start_str
		self.sbus_steering_min_DB = self.min_start_str

		## In-lanes wall follow mode
		self.sbus_steering_adj_FL = self.wf_str_adj
		self.sbus_steering_max_FL = self.sbus_steering_max_DB + self.sbus_steering_adj_FL
		self.sbus_steering_min_FL = self.sbus_steering_min_DB - self.sbus_steering_adj_FL
		self.sbus_throttle_fwd_FL = self.wf_thr #self.sbus_throttle_mid + 81

		## Lane changing mode
		self.sbus_steering_adj_LC = self.lc_str_adj
		self.sbus_steering_max_LC = self.sbus_steering_max_DB + self.sbus_steering_adj_LC
		self.sbus_steering_min_LC = self.sbus_steering_min_DB - self.sbus_steering_adj_LC
		self.sbus_throttle_fwd_LC = self.lc_thr #self.sbus_throttle_mid + 120

		## Repose mode
		self.sbus_steering_adj_RH = self.rh_str_adj
		self.sbus_steering_max_RH = self.sbus_steering_max_DB + self.sbus_steering_adj_RH
		self.sbus_steering_min_RH = self.sbus_steering_min_DB - self.sbus_steering_adj_RH

		## UTurn mode
		self.sbus_steering_adj_UT = self.ut_str_adj
		self.sbus_steering_max_UT = self.sbus_steering_max_DB + self.sbus_steering_adj_UT
		self.sbus_steering_min_UT = self.sbus_steering_min_DB - self.sbus_steering_adj_UT

		self.stop_dist = self.front_stop_dist
		self.T = ((self.robot_length-self.AC_O+self.stop_dist), 0)
		self.m1 = (self.T[1] - self.B[1])/(self.T[0] - self.B[0])
		self.b1 = self.B[1] - self.m1*(self.B[0])
		# slope and y-intercept of second quadant
		self.m2 = (self.T[1] - self.D[1])/(self.T[0] - self.D[0])
		self.b2 = self.D[1] - self.m2*(self.D[0])

		## Wall scan params
		# Left wall
		self.left_wall_detect_deg = self.wall_scan_ang
		self.pcs_on_pie = 1.0/(self.left_wall_detect_deg/360.0)
		self.shifted_from_mid = int(2019.0/self.pcs_on_pie)		# ranges_len is constant as 2019
		self.leftWall_mid_idx = 1515 - self.shifted_from_mid	#	1515 				# left side, 3/4 of 2019		
		self.leftWall_left_shifted_idx = self.leftWall_mid_idx - self.shifted_from_mid
		self.leftWall_right_shifted_idx = self.leftWall_mid_idx + self.shifted_from_mid
		# Rigth wall
		self.right_wall_detect_deg = self.left_wall_detect_deg
		self.rightWall_mid_idx = 505 + self.shifted_from_mid	#	505 	# 1/4 of 2019
		self.rightWall_left_shifted_idx = self.rightWall_mid_idx - self.shifted_from_mid
		self.rightWall_right_shifted_idx = self.rightWall_mid_idx + self.shifted_from_mid

	def map_with_limit(self, val, in_min, in_max, out_min, out_max):

		# out = ((val - in_min) * ((out_max - out_min) / (in_max - in_min))) + out_min
		## in_min must be the minimum input 
		## in_max must be the maximum input

		## out_min is supposed to map with in_min value
		## out_max is sipposed to map with in_max value
		## out_min can be less/more than out_max, doesn't matter


		m = (out_max - out_min)/(in_max - in_min)
		out = m*(val - in_min) + out_min

		if out_min > out_max:
			if out > out_min:
				out = out_min
			elif out < out_max:
				out = out_max
			else:
				pass
		elif out_max > out_min:
			if out > out_max:
				out = out_max
			elif out < out_min:
				out = out_min
			else:
				pass
		else:
			pass

		# print(m, val, in_min, in_max, out_min, out_max)

		return out

	def inFrontStopZone(self, x,y):

		if (self.B[0] < x) and (x < self.T[0]):
			if (self.D[1] < y) and (y < self.B[1]):
				y_cal1 = self.m1*x + self.b1
				y_cal2 = self.m2*x + self.b2
				# print("y_cal1", y_cal1)
				# print("y_cal2", y_cal2)
				if (y < y_cal1) and (y > y_cal2):
					return True
				else:
					return False

			else:
				return False

		else:
			return False

	def scan_callback(self, msg):
		self.ranges_list = msg.ranges
		self.angle_min = msg.angle_min
		self.angle_max = msg.angle_max
		self.angle_increment = msg.angle_increment
		self.received_time = time.time()	#rospy.Time.now()
		self.intensities = msg.intensities

		self.angle_list = np.arange((self.angle_min), self.angle_max, self.angle_increment)

		## copy original scan msg to wall_scan msg
		## and publish wall_scan topic immediately
		self.left_wall_scan.header.stamp = rospy.Time.now()
		self.left_wall_scan.header.frame_id = "laser_frame"	#"laser_frame"
		self.left_wall_scan.time_increment = msg.time_increment
		self.left_wall_scan.angle_increment = msg.angle_increment
		self.left_wall_scan.angle_min = ((90.0-(2*self.left_wall_detect_deg))*pi)/180.0	#((90-self.left_wall_detect_deg)*pi)/180.0
		self.left_wall_scan.angle_max = (90.0*pi)/180.0	#((90+self.left_wall_detect_deg)*pi)/180.0
		self.left_wall_scan.scan_time = msg.scan_time
		self.left_wall_scan.range_min = msg.range_min
		self.left_wall_scan.range_max = 5.0	#msg.range_max
		self.left_wall_scan.ranges = msg.ranges[self.leftWall_left_shifted_idx:self.leftWall_right_shifted_idx]
		self.left_wall_scan.intensities = msg.intensities[self.leftWall_left_shifted_idx:self.leftWall_right_shifted_idx]

		## Use this dist to trig the video
		self.left_dist_per = msg.ranges[self.left_scan_trig_idx] # 90 deg of scan


		self.right_wall_scan.header.stamp = rospy.Time.now()
		self.right_wall_scan.header.frame_id = "laser_frame"	#"laser_frame"
		self.right_wall_scan.time_increment = msg.time_increment
		self.right_wall_scan.angle_increment = msg.angle_increment
		self.right_wall_scan.angle_min = (-90.0*pi)/180.0	#((-90-self.right_wall_detect_deg)*pi)/180.0
		self.right_wall_scan.angle_max = ((-90+(2*self.right_wall_detect_deg))*pi)/180.0	#((-90+self.right_wall_detect_deg)*pi)/180.0
		self.right_wall_scan.scan_time = msg.scan_time
		self.right_wall_scan.range_min = msg.range_min
		self.right_wall_scan.range_max = 5.0	#msg.range_max
		self.right_wall_scan.ranges = msg.ranges[self.rightWall_left_shifted_idx:self.rightWall_right_shifted_idx]
		self.right_wall_scan.intensities = msg.intensities[self.rightWall_left_shifted_idx:self.rightWall_right_shifted_idx]

		# self.front_wall_scan.header.stamp = rospy.Time.now()
		# self.front_wall_scan.header.frame_id = "laser_frame"	#"laser_frame"
		# self.front_wall_scan.time_increment = msg.time_increment
		# self.front_wall_scan.angle_increment = msg.angle_increment
		# self.front_wall_scan.angle_min = (-(self.front_wall_detect_deg/2.0)*pi)/180.0	#((-90-self.right_wall_detect_deg)*pi)/180.0
		# self.front_wall_scan.angle_max = ((self.front_wall_detect_deg/2.0)*pi)/180.0	#((-90+self.right_wall_detect_deg)*pi)/180.0
		# self.front_wall_scan.scan_time = msg.scan_time
		# self.front_wall_scan.range_min = msg.range_min
		# self.front_wall_scan.range_max = 5.0	#msg.range_max
		# self.front_wall_scan.ranges = msg.ranges[self.frontWall_left_shifted_idx:self.frontWall_right_shifted_idx]
		# self.front_wall_scan.intensities = msg.intensities[self.frontWall_left_shifted_idx:self.frontWall_right_shifted_idx]


		## if I put this in the run() loop, I will get strange value of x,y sometimes...
		self.X = np.asarray(self.ranges_list, dtype=np.float)*np.cos(np.asarray(self.angle_list, dtype=np.float))
		self.Y = np.asarray(self.ranges_list, dtype=np.float)*np.sin(np.asarray(self.angle_list, dtype=np.float))


		## this for loop takes ~5ms
		for i, (x,y) in enumerate(zip(self.X, self.Y)):
			if self.inFrontStopZone(x,y):
				# print("In FRONT STOP zone: i: {:d}  x {:.7f} y {:.7f}  | range {:.3f}  ang {:.3f}".format(i,x,y, self.ranges_list[i], self.angle_list[i]))
				_front_stop_flag = True
				break
			else:
				_front_stop_flag = False

		## We set FRONT_STOP flag only after finish the for loop
		self.FRONT_STOP = _front_stop_flag

	def atcart_mode_callback(self, msg):
		self.cart_mode = msg.data

		if (self.prev_cart_mode != self.cart_mode):
			if (self.cart_mode == 2):
				self.video_trig_flag = True
			else:
				self.video_trig_flag = False

			self.video_trig_msg.data = self.video_trig_flag
			self.video_trig_pub.publish(self.video_trig_msg) 

		self.prev_cart_mode = msg.data

	def imu_callback(self, msg):

		self.hdg = msg.data[2]

	def ConvertTo180Range(self, deg):

		deg = self.ConvertTo360Range(deg)
		if deg > 180.0:
			deg = -(180.0 - (deg%180.0))

		return deg

	def ConvertTo360Range(self, deg):

		# if deg < 0.0:
		deg = deg%360.0

		return deg


	def find_smallest_diff_ang(self, goal, cur):

		## goal is in 180ranges, we need to convert to 360ranges first

		diff_ang1 = abs(self.ConvertTo360Range(goal) - cur)

		if diff_ang1 > 180.0:

			diff_ang = 180.0 - (diff_ang1%180.0)
		else:
			diff_ang = diff_ang1

		## check closet direction
		compare1 = self.ConvertTo360Range(self.ConvertTo360Range(goal) - self.ConvertTo360Range(cur + diff_ang))
		compare2 = self.ConvertTo180Range(goal - self.ConvertTo180Range(cur + diff_ang))
		# print(compare1, compare2)
		if (abs(compare1) < 0.5) or (compare1 == 360.0) or (abs(compare2) < 0.5) or (compare2 == 360.0):
			sign = 1.0 # clockwise count from current hdg to target
		else:
			sign = -1.0 # counter-clockwise count from current hdg to target


		return diff_ang, sign


	def run(self):
		rate = rospy.Rate(20) # 10hz


		output_pid_wf = 0.0
		output_pid_lc = 0.0
		output_pid_rh = 0.0
		output_pid_ut = 0.0

		sbus_steering = 1024
		sbus_throttle = 1024

		last_hdg = 0.0

		## Flags
		inside_lane_flag = True
		from_inside_lane_flag = False
		lane_change_flag = False
		done_lane_change_flag = False
		repose_flag = False
		done_repose_flag = False

		while not rospy.is_shutdown():


			## If we got scan data
			if self.left_wall_scan.ranges  and self.right_wall_scan.ranges:
				left_wall_ranges_array = np.asarray(self.left_wall_scan.ranges)
				right_wall_ranges_array = np.asarray(self.right_wall_scan.ranges)
				## remove zeros out from array
				left_wall_ranges_array = left_wall_ranges_array[left_wall_ranges_array != 0]
				right_wall_ranges_array = right_wall_ranges_array[right_wall_ranges_array != 0]

				## https://stackoverflow.com/questions/55806118/remove-elements-from-numpy-array-smaller-than-1
				## 0.45 is where the plants come out from shelf
				right_wall_ranges_array = right_wall_ranges_array[(right_wall_ranges_array>0.45)]
				

				if (len(right_wall_ranges_array) == 0):
					## if nothing inside array, we set the min as 0.2, so PID_WF can bump the cart back to middle again
					right_wall_ranges_min = 0.2 
				else:
					right_wall_ranges_min = np.min(right_wall_ranges_array)

				##############################################################
				### Step 1 and 4, Inside lane right-wall-follow navigation ###
				##############################################################
				if inside_lane_flag:

					################################################################################
					### Check if there is big distance on right side, consider it as out of lane ###
					### end of inside lane navigation, switch to lane change nav  				 ###
					################################################################################
					if (right_wall_ranges_min > 1.0):
	
						inside_lane_flag = False
						from_inside_lane_flag = True
						lane_change_flag = True

						self.pid_wf.auto_mode = False
						self.pid_lc.auto_mode = True
						self.pid_rh.auto_mode = False
						self.pid_ut.auto_mode = False

						self.video_trig_flag = False
						self.video_trig_msg.data = self.video_trig_flag
						self.video_trig_pub.publish(self.video_trig_msg) 

						last_hdg = self.hdg
						print("=================== Outside the lane ====================")

					#########################################################################
					### Check if front stop zone has reached the front wall 			  ###
					### Must finished lane change (step 2) before comes to this condition ###
					#########################################################################
					elif (done_lane_change_flag and done_repose_flag and self.FRONT_STOP):
						self.sbus_cmd.data = [1024, 1024]
						turning_flag = True
						inside_lane_flag = False

						self.pid_wf.auto_mode = False
						self.pid_lc.auto_mode = False
						self.pid_rh.auto_mode = False
						self.pid_ut.auto_mode = True

						self.video_trig_flag = False
						self.video_trig_msg.data = self.video_trig_flag
						self.video_trig_pub.publish(self.video_trig_msg) 

						self.sbus_cmd.data = [self.sbus_steering_mid, self.sbus_throttle_mid]
						## from current angle, we want to make it turn 180 deg
						goal_uturn_ang = self.ConvertTo360Range(self.hdg + 180.0)

						print("+++++++++++++++++++++ Something in front stop ++++++++++++++++++++")

					###################################
					### Righ wall-follow navigation ###
					###################################
					else:

						if not (np.isinf(right_wall_ranges_min)):
							output_pid_wf = self.pid_wf(right_wall_ranges_min)

							if output_pid_wf < -0.1:
								sbus_steering = int(self.map_with_limit(output_pid_wf, -self.wf_out_range, 0.0, self.sbus_steering_max_FL, self.sbus_steering_max_DB))
							elif output_pid_wf > 0.1:
								sbus_steering = int(self.map_with_limit(output_pid_wf, 0.0, self.wf_out_range, self.sbus_steering_min_DB, self.sbus_steering_min_FL))
							else:
								sbus_steering = self.str_mid

							# sbus_steering = int(self.map_with_limit(output_pid_wf, -self.wf_out_range, self.wf_out_range, self.sbus_steering_max_FL, self.sbus_steering_min_FL))
							sbus_throttle = self.sbus_throttle_fwd_FL
							
							self.sbus_cmd.data = [sbus_steering, sbus_throttle]

						else:
							self.sbus_cmd.data = [prev_sbus_steering, prev_sbus_throttle]

					self.mode = "WallFollow"
					output_pid = output_pid_wf
				
				#######################################
				### Step 2 lane changing navigation ###
				#######################################
				elif lane_change_flag:

					###################################################
					### Check whether it becames inside lane or not ###
					###################################################
					if (right_wall_ranges_min < self.right_dist_in_lane) and (self.left_dist_per < self.left_dist_in_lane):
						
						
						done_lane_change_flag = True
						lane_change_flag = False
						# from_inside_lane_flag = False

						repose_flag = True
						inside_lane_flag = False 


						self.pid_wf.auto_mode = False
						self.pid_lc.auto_mode = False
						self.pid_rh.auto_mode = True
						self.pid_ut.auto_mode = False

						self.sbus_cmd.data = [self.sbus_steering_mid, self.sbus_throttle_mid]
						print("=================== Back Inside the lane ====================")

						goal_repose_ang = self.ConvertTo360Range(last_hdg - 180.0)

					else:

						if not (np.isinf(right_wall_ranges_min)):
							output_pid_lc = self.pid_lc(right_wall_ranges_min)

							if output_pid_lc < -0.1:
								sbus_steering = int(self.map_with_limit(output_pid_lc, -self.lc_out_range, 0.0, self.sbus_steering_max_LC, self.sbus_steering_max_DB))
							elif output_pid_lc > 0.1:
								sbus_steering = int(self.map_with_limit(output_pid_lc, 0.0, self.lc_out_range, self.sbus_steering_min_DB, self.sbus_steering_min_LC))
							else:
								sbus_steering = self.str_mid

							#sbus_steering = int(self.map_with_limit(output_pid_lc, -self.lc_out_range, self.lc_out_range, self.sbus_steering_max_LC, self.sbus_steering_min_LC))
							sbus_throttle = self.sbus_throttle_fwd_LC

							self.sbus_cmd.data = [sbus_steering, sbus_throttle]

						else:
							self.sbus_cmd.data = [prev_sbus_steering, prev_sbus_throttle]

					self.mode = "LaneChange"
					output_pid = output_pid_lc

				#######################################
				### Step 3 Repose once changed lane ###
				#######################################
				elif repose_flag:

					diff_ang, sign = self.find_smallest_diff_ang(goal_repose_ang, self.hdg)
					goal_ang = goal_repose_ang

					if abs(diff_ang) < 1.0:
						self.pid_wf.auto_mode = True
						self.pid_lc.auto_mode = False
						self.pid_rh.auto_mode = False
						self.pid_ut.auto_mode = False

						repose_flag = False
						inside_lane_flag = True
						done_repose_flag = True

						self.video_trig_flag = True
						self.video_trig_msg.data = self.video_trig_flag
						self.video_trig_pub.publish(self.video_trig_msg) 

						print("=================== Done Repose ====================")
						self.sbus_cmd.data = [self.sbus_steering_mid, self.sbus_throttle_mid]

					else:
						output_pid_rh = self.pid_rh(diff_ang)

						sbus_steering = int(self.map_with_limit(output_pid_rh, -self.rh_out_range, self.rh_out_range, self.sbus_steering_max_RH, self.sbus_steering_min_RH))
						sbus_throttle = self.sbus_throttle_mid
						self.mode = "Reheading"
						output_pid = output_pid_rh

						self.sbus_cmd.data = [sbus_steering, sbus_throttle]

				#####################
				### Step 5 U-Turn ###
				#####################
				elif turning_flag:

					diff_ang, sign = self.find_smallest_diff_ang(goal_uturn_ang, self.hdg)
					goal_ang = goal_uturn_ang

					if abs(diff_ang) < 1.0:
						self.pid_wf.auto_mode = True
						self.pid_lc.auto_mode = False
						self.pid_rh.auto_mode = False
						self.pid_ut.auto_mode = False

						turning_flag = False
						inside_lane_flag = True
						done_lane_change_flag = False
						done_repose_flag = False

						self.sbus_cmd.data = [self.sbus_steering_mid, self.sbus_throttle_mid]
						self.sbus_cmd_pub.publish(self.sbus_cmd)
						print("******************* Done U-Turn ********************")
						print("wait few seconds....")
						time.sleep(2)

						self.shelf_counter += 1
						self.video_trig_flag = True
						self.video_trig_msg.data = self.video_trig_flag
						self.video_trig_pub.publish(self.video_trig_msg) 

						

					else:
						output_pid_ut = self.pid_ut(diff_ang)

						# if output_pid_ut < 0.0:
						# 	sbus_steering = int(self.map_with_limit(output_pid_ut, -self.ut_out_range, 0.0, self.sbus_steering_max_UT, self.sbus_steering_max_DB))
						# elif output_pid_ut > 0.0:
						# 	sbus_steering = int(self.map_with_limit(output_pid_ut, 0.0, self.ut_out_range, self.sbus_steering_min_DB, self.sbus_steering_min_UT))
						# else:
						# 	sbus_steering = self.str_mid

						sbus_steering = int(self.map_with_limit(output_pid_ut, -self.ut_out_range, self.ut_out_range, self.sbus_steering_max_UT, self.sbus_steering_min_UT))
						sbus_throttle = self.sbus_throttle_mid
						self.mode = "Uturn"
						output_pid = output_pid_ut

						self.sbus_cmd.data = [sbus_steering, sbus_throttle]



				####################
				### Log info out ###
				####################
				if (self.mode == "LaneChange") or (self.mode == "WallFollow"):
					print("shelf_count: {:d} | mode: {:} | left_dist_per: {:.2f} | right_min: {:.2f} | out_pid: {:.2f} | str: {:d} | thr: {:d} | front_stop: {:} | vid_flag: {:}".format(\
						self.shelf_counter, self.mode, self.left_dist_per, right_wall_ranges_min, output_pid, sbus_steering, sbus_throttle, self.FRONT_STOP, self.video_trig_flag))
				elif (self.mode == "Reheading") or (self.mode == "Uturn"):
					print("shelf_count: {:d} | mode: {:} | out_pid: {:.2f} | str: {:d} | thr: {:d} | hdg: {:.2f} | last_hdg: {:.2f} | goal_ang: {:.2f} | diff_ang: {:.2f} | vid_flag: {:}".format(\
						self.shelf_counter, self.mode, output_pid, sbus_steering, sbus_throttle, self.hdg, last_hdg, goal_ang, diff_ang, self.video_trig_flag))

						

				prev_sbus_steering = sbus_steering
				prev_sbus_throttle = sbus_throttle



			# # If we don't get any scan data, just stop
			else:
				self.sbus_cmd.data = [self.sbus_steering_mid, self.sbus_throttle_mid]


			# prev_diff = diff
			self.left_wall_scan_pub.publish(self.left_wall_scan)
			self.right_wall_scan_pub.publish(self.right_wall_scan)
			# self.front_wall_scan_pub.publish(self.front_wall_scan)

			self.sbus_cmd_pub.publish(self.sbus_cmd)


			## Drawing Polygons ##
			# footprint
			self.pg.header.stamp = rospy.Time.now()
			self.pg.header.frame_id = "base_footprint"
			self.pg.polygon.points = [
								Point32(x=self.A[0], y=self.A[1]),
								Point32(x=self.B[0], y=self.B[1]),
								Point32(x=self.D[0], y=self.D[1]),
								Point32(x=self.C[0], y=self.C[1])]

			# front stop zone
			self.pg_front_stop.header.stamp = rospy.Time.now()
			self.pg_front_stop.header.frame_id = "base_footprint"
			self.pg_front_stop.polygon.points = [
								Point32(x=self.B[0], y=self.B[1]),
								Point32(x=self.T[0], y=self.T[1]),
								Point32(x=self.D[0], y=self.D[1]),
								Point32(x=self.B[0], y=self.B[1])]

			# back stop zone
			# self.pg_back_stop.header.stamp = rospy.Time.now()
			# self.pg_back_stop.header.frame_id = "base_footprint"
			# self.pg_back_stop.polygon.points = [
			# 						Point32(x=A[0], y=A[1]),
			# 						Point32(x=C[0], y=C[1]),
			# 						Point32(x=W[0], y=W[1]),
			# 						Point32(x=A[0], y=A[1])]

			# construct tf
			self.t.header.frame_id = "base_footprint" 
			self.t.header.stamp = rospy.Time.now()
			self.t.child_frame_id = "base_link"
			self.t.transform.translation.x = 0.0
			self.t.transform.translation.y = 0.0
			self.t.transform.translation.z = 0.0

			self.t.transform.rotation.x = 0.0
			self.t.transform.rotation.y = 0.0
			self.t.transform.rotation.z = 0.0
			self.t.transform.rotation.w = 1.0
			self.br.sendTransform(self.t)

			# laser_pub.publish(new_scan)
			self.foot_pub.publish(self.pg)
			self.front_stop_pub.publish(self.pg_front_stop)
			# self.back_stop_pub.publish(self.pg_back_stop)



			rate.sleep()




if __name__ == "__main__":

	parser = argparse.ArgumentParser(description='Greenhouse Navigation node')
	parser.add_argument('--param_file',
						help="A file path of GreenhouseNav.yaml, default is the one in jmoab_autopilot_ros/cfg/")

	args = parser.parse_args(rospy.myargv()[1:])
	print(args.param_file)
	param_file = args.param_file

	if param_file is None:
		print("Use jmoab_autopilot_ros/cfg/GreenhouseNav.yaml")
		rospack = rospkg.RosPack()
		jmoab_autopilot_ros_path = rospack.get_path("jmoab_autopilot_ros")
		yaml_name = "GreenhouseNav.yaml"
		yaml_path = os.path.join(jmoab_autopilot_ros_path, "cfg", yaml_name)
	else:
		print("Use {:}".format(param_file))
		yaml_path = param_file

	ghn = GreenhouseNav(yaml_path)