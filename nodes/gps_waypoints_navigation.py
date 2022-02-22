#!/usr/bin/env python

import rospy
import rospkg
import rosparam
import yaml
import dynamic_reconfigure.client
import os
import argparse

import time
from std_msgs.msg import Float32MultiArray, Int32MultiArray, Bool, Int16, String, Int8, UInt16, UInt8
from jmoab_autopilot_ros.msg import GoalWaypoints
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
import numpy as np
from simple_pid import PID

class GPS_NAV(object):

	def __init__(self, param_file, mission_file, NS):

		################################ Init/Pub/Sub #####################################
		rospy.init_node("gps_waypoints_autopilot_node", anonymous=True)

		################################ ROS Parameters #####################################
		sv_node = "gps_waypoints_server_node"

		# https://answers.ros.org/question/169866/load-yaml-with-code/
		# load yaml file to rosparam server without running server on python
		f = open(param_file, 'r')
		yamlfile = yaml.load(f)
		rosparam.upload_params("/", yamlfile)

		## get parameter from rosparam server that we just loaded above
		self.max_start_str = rosparam.get_param(sv_node+"/max_start_str")
		self.min_start_str = rosparam.get_param(sv_node+"/min_start_str")
		self.str_adj = rosparam.get_param(sv_node+"/str_adj")
		self.skid_adj = rosparam.get_param(sv_node+"/skid_adj")
		self.str_mid = rosparam.get_param(sv_node+"/str_mid")
		self.max_start_thr = rosparam.get_param(sv_node+"/max_start_thr")
		self.thr_adj = rosparam.get_param(sv_node+"/thr_adj")
		self.thr_slowest = rosparam.get_param(sv_node+"/thr_slowest")
		self.thr_mid = rosparam.get_param(sv_node+"/thr_mid")
		self.goal_dist_thresh = rosparam.get_param(sv_node+"/goal_dist_thresh")
		self.goal_ang_thresh = rosparam.get_param(sv_node+"/goal_ang_thresh")
		self.x_track_error_start = rosparam.get_param(sv_node+"/x_track_error_start")
		self.x_track_repose_dist = rosparam.get_param(sv_node+"/x_track_repose_dist")
		self.pid_hdg_out_thresh = rosparam.get_param(sv_node+"/pid_hdg_out_thresh")
		self.pid_x_out_thresh = rosparam.get_param(sv_node+"/pid_x_out_thresh")
		self.hdg_p = rosparam.get_param(sv_node+"/hdg_p")
		self.hdg_i = rosparam.get_param(sv_node+"/hdg_i")
		self.hdg_d = rosparam.get_param(sv_node+"/hdg_d")
		self.cross_p = rosparam.get_param(sv_node+"/cross_p")
		self.cross_i = rosparam.get_param(sv_node+"/cross_i")
		self.cross_d = rosparam.get_param(sv_node+"/cross_d")
		self.vel_p = rosparam.get_param(sv_node+"/vel_p")
		self.vel_i = rosparam.get_param(sv_node+"/vel_i")
		self.vel_d = rosparam.get_param(sv_node+"/vel_d")
		self.use_heartbeat = rosparam.get_param(sv_node+"/use_heartbeat")

		########################### GPS Parameters ##########################  
		self.lat = 0.0
		self.lon = 0.0
		self.hdg = 0.0

		self.lat1 = 0.0
		self.lon1 = 0.0

		self.lat2 = 0.0
		self.lon2 = 0.0

		self.gps_abs_vel = 0.0

		self.use_gps1 = False
		self.use_gps2 = False

		mission_name = "mission.txt"
		# self.mission_path = os.path.join(jmoab_autopilot_ros_path, "waypoints", mission_name)
		self.mission_path = mission_file

		self.lat_target_list = []
		self.lon_target_list = []
		self.speed_target_list = []
		self.delay_target_list = []
		self.sbus_throttle_target_list = []
		self.relay_target_list = []
		self.total_points = 0
		self.target_wp = 0

		self.got_new_wps = False

		self.hb_timestamp = time.time()
		self.hb_timeout = 10.0
		self.hb_period = 0.0

		########################### PID Parameters ##########################
		## PID heading steering control
		self.setpoint_diff_hdg = 0.0

		self.max_err = 100.0	#10.0	#2.0

		self.pid_hdg = PID(self.hdg_p, self.hdg_i, self.hdg_d, setpoint=self.setpoint_diff_hdg)
		self.pid_hdg.tunings = (self.hdg_p, self.hdg_i, self.hdg_d)
		self.pid_hdg.sample_time = 0.001
		self.pid_hdg.output_limits = (-self.max_err, self.max_err)

		## PID cross track error steering control
		self.setpoint_x = 0.0

		self.max_x_err = 100.0	#2.0

		self.pid_x = PID(self.cross_p, self.cross_i, self.cross_d, setpoint=self.setpoint_x)
		self.pid_x.tunings = (self.cross_p, self.cross_i, self.cross_d)
		self.pid_x.sample_time = 0.001
		self.pid_x.output_limits = (-self.max_x_err, self.max_x_err)

		self.pid_vel = PID(self.vel_p, self.vel_i, self.vel_d, setpoint=1.0)
		self.pid_vel.tunings = (self.vel_p, self.vel_i, self.vel_d)
		self.pid_vel.sample_time = 0.001
		self.max_vel = 3.0
		self.pid_vel.output_limits = (0.0, self.max_vel)

		self.enable_update = True
		################################ Dynamic Reconfigure #####################################
		print("Wait for server node...")
		try:
			## This we will get live data from rqt_reconfigure
			self.client = dynamic_reconfigure.client.Client(sv_node, timeout=2, config_callback=self.param_callback)
			print("Open rqt_reconfigure to tune paramters realtime")
		except rospy.ROSException:
			print("Server node is not alive, load parameters from GpsWaypoints.yaml file")
			pass

		########################### Robot Parameters ########################## 
		self.ROBOT_MODE = "MANUAL"
		#self.sbus_throttle_mid = 1024
		#self.sbus_steering_mid = 969	#1024
		self.cart_mode = 0
		self.completed_flag = False
		self.hdg_calib_flag = False

		self.sbus_max = 1680
		self.vel_min = 0.2
		self.vel_max = 2.2

		self.obst_flag = False

		self.update_local_param()

		########################### Pub/Sub Topics ##########################

		if NS is None:
			gps_topic = "/ublox/fix"
			gps2_topic = "/ublox/fix"
			compass_topic = "/jmoab_compass"
			atcart_mode_topic = "/atcart_mode"
			hb_topic = "/heartbeat"
			goal_wp_topic = "/goal_waypoints"
			rq_goal_wp_topic = "/request_store_waypoints"
			obs_detect_topic = "/obstacle_detection"
			vel_topic = "/ublox/fix_velocity"
			sbus_cmd_topic = "/sbus_cmd"
			rep_goal_wp_topic = "/reply_goal_waypoints"
			arr_wp_topic = "/arrived_waypoints"
			atcart_mode_cmd_topic = "/atcart_mode_cmd"
			relay_topic = "/jmoab_relay1"
			mis_com_topic = "/mission_completed"
		else:
			if NS.startswith("/"):
				gps_topic = NS + "/ublox/fix"
				gps2_topic = NS + "/ublox/fix"
				compass_topic = NS + "/jmoab_compass"
				atcart_mode_topic = NS + "/atcart_mode"
				hb_topic = NS + "/heartbeat"
				goal_wp_topic = NS + "/goal_waypoints"
				rq_goal_wp_topic = NS + "/request_store_waypoints"
				obs_detect_topic = NS + "/obstacle_detection"
				vel_topic = NS + "/ublox/fix_velocity"
				sbus_cmd_topic = NS + "/sbus_cmd"
				rep_goal_wp_topic = NS + "/reply_goal_waypoints"
				arr_wp_topic = NS + "/arrived_waypoints"
				atcart_mode_cmd_topic = NS + "/atcart_mode_cmd"
				relay_topic = NS + "/jmoab_relay1"
				mis_com_topic = NS + "/mission_completed"
			else:
				gps_topic = "/" + NS + "/ublox/fix"
				gps2_topic = "/" + NS + "/ublox/fix"
				compass_topic = "/" + NS + "/jmoab_compass"
				atcart_mode_topic = "/" + NS + "/atcart_mode"
				hb_topic = "/" + NS + "/heartbeat"
				goal_wp_topic = "/" + NS + "/goal_waypoints"
				rq_goal_wp_topic = "/" + NS + "/request_store_waypoints"
				obs_detect_topic = "/" + NS + "/obstacle_detection"
				vel_topic = "/" + NS + "/ublox/fix_velocity"
				sbus_cmd_topic = "/" + NS + "/sbus_cmd"
				rep_goal_wp_topic = "/" + NS + "/reply_goal_waypoints"
				arr_wp_topic = "/" + NS + "/arrived_waypoints"
				atcart_mode_cmd_topic = "/" + NS + "/atcart_mode_cmd"
				relay_topic = "/" + NS + "/jmoab_relay1"
				mis_com_topic = "/" + NS + "/mission_completed"

		rospy.Subscriber(gps_topic, NavSatFix, self.gps_callback)
		rospy.Subscriber(gps2_topic, NavSatFix, self.gps2_callback)
		rospy.Subscriber(compass_topic, Float32MultiArray, self.compass_callback)
		rospy.Subscriber(atcart_mode_topic, Int8, self.atcart_mode_callback)
		# rospy.Subscriber("/hdg_calib_flag", Bool, self.hdg_calib_flag_callback)
		rospy.Subscriber(hb_topic, Bool, self.heartbeat_callback)
		rospy.Subscriber(goal_wp_topic, GoalWaypoints, self.goal_wp_callback)
		rospy.Subscriber(rq_goal_wp_topic, Bool, self.request_wp_callback)
		rospy.Subscriber(obs_detect_topic, UInt8, self.obst_detect_callback)
		rospy.Subscriber(vel_topic, TwistWithCovarianceStamped, self.gps_vel_callback)

		self.sbus_cmd_pub = rospy.Publisher(sbus_cmd_topic, Int32MultiArray, queue_size=10)
		self.sbus_cmd = Int32MultiArray()
		self.reply_wp_pub = rospy.Publisher(rep_goal_wp_topic, GoalWaypoints, queue_size=10)
		self.reply_wp_msg = GoalWaypoints()
		self.arrived_wp_pub = rospy.Publisher(arr_wp_topic, UInt16, queue_size=10)
		self.arrived_wp_msg = UInt16()
		self.atcart_mode_cmd_pub = rospy.Publisher(atcart_mode_cmd_topic, Int8, queue_size=10)
		self.atcart_mode_cmd_msg = Int8()
		self.jmoab_relay1_pub = rospy.Publisher(relay_topic, Bool, queue_size=10)
		self.jmoab_relay1_msg = Bool()
		self.mission_completed_pub = rospy.Publisher(mis_com_topic, Bool, queue_size=10)
		self.mission_completed_msg = Bool()

		########################### Start ########################## 
		self.get_target_points()

		time.sleep(1)
		self.loop()
		rospy.spin()

	def get_target_points(self):

		## Reset all previos data
		self.lat_target_list = []
		self.lon_target_list = []
		self.speed_target_list = []
		self.delay_target_list = []
		self.sbus_throttle_target_list = []
		self.relay_target_list = []

		if os.path.exists(self.mission_path):
			print("Extract lat/lon from mission.txt")
			file = open(self.mission_path, 'r')
			lines = file.read().splitlines()

			for line in lines:
				if len(line)<40:
					pass
				else:
					line_list = line.split('\t')
					self.speed_target_list.append(float(line_list[4]))
					self.delay_target_list.append(float(line_list[5]))
					self.relay_target_list.append(int(float(line_list[6])))
					self.lat_target_list.append(float(line_list[8]))
					self.lon_target_list.append(float(line_list[9]))

			file.close()

			self.total_points = len(self.lat_target_list)

			for speed in self.speed_target_list:
				self.sbus_throttle_target_list.append(self.vel2sbus(speed))

			print("lat_target_list", self.lat_target_list)
			print("lon_target_list", self.lon_target_list)
			print("speed_target_list", self.speed_target_list)
			print("delay_target_list", self.delay_target_list)
			print("sbus_throttle_target_list", self.sbus_throttle_target_list)
			print("relay_target_list", self.relay_target_list)

	def generate_mission_file(self, current_wp, coord_frame, command, param1, param2, param3, param4, lat_list, lon_list, alt_list, autocontinue):
		
		mission_file = open(self.mission_path, 'w+')
		mission_file.write("QGC WPL 110\n")

		for i in range(len(lat_list)):
			mission_file.write('{:d}'.format(i))
			mission_file.write("\t")
			mission_file.write("{:d}".format(current_wp[i]))
			mission_file.write("\t")
			mission_file.write("{:d}".format(coord_frame[i]))
			mission_file.write("\t")
			mission_file.write("{:d}".format(command[i]))
			mission_file.write("\t")
			mission_file.write("{:.6f}".format(param1[i]))
			mission_file.write("\t")
			mission_file.write("{:.6f}".format(param2[i]))
			mission_file.write("\t")
			mission_file.write("{:.6f}".format(param3[i]))
			mission_file.write("\t")
			mission_file.write("{:.6f}".format(param4[i]))
			mission_file.write("\t")
			mission_file.write('{:.16f}'.format(lat_list[i]))
			mission_file.write("\t")
			mission_file.write('{:.16f}'.format(lon_list[i]))
			mission_file.write("\t")
			mission_file.write("{:.2f}".format(alt_list[i]))
			mission_file.write("\t")
			mission_file.write("{:d}".format(autocontinue[i]))
			if i != (len(lat_list)-1):
				mission_file.write("\n")

		mission_file.close()

	def heartbeat_callback(self, msg):

		if msg.data == True:

			self.hb_timestamp = time.time()

	def param_callback(self, config):

		print("Got new parameters")
		# rospy.loginfo("Config set to {int_param}, {double_param}, {str_param}, {bool_param}".format(**config))
		# rospy.loginfo("Config set to {p}, {i}, {d}".format(**config))
		self.max_start_str = config["max_start_str"]
		self.min_start_str = config["min_start_str"]
		self.str_adj = config["str_adj"]
		self.skid_adj = config["skid_adj"]
		self.str_mid = config["str_mid"]
		self.max_start_thr = config["max_start_thr"]
		self.thr_adj = config["thr_adj"]
		self.thr_slowest = config["thr_slowest"]
		self.thr_mid = config["thr_mid"]
		self.goal_dist_thresh = config["goal_dist_thresh"]
		self.goal_ang_thresh = config["goal_ang_thresh"]
		self.x_track_error_start = config["x_track_error_start"]
		self.x_track_repose_dist = config["x_track_repose_dist"]
		self.pid_hdg_out_thresh = config["pid_hdg_out_thresh"]
		self.pid_x_out_thresh = config["pid_x_out_thresh"]
		self.hdg_p = config["hdg_p"]
		self.hdg_i = config["hdg_i"]
		self.hdg_d = config["hdg_d"]
		self.cross_p = config["cross_p"]
		self.cross_i = config["cross_i"]
		self.cross_d = config["cross_d"]
		self.vel_p = config["vel_p"]
		self.vel_i = config["vel_i"]
		self.vel_d = config["vel_d"]
		self.use_heartbeat = config["use_heartbeat"]
		self.pid_hdg.tunings = (self.hdg_p, self.hdg_i, self.hdg_d)
		self.pid_x.tunings = (self.cross_p, self.cross_i, self.cross_d)
		self.pid_vel.tunings = (self.vel_p, self.vel_i, self.vel_d)

		self.update_local_param()

	def update_local_param(self):
		self.sbus_throttle_mid = self.thr_mid #1024
		self.sbus_steering_mid = self.str_mid #969
		# Throttle
		self.sbus_throttle_const = self.max_start_thr + self.thr_adj

		# PIVOT Turning
		self.sbus_skidding_right = self.max_start_str + self.skid_adj
		self.sbus_skidding_left = self.min_start_str - self.skid_adj

		# Cross track steering	
		self.sbus_steering_max = self.max_start_str + self.str_adj  # steer to right
		self.sbus_steering_min = self.min_start_str - self.str_adj  # steer to left

	def goal_wp_callback(self, msg):
		print("New goal_waypoints come")
		self.total_wps = len(msg.lat.data)
		current_wp_list = [0]*self.total_wps
		coord_frame_list = [3]*self.total_wps
		command_list = [16]*self.total_wps
		param1_list = msg.speed.data	# speed list
		param2_list = msg.delay.data	# delay list
		param3_list = msg.relay.data # relay list
		param4_list = [0]*self.total_wps
		lat_list = msg.lat.data
		lon_list = msg.lon.data
		alt_list = [10]*self.total_wps
		autocon_list = [1]*self.total_wps

		self.got_new_wps = True

		self.generate_mission_file(current_wp_list, coord_frame_list, command_list, \
			param1_list, param2_list, param3_list, param4_list, \
			lat_list, lon_list, alt_list, autocon_list)

		self.reply_wp_pub.publish(msg)

	def request_wp_callback(self,msg):

		print("Request waypoints")
		if msg.data == True:
			self.reply_wp_msg.lat.data = self.lat_target_list
			self.reply_wp_msg.lon.data = self.lon_target_list
			self.reply_wp_msg.speed.data = self.speed_target_list
			self.reply_wp_msg.delay.data = self.delay_target_list
			self.reply_wp_msg.relay.data = self.relay_target_list

			self.reply_wp_pub.publish(self.reply_wp_msg)

	def gps_callback(self, msg):

		self.lat1 = msg.latitude
		self.lon1 = msg.longitude

		self.use_gps1 = True

	def gps2_callback(self, msg):

		self.lat2 = msg.latitude
		self.lon2 = msg.longitude

		self.use_gps2 = True

	def compass_callback(self, msg):

		self.hdg = msg.data[2]

	def atcart_mode_callback(self, msg):
		self.cart_mode = msg.data

		# if self.cart_mode != 2:
		# 	self.completed_flag = False
	# def hdg_calib_flag_callback(self, msg):
	# 	self.hdg_calib_flag = msg.data

	# 	if msg.data == True:
	# 		print("=============== got calib flag =================")

	def gps_vel_callback(self, msg):
		vel_x = msg.twist.twist.linear.x
		vel_y = msg.twist.twist.linear.y
		self.gps_abs_vel = np.sqrt(vel_x**2 + vel_y**2)

	def obst_detect_callback(self, msg):
		if msg.data == 2:
			self.obst_flag = True
			# print("WARNING: Obstacle on front or back!!")
		else:
			self.obst_flag = False

	def vel2sbus(self, vel):

		if vel < 0.05:
			sbus = 1024
		elif 0.05 <= vel < self.vel_min:
			sbus = self.thr_slowest
		elif (self.vel_min <= vel <= self.vel_max):
			sbus = self.map_with_limit(vel, self.vel_min, self.vel_max, self.thr_slowest, self.sbus_max)
		elif vel > self.vel_max:
			sbus = self.sbus_max

		return int(sbus)

	def get_distance(self, lat1, lon1, lat2, lon2):

		R = 6371.0*1000.0
		lat_start = np.radians(lat1)
		lon_start = np.radians(lon1)
		lat_end = np.radians(lat2)
		lon_end = np.radians(lon2)
		dLat = lat_end - lat_start
		dLon = lon_end - lon_start

		a = np.sin(dLat/2.0)*np.sin(dLat/2.0) + np.cos(lat_start)*np.cos(lat_end)*np.sin(dLon/2.0)*np.sin(dLon/2.0)
		c = 2.0*np.arctan2(np.sqrt(a),np.sqrt(1-a))

		d = c*R

		return d

	def get_bearing(self, lat1, lon1, lat2, lon2):

		lat_start = np.radians(lat1)
		lon_start = np.radians(lon1)
		lat_end = np.radians(lat2)
		lon_end = np.radians(lon2)
		dLat = lat_end - lat_start
		dLon = lon_end - lon_start

		y = np.sin(dLon)*np.cos(lat_end)
		x = np.cos(lat_start)*np.sin(lat_end) - np.sin(lat_start)*np.cos(lat_end)*np.cos(dLon)
		bearing = np.degrees(np.arctan2(y,x))

		return bearing

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

	def control_relay(self, flag):

		for i in range(10):
			self.jmoab_relay1_msg.data = flag
			self.jmoab_relay1_pub.publish(self.jmoab_relay1_msg)

	def mission_is_completed(self, completed_flag):
		self.mission_completed_msg.data = completed_flag
		self.mission_completed_pub.publish(self.mission_completed_msg)

	def loop(self):

		rate = rospy.Rate(30)
		print("Start GPS Waypoints Navigation")


		## it must enter get_goal_ang_once first
		get_goal_ang_once = True
		turning_flag = False
		get_goal_dist_once = False
		throttle_flag = False
		end_flag = False
		cur_dist = 0.0
		goal_dist = 0.0

		## for skid to DIR heading
		get_goal_turn_ang_once = True
		skidding_flag = False
		reset_flag = False
		hdg_diff = 0.0

		from_hdg_diff = True
		from_x_track = True

		## for checking cart mode
		cart_mode_check_lock = False
		from_step = 0
		pause_flag = False
		from_obst_flag_detect = False
		## for heartbeat
		hb_trig_once = True

		while not rospy.is_shutdown():

			## average front/back gps to get somewhere in middle of the cart
			if self.use_gps1 and self.use_gps2:
				self.lat = (self.lat1 + self.lat2)/2.0
				self.lon = (self.lon1 + self.lon2)/2.0
			else:
				self.lat = self.lat1
				self.lon = self.lon1
			
			# if (not self.hdg_calib_flag):
			###########################################
			### Robot will start once got auto_mode ###
			###########################################
			if (self.cart_mode == 2) and (not self.completed_flag) and (not self.obst_flag):

				######################################################
				### In case of pause, these condition handle flags ###
				######################################################
				if (from_step == 1) and cart_mode_check_lock:
					get_goal_ang_once = True
					cart_mode_check_lock = False
					print("Resume step1")
				elif (from_step == 2) and cart_mode_check_lock:
					turning_flag = True
					cart_mode_check_lock = False
					print("Resume step2")
				elif (from_step == 3) and cart_mode_check_lock:
					get_goal_dist_once = True
					cart_mode_check_lock = False
					print("Resume step3")
				elif (from_step == 4) and cart_mode_check_lock:
					throttle_flag = True
					self.pid_hdg.auto_mode = True
					self.pid_x.auto_mode = True
					cart_mode_check_lock = False
					self.pid_vel.auto_mode = True
					print("Resume step4")
				elif (from_step == 5) and cart_mode_check_lock:
					end_flag = True
					cart_mode_check_lock = False
					print("Resume step5")
				elif from_obst_flag_detect:
					self.control_relay(self.relay_target_list[self.target_wp])
					self.pid_vel.auto_mode = True
					from_obst_flag_detect = False

				#######################################
				##### Step 1 Get how much to turn #####
				##### only one time               #####
				#######################################
				if get_goal_ang_once:
					goal_ang = self.get_bearing(self.lat, self.lon, self.lat_target_list[self.target_wp], self.lon_target_list[self.target_wp]) 
					## goal_ang is in 180ranges, hdg is in 360ranges
					diff_ang, sign = self.find_smallest_diff_ang(goal_ang, self.hdg)
					goal_hdg = self.hdg + (diff_ang*sign)

					get_goal_ang_once = False
					turning_flag = True
					end_flag = False
					print("step: 1 | goal_ang: {:.2f} | diff_ang: {:.2f} | sign: {:.1f} | hdg: {:.2f}".format(goal_ang, diff_ang, sign, self.hdg))
					step = 1

					self.pid_vel.auto_mode = False
					self.pid_vel.setpoint = self.speed_target_list[self.target_wp]

					self.control_relay(self.relay_target_list[self.target_wp])

				##########################################################
				##### Step 2 turning until less than angle threshold #####
				##### continuously                                   #####
				##########################################################
				if goal_hdg < 0.0:
					self.hdg = self.ConvertTo180Range(self.hdg)
				if goal_hdg > 360.0:
					goal_hdg = self.ConvertTo360Range(goal_hdg)


				if turning_flag and (abs(goal_hdg - self.hdg) > self.goal_ang_thresh) and (get_goal_ang_once==False):
					if sign == 1.0:
						sbus_steering = self.sbus_skidding_right # turn right
						sbus_throttle = self.sbus_throttle_mid
					elif sign == -1.0:
						sbus_steering = self.sbus_skidding_left # turn left
						sbus_throttle = self.sbus_throttle_mid
					step = 2
					
				elif (throttle_flag == False) and (turning_flag == True):
					print("finish turning")
					turning_flag = False
					get_goal_dist_once = True
					sbus_steering = self.sbus_steering_mid
					sbus_throttle = self.sbus_throttle_mid
					
					
				##############################################
				##### Step 3 Get how much distance to go #####
				##### only one time                      #####
				##############################################
				if get_goal_dist_once:
					goal_dist = self.get_distance(self.lat, self.lon, self.lat_target_list[self.target_wp], self.lon_target_list[self.target_wp])
					start_lat = self.lat
					start_lon = self.lon
					cur_dist = 0.0
					get_goal_dist_once = False
					throttle_flag = True
					end_flag = False

					step = 3
					print("step: 3 | hdg: {:.2f}".format(self.hdg))
					reset_tic = time.time()

					self.pid_vel.auto_mode = True
					

				#############################################################
				##### Step 4 go straight until less than dist threshold #####
				##### continuously                                      #####
				#############################################################
				if throttle_flag and (abs(goal_dist - cur_dist) > self.goal_dist_thresh) :

					## A is point of previous point 
					## in case of just start there is no -1 element in target_list so we use previous lat/lon
					if (self.target_wp == 0):
						A_lat = start_lat
						A_lon = start_lon
					else:
						A_lat = self.lat_target_list[self.target_wp-1]
						A_lon = self.lon_target_list[self.target_wp-1]

					## reset_flag tells if the cart went out of the route, so it will re-target heading with current pose 
					## if no reset_flag, it will just calculate angle to goal/bot with self.target_wp-1 and self.target_wp
					if reset_flag == True:
						ang_A_to_goal = self.get_bearing(start_lat, start_lon, self.lat_target_list[self.target_wp], self.lon_target_list[self.target_wp])
						ang_A_to_bot = self.get_bearing(start_lat, start_lon, self.lat, self.lon)
					else:
						ang_A_to_goal = self.get_bearing(A_lat, A_lon, self.lat_target_list[self.target_wp], self.lon_target_list[self.target_wp])
						ang_A_to_bot = self.get_bearing(A_lat, A_lon, self.lat, self.lon)


					if (ang_A_to_goal > 90.0) or (ang_A_to_goal < -90.0):
						ang_A_to_bot = self.ConvertTo360Range(ang_A_to_bot)
						ang_A_to_goal = self.ConvertTo360Range(ang_A_to_goal)
						

					#########################################################################################
					############################### cross-track-error calculation ###########################
					#########################################################################################

					##############################################################################################################################
					## "a" is a distance from target_wp-1 to bot, but if there is reset_flag, we use current pose of bot instead of target_wp-1 ##
					## "c" is a distance from target_wp-1 to target_wp                                                                          ##
					## "b" is a distance from bot to target_wp                                                                                  ##
					## "k" is a distance projection of bot's pose back to route plane                                                           ##
					## "x" is a distance from bot to route (cross-track-distance)                                                               ##
					##############################################################################################################################
					if reset_flag == True:
						# previous point and current pose
						a = self.get_distance(start_lat, start_lon, self.lat, self.lon)
						# previous point and goal point
						c = self.get_distance(start_lat, start_lon, self.lat_target_list[self.target_wp], self.lon_target_list[self.target_wp])
					else:
						# previous point and current pose
						a = self.get_distance(A_lat, A_lon, self.lat, self.lon)
						# previous point and goal point
						c = self.get_distance(A_lat, A_lon, self.lat_target_list[self.target_wp], self.lon_target_list[self.target_wp])

					# current pose and goal point
					b = self.get_distance(self.lat, self.lon, self.lat_target_list[self.target_wp], self.lon_target_list[self.target_wp])
				
					k = (a**2 - b**2 + c**2)/(2*c)
					x = np.sqrt(a**2 - k**2)
					if np.isnan(x):
						x = 0.0
					# print("a: {:.2f} | b: {:.2f} | c: {:.2f} | k: {:.2f} | x: {:.2f}".format(a,b,c,k,x))

					##########################################################
					## To find the robot is on which side of the route      ##
					## both ang_A_to_goal/bot are in 180ranges,             ##
					## so if the path c is pointing to 3rd or 4th quadrant, ##
					## then convert ang_A_to_goal/bot to 360ranges          ##
					##########################################################
					if ang_A_to_bot < ang_A_to_goal:
						x_side = -1.0	# on the left side of route
					else:
						x_side = 1.0	# on the right side of route

					x_track_error = x*x_side

					#############################################################################################
					############################### bearing hdg compare calculation #############################
					#############################################################################################

					#########################
					## use cross track PID ##
					#########################
					if (abs(x_track_error) > self.x_track_error_start):

						## Check one time that it's from hdg_diff PID, then enable x_tracK PID and disable hdg_diff PID
						if from_hdg_diff:
							self.pid_hdg.auto_mode = False
							self.pid_x.auto_mode = True
							from_x_track = True
							from_hdg_diff = False

						output_pid_x = self.pid_x(x_track_error)
						if x_track_error < -self.pid_x_out_thresh: #if output_pid_x > self.pid_x_out_thresh:
							sbus_steering = int(self.map_with_limit(output_pid_x, 0.0, self.max_x_err, self.max_start_str, self.sbus_steering_max))
						elif x_track_error > self.pid_x_out_thresh:	#elif output_pid_x < -self.pid_x_out_thresh:
							sbus_steering = int(self.map_with_limit(output_pid_x, -self.max_x_err, 0.0, self.sbus_steering_min, self.min_start_str))
						else:
							sbus_steering = self.sbus_steering_mid

						# sbus_throttle = self.sbus_throttle_const

						pid_mode = "x_error"
						OUTPUT_PID = output_pid_x

					#################
					## use hdg PID ##
					#################
					else:

						## Check one time that it's from x_tracK PID, then enable hdg_diff PID and disable x_tracK PID
						if from_x_track:
							self.pid_hdg.auto_mode = True
							self.pid_x.auto_mode = False
							from_x_track = False
							from_hdg_diff = True

						## hdg might got converted to 180ranges somewhere, but we need 360ranges to use with find_smallest_diff_ang function
						self.hdg = self.ConvertTo360Range(self.hdg)
						## using find_smallest_diff_ang function to compare between goal bearing (180ranges) and current heading (360ranges)
						hdg_diff, diff_sign = self.find_smallest_diff_ang(ang_A_to_goal, self.hdg)
						hdg_diff = self.ConvertTo180Range(hdg_diff*(-diff_sign))
						output_pid = self.pid_hdg(hdg_diff)

						if hdg_diff < -self.pid_hdg_out_thresh:	#if output_pid > self.pid_hdg_out_thresh:
							sbus_steering = int(self.map_with_limit(output_pid, 0.0, self.max_err, self.max_start_str, self.sbus_steering_max))
						elif hdg_diff > self.pid_hdg_out_thresh:	#elif output_pid < -self.pid_hdg_out_thresh:
							sbus_steering = int(self.map_with_limit(output_pid, -self.max_err, 0.0, self.sbus_steering_min, self.min_start_str))
						else:
							sbus_steering = self.sbus_steering_mid

						pid_mode = "hdg_diff"
						OUTPUT_PID = output_pid

					###########################################################################
					## if very close distance, use very low speed, ignore user speed command ##
					###########################################################################
					output_pid_vel = self.pid_vel(self.gps_abs_vel)
					if goal_dist < 2.0:
						sbus_throttle = self.thr_slowest+20
					else:
						if abs(goal_dist - cur_dist) < 2.0:
							sbus_throttle = self.thr_slowest+20
						else:
							if self.speed_target_list[self.target_wp] != 0.0:
								# sbus_throttle = self.sbus_throttle_target_list[self.target_wp] #self.sbus_throttle_const
								sbus_throttle = int(self.map_with_limit(output_pid_vel, 0, self.max_vel, 1024, 1680))
							else:
								sbus_throttle = self.sbus_throttle_const

		
							# print("output_pid_vel: {:.2f} | thr_pid: {:d}".format(output_pid_vel, sbus_throttle_pid))

					###############################################################
					### Resest target heading again if cross track error is big ###
					###############################################################
					reset_timeout = time.time() - reset_tic
					if (abs(x_track_error) > self.x_track_repose_dist) and (reset_timeout > 2.0):
						throttle_flag = False
						get_goal_ang_once = True
						reset_flag = True
						print("Reset target heading")
						sbus_steering = self.sbus_steering_mid
						sbus_throttle = self.sbus_throttle_mid
		
						self.pid_x.auto_mode = False
						self.from_hdg_diff = True
							
					####################################	
					### Print out all necessary info ###
					####################################
					print("PID: {:} | tg_wp: {:d} | x: {:.2f} | rst_T/O: {:.2f} | rst_flg: {:} | AngAtoBot: {:.2f} | hdg: {:.2f} | AngAtoGoal: {:.2f} | hdgDff: {:.2f} | outPID: {:.2f} | goalDist: {:.2f} curDist: {:.2f} | str: {:d} | thr: {:d} | vel: {:.2f} | vel_sp: {:.2f} | outPIDV: {:.2f}".format(\
						pid_mode, self.target_wp, x_track_error, reset_timeout, reset_flag, ang_A_to_bot, self.hdg, ang_A_to_goal, hdg_diff, OUTPUT_PID, goal_dist, cur_dist, sbus_steering, sbus_throttle, self.gps_abs_vel, self.speed_target_list[self.target_wp], output_pid_vel))


					## update cur_dist with current lat/lon compare to start lat/lon
					cur_dist = self.get_distance(start_lat, start_lon, self.lat, self.lon)
					step = 4

				elif (throttle_flag == True) and (turning_flag == False):
					throttle_flag = False
					get_goal_ang_once = True
					end_flag = True
					reset_flag = False
					print("finish throttle")
					sbus_steering = self.sbus_steering_mid
					sbus_throttle = self.sbus_throttle_mid

				#####################################################################################
				##### Step 5 check if turning and throttle are completed, then increment the wp #####
				#####################################################################################
				
				if (throttle_flag == False) and (turning_flag == False) and (end_flag == True):
					sbus_steering = self.sbus_steering_mid
					sbus_throttle = self.sbus_throttle_mid
					self.sbus_cmd.data = [int(sbus_steering), int(sbus_throttle)]
					self.sbus_cmd_pub.publish(self.sbus_cmd)
					print("Delay as {:.2f} seconds".format(self.delay_target_list[self.target_wp]))
					time.sleep(self.delay_target_list[self.target_wp])
					self.target_wp += 1
					self.pid_vel.auto_mode = False 

				######################################################################
				##### Step 6 check every time that target_wp is completed or not #####
				######################################################################
				if self.target_wp == len(self.lat_target_list):
					self.target_wp = 0
					# self.ROBOT_MODE = "MANUAL"
					self.completed_flag = True
					sbus_steering = self.sbus_steering_mid
					sbus_throttle = self.sbus_throttle_mid
					print("Mission completed")
					print("Switch from auto to manual, and back to auto again to restart mission.")

				self.sbus_cmd.data = [int(sbus_steering), int(sbus_throttle)]

				if (step != 4) and (not cart_mode_check_lock):
					print("step: {:d} | target_wp: {:d} | goal_hdg: {:.3f} | hdg: {:.3f} | goal_dist: {:.3f} | cur_dist: {:.3f}".format(\
						step, self.target_wp, goal_hdg, self.hdg, goal_dist, cur_dist))

				pause_flag = True

				self.mission_is_completed(False)

			#########################################################
			### This is pause on manual or hold mode by Tx switch ###
			#########################################################
			elif (self.cart_mode != 2) and (pause_flag):
				if get_goal_ang_once:
					get_goal_ang_once = False
					from_step = 1
					print("Stop at step1")
				elif turning_flag:
					turning_flag = False
					from_step = 2
					print("Stop at step2")
				elif get_goal_dist_once:
					get_goal_dist_once = False
					from_step = 3
					print("Stop at step3")
				elif throttle_flag:	
					throttle_flag = False
					from_step = 4
					self.pid_hdg.auto_mode = False
					self.pid_x.auto_mode = False
					self.pid_vel.auto_mode = False
					print("Stop at step4")
				elif end_flag:
					end_flag = False
					from_step = 5
					print("Stop at step5")

				cart_mode_check_lock = True

				if (self.got_new_wps):
					print("Reset lat/lon mission during mission")
					self.get_target_points()
					self.target_wp = 0
					self.got_new_wps = False

				self.mission_is_completed(False)


			#########################################
			### This is already completed mission ###
			#########################################
			elif (self.cart_mode == 2) and self.completed_flag:
				pause_flag = False
				self.mission_is_completed(True)
				pass

			########################################################################
			### Set back to manual or hold once, disable completed_flag to False ###
			########################################################################
			elif (self.cart_mode != 2) and self.completed_flag:
				self.completed_flag = False
				pause_flag = False

				if (self.got_new_wps):
					print("Completed previous mission, load new mission")
					self.get_target_points()
					self.target_wp = 0
					self.got_new_wps = False

				self.mission_is_completed(True)

			#########################################################################
			### Firstly start, maybe in manual/hold mode, completed_flag is False ###
			#########################################################################
			else:

				if (self.got_new_wps):
					print("Reset lat/lon mission before start")

					self.get_target_points()
					self.target_wp = 0
					self.got_new_wps = False

				if (self.obst_flag):
					## Trig only one time
					if from_obst_flag_detect == False:
						self.control_relay(False)
						self.pid_vel.auto_mode = False
						print("Detected obstacle....")
					from_obst_flag_detect = True

				self.sbus_cmd.data = [self.sbus_steering_mid, self.sbus_throttle_mid]

				self.mission_is_completed(False)


			self.sbus_cmd_pub.publish(self.sbus_cmd)

			##########################
			### heartbeat checking ###
			##########################
			self.hb_period = (time.time() - self.hb_timestamp)
			if self.use_heartbeat and (self.hb_period > self.hb_timeout):
				if (int(self.hb_period)%self.hb_timeout == 0) or hb_trig_once:
					print("Hearbeat timeout.. switch cart mode to hold")
					self.atcart_mode_cmd_msg.data = 0
					self.atcart_mode_cmd_pub.publish(self.atcart_mode_cmd_msg)
					hb_trig_once = False

			#############################################################
			### continuously send arrived wp                          ###
			### target_wp starts from 0, so we can send this directly ###
			#############################################################
			self.arrived_wp_msg.data = self.target_wp
			self.arrived_wp_pub.publish(self.arrived_wp_msg)

			rate.sleep()

if __name__ == "__main__":

	parser = argparse.ArgumentParser(description='GPS Waypoints Server node')
	parser.add_argument('--param_file',
						help="A file path of GpsWaypoints.yaml, default is the one in jmoab_autopilot_ros/cfg/")
	parser.add_argument('--mission_file', 
						help="A file path of mission.txt, default is the one in jmoab_autopilot_ros/waypoints/")
	parser.add_argument('--ns',
						help="a namespace in front of topics")

	args = parser.parse_args(rospy.myargv()[1:])
	print(args.param_file)
	param_file = args.param_file
	mission_file = args.mission_file
	ns = args.ns

	if param_file is None:
		print("Use jmoab_autopilot_ros/cfg/GpsWaypoints.yaml")
		rospack = rospkg.RosPack()
		jmoab_autopilot_ros_path = rospack.get_path("jmoab_autopilot_ros")
		yaml_name = "GpsWaypoints.yaml"
		yaml_path = os.path.join(jmoab_autopilot_ros_path, "cfg", yaml_name)
	else:
		yaml_path = param_file


	if mission_file is None:
		print("Use jmoab_autopilot_ros/waypoints/mission.txt")
		rospack = rospkg.RosPack()
		jmoab_autopilot_ros_path = rospack.get_path("jmoab_autopilot_ros")
		mission_name = "mission.txt"
		mission_path = os.path.join(jmoab_autopilot_ros_path, "waypoints", mission_name)
	else:
		mission_path = mission_file

	if ns is not None:
		print("Use namespace as {:}".format(ns))
	else:
		print("No namespace, using default")

	
	a = GPS_NAV(yaml_path, mission_path, ns)