#!/usr/bin/env python

import rospy
import rospkg
import rosparam
import yaml
import dynamic_reconfigure.client
import os

import time
from std_msgs.msg import Float32MultiArray, Int32MultiArray, Bool, Int16, String
from sensor_msgs.msg import NavSatFix
import numpy as np
from simple_pid import PID

class GPS_NAV:

	def __init__(self):

		################################ Init/Pub/Sub #####################################
		rospy.init_node("gps_waypoints_autopilot_node", anonymous=True)
		rospy.Subscriber("/ublox/fix", NavSatFix, self.gps_callback)
		rospy.Subscriber("/jmoab_compass", Float32MultiArray, self.compass_callback)
		rospy.Subscriber("/atcart_mode", Int32MultiArray, self.sbus_callback)

		self.sbus_cmd_pub = rospy.Publisher("/sbus_cmd", Int32MultiArray, queue_size=10)
		self.sbus_cmd = Int32MultiArray()

		################################ ROS Parameters #####################################
		sv_node = "gps_waypoints_server_node"

		rospack = rospkg.RosPack()
		jmoab_autopilot_ros_path = rospack.get_path("jmoab_autopilot_ros")
		yaml_name = "GpsWaypoints.yaml"
		yaml_path = os.path.join(jmoab_autopilot_ros_path, "cfg", yaml_name)

		print("Wait for server node...")
		try:
			## This we will get live data from rqt_reconfigure
			self.client = dynamic_reconfigure.client.Client(sv_node, timeout=2, config_callback=self.param_callback)
			print("Open rqt_reconfigure to tune paramters realtime")
		except rospy.ROSException:
			print("Server node is not alive, load parameters from GpsWaypoints.yaml file")
			pass

		# https://answers.ros.org/question/169866/load-yaml-with-code/
		# load yaml file to rosparam server without running server on python
		f = open(yaml_path, 'r')
		yamlfile = yaml.load(f)
		rosparam.upload_params("/", yamlfile)

		## get parameter from rosparam server that we just loaded above
		self.max_start_str = rosparam.get_param(sv_node+"/max_start_str")
		self.min_start_str = rosparam.get_param(sv_node+"/min_start_str")
		self.str_adj = rosparam.get_param(sv_node+"/str_adj")
		self.skid_adj = rosparam.get_param(sv_node+"/skid_adj")
		self.max_start_thr = rosparam.get_param(sv_node+"/max_start_thr")
		self.thr_adj = rosparam.get_param(sv_node+"/thr_adj")
		self.goal_dist_thresh = rosparam.get_param(sv_node+"/goal_dist_thresh")
		self.goal_ang_thresh = rosparam.get_param(sv_node+"/goal_ang_thresh")
		self.x_track_error_start = rosparam.get_param(sv_node+"/x_track_error_start")
		self.pid_hdg_out_thresh = rosparam.get_param(sv_node+"/pid_hdg_out_thresh")
		self.pid_x_out_thresh = rosparam.get_param(sv_node+"/pid_x_out_thresh")
		self.hdg_p = rosparam.get_param(sv_node+"/hdg_p")
		self.hdg_i = rosparam.get_param(sv_node+"/hdg_i")
		self.hdg_d = rosparam.get_param(sv_node+"/hdg_d")
		self.cross_p = rosparam.get_param(sv_node+"/cross_p")
		self.cross_i = rosparam.get_param(sv_node+"/cross_i")
		self.cross_d = rosparam.get_param(sv_node+"/cross_d")

		########################### Robot Parameters ########################## 
		self.sbus_throttle_mid = 1024
		self.sbus_steering_mid = 1024

		self.update_local_param()

		########################### GPS Parameters ##########################  
		self.lat = 0.0
		self.lon = 0.0
		self.hdg = 0.0

		mission_name = "mission.txt"
		self.mission_path = os.path.join(jmoab_autopilot_ros_path, "waypoints", mission_name)

		self.lat_target_list = []
		self.lon_target_list = []
		self.total_points = 0

		########################### PID Parameters ##########################
		## PID heading steering control
		# self.kp = 0.85		# 15.0
		# self.ki = 0.07
		# self.kd = 0.0
		self.setpoint_diff_hdg = 0.0

		self.max_err = 100.0	#10.0	#2.0

		self.pid_hdg = PID(self.hdg_p, self.hdg_i, self.hdg_d, setpoint=self.setpoint_diff_hdg)
		self.pid_hdg.tunings = (self.hdg_p, self.hdg_i, self.hdg_d)
		self.pid_hdg.sample_time = 0.001
		self.pid_hdg.output_limits = (-self.max_err, self.max_err)

		# self.hdg_diff_output_DB = 1.0

		## PID cross track error steering control
		# self.kp_x = 2.5		# 15.0
		# self.ki_x = 0.07
		# self.kd_x = 0.0
		self.setpoint_x = 0.0

		self.max_x_err = 100.0	#2.0

		self.pid_x = PID(self.cross_p, self.cross_i, self.cross_d, setpoint=self.setpoint_x)
		self.pid_x.tunings = (self.cross_p, self.cross_i, self.cross_d)
		self.pid_x.sample_time = 0.001
		self.pid_x.output_limits = (-self.max_x_err, self.max_x_err)

		# self.x_track_error_start = 0.3	#testcart 0.15		# min distance to start using x-track PID
		# self.x_track_output_DB = 0.1

		self.x_track_repose_dist = 0.6	#testcart 0.4

		self.get_target_points()

		self.loop()
		rospy.spin()

	def get_target_points(self):
		print("Extract lat/lon from mission.txt")
		file = open(self.mission_file_path, 'r')
		lines = file.read().splitlines()

		for line in lines:
			if len(line)<40:
				pass
			else:
				line_list = line.split('\t')
				self.lat_target_list.append(float(line_list[8]))
				self.lon_target_list.append(float(line_list[9]))

		self.total_points = len(self.lat_target_list)

		print("lat_target_list", self.lat_target_list)
		print("lon_target_list", self.lon_target_list)


	def param_callback(self, config):
		# rospy.loginfo("Config set to {int_param}, {double_param}, {str_param}, {bool_param}".format(**config))
		# rospy.loginfo("Config set to {p}, {i}, {d}".format(**config))
		self.max_start_str = config["max_start_str"]
		self.min_start_str = config["min_start_str"]
		self.str_adj = config["str_adj"]
		self.skid_adj = config["skid_adj"]
		self.max_start_thr = config["max_start_thr"]
		self.thr_adj = config["thr_adj"]
		self.goal_dist_thresh = config["goal_dist_thresh"]
		self.goal_ang_thresh = config["goal_ang_thresh"]
		self.x_track_error_start = config["x_track_error_start"]
		self.pid_hdg_out_thresh = config["pid_hdg_out_thresh"]
		self.pid_x_out_thresh = config["pid_x_out_thresh"]
		self.hdg_p = config["hdg_p"]
		self.hdg_i = config["hdg_i"]
		self.hdg_d = config["hdg_d"]self.sbus_steering_max
		self.cross_p = config["cross_p"]
		self.cross_i = config["cross_i"]
		self.cross_d = config["cross_d"]

		self.update_local_param()

	def update_local_param(self):
		# Throttle
		self.sbus_throttle_const = self.sbus_throttle_mid + self.thr_adj

		# PIVOT Turning
		self.sbus_skidding_right = self.max_start_str + self.skid_adj
		self.sbus_skidding_left = self.min_start_str - self.skid_adj

		# Cross track steering	
		self.sbus_steering_max = self.max_start_str + self.str_adj  # steer to right
		self.sbus_steering_min = self.min_start_str - self.str_adj  # steer to left

	def gps_callback(self, msg):

		self.lat = msg.latitude
		self.lon = msg.longitude

	def compass_callback(self, msg):

		self.hdg = msg.data[2]

	def sbus_callback(self, msg):
		if msg.data[8] > 1000:
			self.ROBOT_MODE = 'PATROL'

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

	def loop(self):

		rate = rospy.Rate(100)
		print("Start GPS Waypoints Navigation")

		## it must enter get_goal_ang_once first
		get_goal_ang_once = True
		turning_flag = False
		get_goal_dist_once = False
		throttle_flag = False
		cur_dist = 0.0
		goal_dist = 0.0

		## for skid to DIR heading
		get_goal_turn_ang_once = True
		skidding_flag = False
		reset_flag = False
		hdg_diff = 0.0

		from_hdg_diff = True
		from_x_track = True

		while not rospy.is_shutdown():
			if (self.MODE == "patrol") and (self.FWD_FLAG == True):

				#######################################
				##### Step 1 Get how much to turn #####
				##### only one time               #####
				#######################################
				if get_goal_ang_once:
					# goal_dist = self.get_distance(self.lat, self.lon, self.lat_target_list[target_wp], self.lon_target_list[target_wp])
					goal_ang = self.get_bearing(self.lat, self.lon, self.lat_target_list[target_wp], self.lon_target_list[target_wp]) 
					## goal_ang is in 180ranges, hdg is in 360ranges
					diff_ang, sign = self.find_smallest_diff_ang(goal_ang, self.hdg)
					goal_hdg = self.hdg + (diff_ang*sign)

					get_goal_ang_once = False
					turning_flag = True
					end_flag = False

					step = 1

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
					# print("step: 2")
				elif (throttle_flag == False) and (turning_flag == True):
					print("finish turning")
					turning_flag = False
					get_goal_dist_once = True
					sbus_steering = self.sbus_steering_mid
					sbus_throttle = self.sbus_throttle_mid
					# time.sleep(2)

				##############################################
				##### Step 3 Get how much distance to go #####
				##### only one time                      #####
				##############################################
				if get_goal_dist_once:
					goal_dist = self.get_distance(self.lat, self.lon, self.lat_target_list[target_wp], self.lon_target_list[target_wp])
					start_lat = self.lat
					start_lon = self.lon
					cur_dist = 0.0
					get_goal_dist_once = False
					throttle_flag = True
					end_flag = False
					# reset_flag = False
					step = 3
					print("step: 3 | hdg: {:.2f}".format(self.hdg))
					reset_tic = time.time()
					# time.sleep(1)
					

				#############################################################
				##### Step 4 go straight until less than dist threshold #####
				##### continuously                                      #####
				#############################################################
				if throttle_flag and (abs(goal_dist - cur_dist) > self.goal_dist_thresh) :
					# sbus_steering = self.sbus_steering_mid
					# sbus_throttle = self.sbus_throttle_const

					

					## A is point of previous point 
					## in case of just start there is no -1 element in target_list so we use previous lat/lon
					if (target_wp == 0):
						A_lat = start_lat
						A_lon = start_lon
					else:
						A_lat = self.lat_target_list[target_wp-1]
						A_lon = self.lon_target_list[target_wp-1]

					ang_A_to_bot = self.get_bearing(A_lat, A_lon, self.lat, self.lon)
					if reset_flag == True:
						ang_A_to_goal = self.get_bearing(start_lat, start_lon, self.lat_target_list[target_wp], self.lon_target_list[target_wp])
					else:
						ang_A_to_goal = self.get_bearing(A_lat, A_lon, self.lat_target_list[target_wp], self.lon_target_list[target_wp])

					if (ang_A_to_goal > 90.0) or (ang_A_to_goal < -90.0):
						ang_A_to_bot = self.ConvertTo360Range(ang_A_to_bot)
						ang_A_to_goal = self.ConvertTo360Range(ang_A_to_goal)
						

					#########################################################################################
					############################### cross-track-error calculation ###########################
					#########################################################################################
					# previous point and current pose
					a = self.get_distance(A_lat, A_lon, self.lat, self.lon)
					# current pose and goal point
					b = self.get_distance(self.lat, self.lon, self.lat_target_list[target_wp], self.lon_target_list[target_wp])
					# previous point and goal point
					c = self.get_distance(A_lat, A_lon, self.lat_target_list[target_wp], self.lon_target_list[target_wp])

					k = (a**2 - b**2 + c**2)/(2*c)
					x = np.sqrt(a**2 - k**2)
					if np.isnan(x):
						x = 0.0
					# print("a: {:.2f} | b: {:.2f} | c: {:.2f} | k: {:.2f} | x: {:.2f}".format(a,b,c,k,x))

					## To find the robot is on which side of the route
					## both ang_A_to_goal/bot are in 180ranges, 
					## so if the path c is pointing to 3rd or 4th quadrant, then convert ang_A_to_goal/bot to 360ranges
					

					if ang_A_to_bot < ang_A_to_goal:
						x_side = -1.0	# on the left side of route
					else:
						x_side = 1.0	# on the right side of route

					x_track_error = x*x_side

					#############################################################################################
					############################### bearing hdg compare calculation #############################
					#############################################################################################

					## use cross track PID
					if (abs(x_track_error) > self.x_track_error_start):

						## Check one time that it's from hdg_diff PID, then enable x_tracK PID and disable hdg_diff PID
						if from_hdg_diff:
							self.pid_hdg.auto_mode = False
							self.pid_x.auto_mode = True
							from_x_track = True
							from_hdg_diff = False

						output_pid_x = self.pid_x(x_track_error)
						if output_pid_x > self.pid_x_out_thresh:
							sbus_steering = int(self.map_with_limit(output_pid_x, 0.0, self.max_x_err, self.max_start_str, self.sbus_steering_max))
						elif output_pid_x < -self.pid_x_out_thresh:
							sbus_steering = int(self.map_with_limit(output_pid_x, -self.max_x_err, 0.0, self.sbus_steering_min, self.min_start_str))
						else:
							sbus_steering = 1024

						sbus_throttle = self.sbus_throttle_const

						pid_mode = "x_error"
						OUTPUT_PID = output_pid_x

					## use hdg PID
					else:

						## Check one time that it's from x_tracK PID, then enable hdg_diff PID and disable x_tracK PID
						if from_x_track:
							self.pid_hdg.auto_mode = True
							self.pid_x.auto_mode = False
							from_x_track = False
							from_hdg_diff = True

						## hsg might got converted to 180ranges somewhere, but we need 360ranges to use with find_smallest_diff_ang function
						self.hdg = self.ConvertTo360Range(self.hdg)
						## using find_smallest_diff_ang function to compare between goal bearing (180ranges) and current heading (360ranges)
						hdg_diff, diff_sign = self.find_smallest_diff_ang(ang_A_to_goal, self.hdg)
						hdg_diff = self.ConvertTo180Range(hdg_diff*(-diff_sign))
						output_pid = self.pid(hdg_diff)

						if output_pid > self.pid_hdg_out_thresh:
							sbus_steering = int(self.map_with_limit(output_pid, 0.0, self.max_err, self.max_start_str, self.sbus_steering_max))
						elif output_pid < -self.pid_hdg_out_thresh:
							sbus_steering = int(self.map_with_limit(output_pid, -self.max_err, 0.0, self.sbus_steering_min, self.min_start_str))
						else:
							sbus_steering = 1024

						sbus_throttle = self.sbus_throttle_const

						pid_mode = "hdg_diff"
						OUTPUT_PID = output_pid


					## Resest target heading again if cross track error is big
					reset_timeout = time.time() - reset_tic
					if (abs(x_track_error) > self.x_track_repose_dist) and (reset_timeout > 5.0):
						throttle_flag = False
						get_goal_ang_once = True
						reset_flag = True
						print("Reset target heading")
						sbus_steering = self.sbus_steering_mid
						sbus_throttle = self.sbus_throttle_mid


					print("PID: {:} | target_wp: {:d} | x: {:.2f} | rst_timeout: {:.2f} | rst_flag: {:} | hdg: {:.2f} | ang_A_to_goal: {:.2f} | hdg_diff: {:.2f} | output_pid: {:.2f} | str: {:d} | thr: {:d}".format(\
						pid_mode, target_wp, x_track_error, reset_timeout, reset_flag, self.hdg, ang_A_to_goal, hdg_diff, OUTPUT_PID, sbus_steering, sbus_throttle))
					

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
					# time.sleep(2)

				#####################################################################################
				##### Step 5 check if turning and throttle are completed, then increment the wp #####
				#####################################################################################
				
				if (throttle_flag == False) and (turning_flag == False) and (end_flag == True):
					sbus_steering = self.sbus_steering_mid
					sbus_throttle = self.sbus_throttle_mid
					# time.sleep(2)
					target_wp += 1

				######################################################################
				##### Step 6 check every time that target_wp is completed or not #####
				######################################################################
				if target_wp == len(self.lat_target_list):
					target_wp = 0
					self.ROBOT_MODE = "MANUAL"
					sbus_steering = self.sbus_steering_mid
					sbus_throttle = self.sbus_throttle_mid
					print("Mission completed")
					

				self.sbus_cmd.data = [int(sbus_steering), int(sbus_throttle)]

				if step != 4:
					print("step: {:d} | target_wp: {:d} | goal_hdg: {:.3f} | hdg: {:.3f} | goal_dist: {:.3f} | cur_dist: {:.3f}".format(\
						step, target_wp, goal_hdg, self.hdg, goal_dist, cur_dist))

			else:

				self.sbus_cmd.data = [self.sbus_steering_mid, self.sbus_throttle_mid]

			self.sbus_cmd_pub.publish(self.sbus_cmd)

			rate.sleep()

if __name__ == "__main__":
	
	a = GPS_NAV()