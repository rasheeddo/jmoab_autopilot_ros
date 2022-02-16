#! /usr/bin/env python

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

class SwarmSimpleFollower(object):

	def __init__(self, param_file, load_param, master_ns, follower_ns):

		################################ Init #####################################
		rospy.init_node("swarm_simple_follower_node", anonymous=True)

		################################ ROS Parameters #####################################
		sv_node = "swarm_simple_server_node"

		# If we run the script alone as tuning mode, we also load param
		# but if load_param is False, means we will load it with namespace on swarm_simple_follower.launch file
		if load_param:
			f = open(param_file, 'r')
			yamlfile = yaml.load(f)
			rosparam.upload_params("/", yamlfile)

		## get parameter from rosparam server that we just loaded above
		self.max_start_str = rosparam.get_param(sv_node+"/max_start_str")
		self.min_start_str = rosparam.get_param(sv_node+"/min_start_str")
		self.str_adj = rosparam.get_param(sv_node+"/str_adj")
		self.skid_adj = rosparam.get_param(sv_node+"/skid_adj")
		self.thr_slowest = rosparam.get_param(sv_node+"/thr_slowest")
		self.goal_dist_thresh = rosparam.get_param(sv_node+"/goal_dist_thresh")
		self.goal_ang_thresh = rosparam.get_param(sv_node+"/goal_ang_thresh")
		self.hdg_p = rosparam.get_param(sv_node+"/hdg_p")
		self.hdg_i = rosparam.get_param(sv_node+"/hdg_i")
		self.hdg_d = rosparam.get_param(sv_node+"/hdg_d")
		self.vel_p = rosparam.get_param(sv_node+"/vel_p")
		self.vel_i = rosparam.get_param(sv_node+"/vel_i")
		self.vel_d = rosparam.get_param(sv_node+"/vel_d")


		########################### PID Parameters ##########################
		self.max_err = 100.0

		self.pid_hdg = PID(self.hdg_p, self.hdg_i, self.hdg_d, setpoint=0.0)
		self.pid_hdg.tunings = (self.hdg_p, self.hdg_i, self.hdg_d)
		self.pid_hdg.sample_time = 0.001
		self.pid_hdg.output_limits = (-self.max_err, self.max_err)

		self.pid_vel = PID(self.vel_p, self.vel_i, self.vel_d, setpoint=1.0)
		self.pid_vel.tunings = (self.vel_p, self.vel_i, self.vel_d)
		self.pid_vel.sample_time = 0.001
		self.max_vel = 3.0
		self.pid_vel.output_limits = (0.0, self.max_vel)

		################################ Dynamic Reconfigure #####################################
		print("Wait for server node...")
		try:
			## This we will get live data from rqt_reconfigure
			self.client = dynamic_reconfigure.client.Client(sv_node, timeout=2, config_callback=self.param_callback)
			print("Open rqt_reconfigure to tune paramters realtime")
		except rospy.ROSException:
			print("Server node is not alive, load parameters from SwarmSimpleFollow.yaml file")
			pass

		########################### General Parameters ##########################
		self.sbus_throttle_mid = 1024
		self.sbus_steering_mid = 1024

		self.update_local_param()

		self.m_lat = 0.0
		self.m_lon = 0.0
		self.f_lat = 0.0
		self.f_lon = 0.0
		self.f_hdg = 0.0

		self.m_vel = 0.0
		self.f_vel = 0.0

		self.f_cart_mode = 1

		################################ Pub/Sub #####################################
		################
		### Follower ###
		################
		if follower_ns.startswith("/"):
			FL_NS_gps_topic = follower_ns + "/ublox/fix"
			FL_NS_ublox_fix_vel_topic = follower_ns + "/ublox/fix_velocity"
			FL_NS_sbus_cmd_topic = follower_ns + "/sbus_cmd"
			FL_NS_atcart_mode_topic = follower_ns + "/atcart_mode"
			FL_NS_jmoab_compass_topic = follower_ns + "/jmoab_compass"
		else:
			FL_NS_gps_topic = "/" + follower_ns + "/ublox/fix"
			FL_NS_ublox_fix_vel_topic = "/" + follower_ns + "/ublox/fix_velocity"
			FL_NS_sbus_cmd_topic = "/" + follower_ns + "/sbus_cmd"
			FL_NS_atcart_mode_topic = "/" + follower_ns + "/atcart_mode"
			FL_NS_jmoab_compass_topic = "/" + follower_ns + "/jmoab_compass"

		rospy.Subscriber(FL_NS_gps_topic, NavSatFix, self.follower_gps_callback)
		rospy.Subscriber(FL_NS_ublox_fix_vel_topic, TwistWithCovarianceStamped, self.follower_vel_callback)
		rospy.Subscriber(FL_NS_atcart_mode_topic, Int8, self.follower_atcart_mode_callback)
		rospy.Subscriber(FL_NS_jmoab_compass_topic, Float32MultiArray, self.follower_compass_callback)

		self.sbus_cmd_pub = rospy.Publisher(FL_NS_sbus_cmd_topic, Int32MultiArray, queue_size=10)
		self.sbus_cmd = Int32MultiArray()

		##############
		### Master ###
		##############
		if master_ns.startswith("/"):
			MT_NS_gps_topic = master_ns + "/ublox/fix"
			MT_NS_ublox_fix_vel_topic = master_ns + "/ublox/fix_velocity"
		else:
			MT_NS_gps_topic = "/" + master_ns + "/ublox/fix"
			MT_NS_ublox_fix_vel_topic = "/" + master_ns + "/ublox/fix_velocity"

		rospy.Subscriber(MT_NS_gps_topic, NavSatFix, self.master_gps_callback)
		rospy.Subscriber(MT_NS_ublox_fix_vel_topic, TwistWithCovarianceStamped, self.master_vel_callback)

		########################### Start ########################## 

		self.loop()
		rospy.spin()

	def param_callback(self, config):

		print("Got new parameters")

		self.max_start_str = config["max_start_str"]
		self.min_start_str = config["min_start_str"]
		self.str_adj = config["str_adj"]
		self.skid_adj = config["skid_adj"]
		self.thr_slowest = config["thr_slowest"]
		self.goal_dist_thresh = config["goal_dist_thresh"]
		self.goal_ang_thresh = config["goal_ang_thresh"]
		self.hdg_p = config["hdg_p"]
		self.hdg_i = config["hdg_i"]
		self.hdg_d = config["hdg_d"]
		self.vel_p = config["vel_p"]
		self.vel_i = config["vel_i"]
		self.vel_d = config["vel_d"]

		self.pid_hdg.tunings = (self.hdg_p, self.hdg_i, self.hdg_d)
		self.pid_vel.tunings = (self.vel_p, self.vel_i, self.vel_d)

		self.update_local_param()

	def update_local_param(self):

		# PIVOT Turning
		self.sbus_skidding_right = self.max_start_str + self.skid_adj
		self.sbus_skidding_left = self.min_start_str - self.skid_adj

		# Cross track steering	
		self.sbus_steering_max = self.max_start_str + self.str_adj  # steer to right
		self.sbus_steering_min = self.min_start_str - self.str_adj  # steer to left

	def master_gps_callback(self, msg):

		self.m_lat = msg.latitude
		self.m_lon = msg.longitude

	def follower_gps_callback(self, msg):

		self.f_lat = msg.latitude
		self.f_lon = msg.longitude

	def master_vel_callback(self, msg):
		vel_x = msg.twist.twist.linear.x
		vel_y = msg.twist.twist.linear.y
		self.m_vel = np.sqrt(vel_x**2 + vel_y**2)

	def follower_vel_callback(self, msg):
		vel_x = msg.twist.twist.linear.x
		vel_y = msg.twist.twist.linear.y
		self.f_vel = np.sqrt(vel_x**2 + vel_y**2)

	def follower_compass_callback(self, msg):

		self.f_hdg = msg.data[2]

	def follower_atcart_mode_callback(self, msg):
		self.f_cart_mode = msg.data

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

		rate = rospy.Rate(30)
		print("Start Swarm simple follower")

		from_skidding = True
		from_stay_under_zone = True

		goal_ang = 0.0
		goal_hdg = 0.0
		diff_ang = 0.0
		dist = 0.0
		output_pid_hdg = 0.0
		output_pid_vel = 0.0
		sbus_steering = 1024
		sbus_throttle = 1024
		vel_setpoint = 1.0

		while not rospy.is_shutdown():

			if (self.f_cart_mode == 2):


				goal_ang = self.get_bearing(self.f_lat, self.f_lon, self.m_lat, self.m_lon)
				diff_ang, sign = self.find_smallest_diff_ang(goal_ang, self.f_hdg)

				dist = self.get_distance(self.f_lat, self.f_lon, self.m_lat, self.m_lon)

				if from_skidding or from_stay_under_zone:
					self.pid_hdg.auto_mode = True
					self.pid_vel.auto_mode = True
					from_skidding = False
					from_stay_under_zone = False

				if dist > self.goal_dist_thresh:
					output_pid_hdg = self.pid_hdg(-diff_ang*sign)

					if output_pid_hdg > 1.0:
						sbus_steering = int(self.map_with_limit(output_pid_hdg, 0.0, self.max_err, self.max_start_str, self.sbus_steering_max))
						# print("steer right >>>>>")
					elif output_pid_hdg < -1.0:
						sbus_steering = int(self.map_with_limit(output_pid_hdg, -self.max_err, 0.0, self.sbus_steering_min, self.min_start_str))
						# print("<<<<< steer left")
					else:
						sbus_steering = self.sbus_steering_mid

					if dist < self.goal_dist_thresh*1.2:
						vel_setpoint = self.m_vel*0.75
					else:
						if self.m_vel > 0.5:
							vel_setpoint = self.m_vel*1.25
						else:
							vel_setpoint= 0.5

					self.pid_vel.setpoint = vel_setpoint

					output_pid_vel = self.pid_vel(self.f_vel)
					sbus_throttle = int(self.map_with_limit(output_pid_vel, 0, self.max_vel, 1024, 1680))
					# print(self.pid_vel.setpoint)
					# sbus_throttle = 1080

				else:

					sbus_steering = self.sbus_steering_mid
					sbus_throttle = self.sbus_throttle_mid

					self.pid_hdg.auto_mode = False
					self.pid_vel.auto_mode = False

					from_stay_under_zone = True

			else:

				sbus_steering = self.sbus_steering_mid
				sbus_throttle = self.sbus_throttle_mid

			#print("mode: {:d} | diff_ang: {:.2f} | dist: {:.2f} | out_pid_hdg: {:.2f} | output_pid_vel: {:.2f} | m_vel: {:.2f} | f_vel: {:.2f} | sp_vel: {:.2f}| str: {:d} | thr: {:d}".format(\
			#	self.f_cart_mode, diff_ang, dist, output_pid_hdg, output_pid_vel, self.m_vel, self.f_vel, vel_setpoint, sbus_steering, sbus_throttle))


			self.sbus_cmd.data = [int(sbus_steering), int(sbus_throttle)]
			self.sbus_cmd_pub.publish(self.sbus_cmd)

			rate.sleep()


if __name__ == "__main__":

	parser = argparse.ArgumentParser(description='Swarm simple follower')
	parser.add_argument('--param_file',
						help="A file path of SwarmSimpleFollow.yaml, default is the one in jmoab_autopilot_ros/cfg/")
	parser.add_argument('--load_param_local',
						help='0 if want to load param from launch file, 1 if load param inside the script, default is 1')
	parser.add_argument('--master_ns',
						help="a namespace of master robot")
	parser.add_argument('--follower_ns',
						help="a namespace of follower robot")

	#args = parser.parse_args()
	args = parser.parse_args(rospy.myargv()[1:])	# to make it work on launch file
	param_file = args.param_file
	load_param_local = args.load_param_local
	master_ns = args.master_ns
	follower_ns = args.follower_ns

	if param_file is None:
		print("Use jmoab_autopilot_ros/cfg/SwarmSimpleFollow.yaml")
		rospack = rospkg.RosPack()
		jmoab_autopilot_ros_path = rospack.get_path("jmoab_autopilot_ros")
		yaml_name = "SwarmSimpleFollow.yaml"
		yaml_path = os.path.join(jmoab_autopilot_ros_path, "cfg", yaml_name)
	else:
		yaml_path = param_file

	if master_ns is None:
		print("Please provide namespace of master robot")
		quit()

	if follower_ns is None:
		print("Please provide namespace of follower robot")
		quit()

	if (load_param_local is None) or (int(load_param_local) == 1):
		load_param = True
	elif int(load_param_local) == 0:
		load_param = False 
	else:
		print("Please provide only 0 or 1 on --load_param_local")
		quit()


	a = SwarmSimpleFollower(yaml_path, load_param, master_ns, follower_ns)