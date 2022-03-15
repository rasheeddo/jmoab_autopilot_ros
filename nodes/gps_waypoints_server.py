#!/usr/bin/env python

import rospy
import rospkg
import rosparam
from dynamic_reconfigure.server import Server
from jmoab_autopilot_ros.cfg import GpsWaypointsConfig
import os
import yaml
import argparse
import sys
import time

def callback(config, level):
	# rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},{str_param}, {bool_param}""".format(**config))
	
	# rospy.loginfo("""Reconfigure Request: {p}, {i}, {d}""".format(**config))
	print("Got new parameters")
	return config

# def get_args():
# 	""" Get arguments for individual tb3 deployment. """
# 	parser = argparse.ArgumentParser(
# 		description="TB3 Gesture Controller Launch File."
# 	)

# 	# Required arguments
# 	parser.add_argument("-n", "--number",
# 						action="store",
# 						type=int,
# 						required=False,
# 						help="Add TB3 node namespace number.",
# 						default=0)

# 	return parser.parse_args(sys.argv[4:])

if __name__ == "__main__":

	rospy.init_node("gps_waypoints_server_node", anonymous = False)
	rospy.loginfo("Start gps_waypoints_server_node")

	parser = argparse.ArgumentParser(description='GPS Waypoints Server node')
	parser.add_argument('--param_file',
						help="A file path of GpsWaypoints.yaml, default is the one in cfg/")
	parser.add_argument('--ns',
						help="namespace of the robot")
	parser.add_argument('--load_param_local',
						help='0 if want to load param from launch file, 1 if load param inside the script, default is 1')

	#args = parser.parse_args()
	args = parser.parse_args(rospy.myargv()[1:])	# to make it work on launch file
	param_file = args.param_file
	ns = args.ns
	load_param_local = args.load_param_local

	if param_file is None:
		print("Use jmoab_autopilot_ros/cfg/GpsWaypoints.yaml")
		rospack = rospkg.RosPack()
		jmoab_autopilot_ros_path = rospack.get_path("jmoab_autopilot_ros")
		yaml_name = "GpsWaypoints.yaml"
		yaml_path = os.path.join(jmoab_autopilot_ros_path, "cfg", yaml_name)
	else:
		yaml_path = param_file

	if ns is not None:
		print("Use namespace as {:}".format(ns))
	else:
		print("No namespace, using default")

	if (load_param_local is None) or (int(load_param_local) == 1):
		load_param = True
		rospy.loginfo("Loading param from python") 
	elif int(load_param_local) == 0:
		load_param = False
		rospy.loginfo("Not loading param from python") 
	else:
		print("Please provide only 0 or 1 on --load_param_local")
		quit()


	server = Server(GpsWaypointsConfig, callback)
	sv_node = "gps_waypoints_server_node"

	# if ns is None:
	# 	sv_node = "gps_waypoints_server_node"
	# else:
	# 	sv_node = ns + "/gps_waypoints_server_node"

	if load_param:
		# https://answers.ros.org/question/169866/load-yaml-with-code/
		# load yaml file to rosparam server without running server on python
		f = open(yaml_path, 'r')
		yamlfile = yaml.load(f)
		rosparam.upload_params("/", yamlfile)
	# time.sleep(2)

	# get parameter from rosparam server that we just loaded above
	max_start_str = rosparam.get_param(sv_node+"/max_start_str")
	min_start_str = rosparam.get_param(sv_node+"/min_start_str")
	str_adj = rosparam.get_param(sv_node+"/str_adj")
	skid_adj = rosparam.get_param(sv_node+"/skid_adj")
	str_mid = rosparam.get_param(sv_node+"/str_mid")

	max_start_thr = rosparam.get_param(sv_node+"/max_start_thr")
	thr_adj = rosparam.get_param(sv_node+"/thr_adj")
	thr_slowest = rosparam.get_param(sv_node+"/thr_slowest")
	thr_mid = rosparam.get_param(sv_node+"/thr_mid")
	
	goal_dist_thresh = rosparam.get_param(sv_node+"/goal_dist_thresh")
	goal_ang_thresh = rosparam.get_param(sv_node+"/goal_ang_thresh")
	x_track_error_start = rosparam.get_param(sv_node+"/x_track_error_start")
	x_track_repose_dist = rosparam.get_param(sv_node+"/x_track_repose_dist")

	pid_hdg_out_thresh = rosparam.get_param(sv_node+"/pid_hdg_out_thresh")
	pid_x_out_thresh = rosparam.get_param(sv_node+"/pid_x_out_thresh")

	hdg_p = rosparam.get_param(sv_node+"/hdg_p")
	hdg_i = rosparam.get_param(sv_node+"/hdg_i")
	hdg_d = rosparam.get_param(sv_node+"/hdg_d")
	cross_p = rosparam.get_param(sv_node+"/cross_p")
	cross_i = rosparam.get_param(sv_node+"/cross_i")
	cross_d = rosparam.get_param(sv_node+"/cross_d")
	vel_p = rosparam.get_param(sv_node+"/vel_p")
	vel_i = rosparam.get_param(sv_node+"/vel_i")
	vel_d = rosparam.get_param(sv_node+"/vel_d")

	use_heartbeat = rosparam.get_param(sv_node+"/use_heartbeat")

	rospy.loginfo("Load these parameters to server")
	rospy.loginfo("max_start_str : {:}".format(max_start_str))
	rospy.loginfo("min_start_str : {:}".format(min_start_str))
	rospy.loginfo("str_adj : {:}".format(str_adj))
	rospy.loginfo("skid_adj : {:}".format(skid_adj))
	rospy.loginfo("str_mid : {:}".format(str_mid))
	rospy.loginfo("max_start_thr : {:}".format(max_start_thr))
	rospy.loginfo("thr_adj : {:}".format(thr_adj))
	rospy.loginfo("thr_slowest : {:}".format(thr_slowest))
	rospy.loginfo("thr_mid : {:}".format(thr_mid))
	rospy.loginfo("goal_dist_thresh : {:}".format(goal_dist_thresh))
	rospy.loginfo("goal_ang_thresh : {:}".format(goal_ang_thresh))
	rospy.loginfo("x_track_error_start : {:}".format(x_track_error_start))
	rospy.loginfo("x_track_repose_dist : {:}".format(x_track_repose_dist))
	rospy.loginfo("pid_hdg_out_thresh : {:}".format(pid_hdg_out_thresh))
	rospy.loginfo("pid_x_out_thresh : {:}".format(pid_x_out_thresh))
	rospy.loginfo("hdg_p : {:}".format(hdg_p))
	rospy.loginfo("hdg_i : {:}".format(hdg_i))
	rospy.loginfo("hdg_d : {:}".format(hdg_d))
	rospy.loginfo("cross_p : {:}".format(cross_p))
	rospy.loginfo("cross_i : {:}".format(cross_i))
	rospy.loginfo("cross_d : {:}".format(cross_d))
	rospy.loginfo("vel_p : {:}".format(vel_p))
	rospy.loginfo("vel_i : {:}".format(vel_i))
	rospy.loginfo("vel_d : {:}".format(vel_d))
	rospy.loginfo("use_heartbeat : {:}".format(use_heartbeat))

	# server.update_configuration({"max_start_str":1094})
	rospy.spin()