#!/usr/bin/env python

import rospy
import rospkg
import rosparam
from dynamic_reconfigure.server import Server
from jmoab_autopilot_ros.cfg import GreenhouseNavConfig
import os
import yaml
import argparse
import sys

def callback(config, level):
	# rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},{str_param}, {bool_param}""".format(**config))
	
	# rospy.loginfo("""Reconfigure Request: {p}, {i}, {d}""".format(**config))
	# print(config)
	return config


if __name__ == "__main__":

	rospy.init_node("greenhouse_nav_server_node", anonymous = False)
	rospy.loginfo("Start greenhouse_nav_server_node")

	server = Server(GreenhouseNavConfig, callback)
	sv_node = "greenhouse_nav_server_node"

	parser = argparse.ArgumentParser(description='Greenhouse Nav server node')
	parser.add_argument('--param_file',
						help="A file path of GreenhouseNav.yaml, default is the one in cfg/")

	#args = parser.parse_args()
	args = parser.parse_args(rospy.myargv()[1:])	# to make it work on launch file
	param_file = args.param_file

	if param_file is None:
		print("Use jmoab_autopilot_ros/cfg/GreenhouseNav.yaml")
		rospack = rospkg.RosPack()
		jmoab_autopilot_ros_path = rospack.get_path("jmoab_autopilot_ros")
		yaml_name = "GreenhouseNav.yaml"
		yaml_path = os.path.join(jmoab_autopilot_ros_path, "cfg", yaml_name)
	else:
		yaml_path = param_file
		
	
	# https://answers.ros.org/question/169866/load-yaml-with-code/
	# load yaml file to rosparam server without running server on python
	f = open(yaml_path, 'r')
	yamlfile = yaml.load(f)
	rosparam.upload_params("/", yamlfile)

	# get parameter from rosparam server that we just loaded above
	wall_scan_ang = rosparam.get_param(sv_node+"/wall_scan_ang")
	front_stop_dist = rosparam.get_param(sv_node+"/front_stop_dist")
	right_dist_in_lane = rosparam.get_param(sv_node+"/right_dist_in_lane")
	left_dist_in_lane = rosparam.get_param(sv_node+"/left_dist_in_lane")
	max_start_str = rosparam.get_param(sv_node+"/max_start_str")
	min_start_str = rosparam.get_param(sv_node+"/min_start_str")
	str_mid = rosparam.get_param(sv_node+"/str_mid")
	wf_str_adj = rosparam.get_param(sv_node+"/wf_str_adj")
	lc_str_adj = rosparam.get_param(sv_node+"/lc_str_adj")
	rh_str_adj = rosparam.get_param(sv_node+"/rh_str_adj")
	ut_str_adj = rosparam.get_param(sv_node+"/ut_str_adj")
	wf_thr = rosparam.get_param(sv_node+"/wf_thr")
	lc_thr = rosparam.get_param(sv_node+"/lc_thr")
	wf_setpoint = rosparam.get_param(sv_node+"/wf_setpoint")
	wf_p = rosparam.get_param(sv_node+"/wf_p")
	wf_i = rosparam.get_param(sv_node+"/wf_i")
	wf_d = rosparam.get_param(sv_node+"/wf_d")
	lc_setpoint = rosparam.get_param(sv_node+"/lc_setpoint")
	lc_p = rosparam.get_param(sv_node+"/lc_p")
	lc_i = rosparam.get_param(sv_node+"/lc_i")
	lc_d = rosparam.get_param(sv_node+"/lc_d")
	rh_p = rosparam.get_param(sv_node+"/rh_p")
	rh_i = rosparam.get_param(sv_node+"/rh_i")
	rh_d = rosparam.get_param(sv_node+"/rh_d")
	ut_p = rosparam.get_param(sv_node+"/ut_p")
	ut_i = rosparam.get_param(sv_node+"/ut_i")
	ut_d = rosparam.get_param(sv_node+"/ut_d")

	rospy.loginfo("Load these parameters to server")
	rospy.loginfo("wall_scan_ang : {:}".format(wall_scan_ang))
	rospy.loginfo("front_stop_dist : {:}".format(front_stop_dist))
	rospy.loginfo("right_dist_in_lane : {:}".format(right_dist_in_lane))
	rospy.loginfo("left_dist_in_lane : {:}".format(left_dist_in_lane))
	rospy.loginfo("max_start_str : {:}".format(max_start_str))
	rospy.loginfo("min_start_str : {:}".format(min_start_str))
	rospy.loginfo("str_mid : {:}".format(str_mid))
	rospy.loginfo("wf_str_adj : {:}".format(wf_str_adj))
	rospy.loginfo("lc_str_adj : {:}".format(lc_str_adj))
	rospy.loginfo("rh_str_adj : {:}".format(rh_str_adj))
	rospy.loginfo("ut_str_adj : {:}".format(ut_str_adj))
	rospy.loginfo("wf_thr : {:}".format(wf_thr))
	rospy.loginfo("lc_thr : {:}".format(lc_thr))
	rospy.loginfo("wf_setpoint : {:}".format(wf_setpoint))
	rospy.loginfo("wf_p : {:}".format(wf_p))
	rospy.loginfo("wf_i : {:}".format(wf_i))
	rospy.loginfo("wf_d : {:}".format(wf_d))
	rospy.loginfo("lc_setpoint : {:}".format(lc_setpoint))
	rospy.loginfo("lc_p : {:}".format(lc_p))
	rospy.loginfo("lc_i : {:}".format(lc_i))
	rospy.loginfo("lc_d : {:}".format(lc_d))
	rospy.loginfo("rh_p : {:}".format(rh_p))
	rospy.loginfo("rh_i : {:}".format(rh_i))
	rospy.loginfo("rh_d : {:}".format(rh_d))
	rospy.loginfo("ut_p : {:}".format(ut_p))
	rospy.loginfo("ut_i : {:}".format(ut_i))
	rospy.loginfo("ut_d : {:}".format(ut_d))

	# server.update_configuration({"max_start_str":1094})
	rospy.spin()