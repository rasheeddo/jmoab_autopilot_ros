#!/usr/bin/env python

import rospy
import rospkg
import rosparam
from dynamic_reconfigure.server import Server
from jmoab_autopilot_ros.cfg import SwarmSimpleFollowConfig
import os
import yaml
import argparse
import sys

def callback(config, level):
	# rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},{str_param}, {bool_param}""".format(**config))
	
	# rospy.loginfo("""Reconfigure Request: {p}, {i}, {d}""".format(**config))
	print("Got new parameters")
	# print(config)
	return config

if __name__ == "__main__":

	rospy.init_node("swarm_simple_server_node", anonymous = False)
	rospy.loginfo("Start swarm_simple_server node")

	server = Server(SwarmSimpleFollowConfig, callback)
	sv_node = "swarm_simple_server_node"

	parser = argparse.ArgumentParser(description='Swarm simple follow server node')
	parser.add_argument('--param_file',
						help="A file path of SwarmSimpleFollow.yaml, default is the one in cfg/")

	#args = parser.parse_args()
	args = parser.parse_args(rospy.myargv()[1:])	# to make it work on launch file
	param_file = args.param_file

	if param_file is None:
		print("Use jmoab_autopilot_ros/cfg/SwarmSimpleFollow.yaml")
		rospack = rospkg.RosPack()
		jmoab_autopilot_ros_path = rospack.get_path("jmoab_autopilot_ros")
		yaml_name = "SwarmSimpleFollow.yaml"
		yaml_path = os.path.join(jmoab_autopilot_ros_path, "cfg", yaml_name)
	else:
		yaml_path = param_file
		
	
	# https://answers.ros.org/question/169866/load-yaml-with-code/
	# load yaml file to rosparam server without running server on python
	f = open(yaml_path, 'r')
	yamlfile = yaml.safe_load(f)
	rosparam.upload_params("/", yamlfile)

	# get parameter from rosparam server that we just loaded above
	max_start_str = rosparam.get_param(sv_node+"/max_start_str")
	min_start_str = rosparam.get_param(sv_node+"/min_start_str")
	str_adj = rosparam.get_param(sv_node+"/str_adj")
	skid_adj = rosparam.get_param(sv_node+"/skid_adj")

	thr_slowest = rosparam.get_param(sv_node+"/thr_slowest")

	goal_dist_thresh = rosparam.get_param(sv_node+"/goal_dist_thresh")
	goal_ang_thresh = rosparam.get_param(sv_node+"/goal_ang_thresh")

	hdg_p = rosparam.get_param(sv_node+"/hdg_p")
	hdg_i = rosparam.get_param(sv_node+"/hdg_i")
	hdg_d = rosparam.get_param(sv_node+"/hdg_d")

	vel_p = rosparam.get_param(sv_node+"/vel_p")
	vel_i = rosparam.get_param(sv_node+"/vel_i")
	vel_d = rosparam.get_param(sv_node+"/vel_d")

	rospy.loginfo("Load these parameters to server")
	rospy.loginfo("max_start_str : {:}".format(max_start_str))
	rospy.loginfo("min_start_str : {:}".format(min_start_str))
	rospy.loginfo("str_adj : {:}".format(str_adj))
	rospy.loginfo("skid_adj : {:}".format(skid_adj))
	rospy.loginfo("thr_slowest : {:}".format(thr_slowest))
	rospy.loginfo("goal_dist_thresh : {:}".format(goal_dist_thresh))
	rospy.loginfo("goal_ang_thresh : {:}".format(goal_ang_thresh))
	rospy.loginfo("hdg_p : {:}".format(hdg_p))
	rospy.loginfo("hdg_i : {:}".format(hdg_i))
	rospy.loginfo("hdg_d : {:}".format(hdg_d))
	rospy.loginfo("vel_p : {:}".format(vel_p))
	rospy.loginfo("vel_i : {:}".format(vel_i))
	rospy.loginfo("vel_d : {:}".format(vel_d))

	# server.update_configuration({"max_start_str":1094})
	rospy.spin()