#!/usr/bin/env python

import rospy
import rospkg
import rosparam
from dynamic_reconfigure.server import Server
from jmoab_autopilot_ros.cfg import HeadingControlConfig
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

	rospy.init_node("heading_control_server_node", anonymous = False)
	rospy.loginfo("Start heading_control_server_node")

	server = Server(HeadingControlConfig, callback)
	sv_node = "heading_control_server_node"

	parser = argparse.ArgumentParser(description='Heading control server node')
	parser.add_argument('--param_file',
						help="A file path of HeadingControl.yaml, default is the one in cfg/")

	#args = parser.parse_args()
	args = parser.parse_args(rospy.myargv()[1:])	# to make it work on launch file
	param_file = args.param_file

	if param_file is None:
		print("Use jmoab_autopilot_ros/cfg/HeadingControl.yaml")
		rospack = rospkg.RosPack()
		jmoab_autopilot_ros_path = rospack.get_path("jmoab_autopilot_ros")
		yaml_name = "HeadingControl.yaml"
		yaml_path = os.path.join(jmoab_autopilot_ros_path, "cfg", yaml_name)
	else:
		yaml_path = param_file
		
	
	# https://answers.ros.org/question/169866/load-yaml-with-code/
	# load yaml file to rosparam server without running server on python
	f = open(yaml_path, 'r')
	yamlfile = yaml.safe_load(f)
	rosparam.upload_params("/", yamlfile)

	# get parameter from rosparam server that we just loaded above
	str_adj = rosparam.get_param(sv_node+"/str_adj")
	hdg_p = rosparam.get_param(sv_node+"/hdg_p")
	hdg_i = rosparam.get_param(sv_node+"/hdg_i")
	hdg_d = rosparam.get_param(sv_node+"/hdg_d")
	skid_adj = rosparam.get_param(sv_node+"/skid_adj")
	skid_p = rosparam.get_param(sv_node+"/skid_p")
	skid_i = rosparam.get_param(sv_node+"/skid_i")
	skid_d = rosparam.get_param(sv_node+"/skid_d")
	vel_setpoint = rosparam.get_param(sv_node+"/vel_setpoint")
	vel_p = rosparam.get_param(sv_node+"/vel_p")
	vel_i = rosparam.get_param(sv_node+"/vel_i")
	vel_d = rosparam.get_param(sv_node+"/vel_d")

	rospy.loginfo("Load these parameters to server")
	rospy.loginfo("str_adj : {:}".format(str_adj))
	rospy.loginfo("hdg_p : {:}".format(hdg_p))
	rospy.loginfo("hdg_i : {:}".format(hdg_i))
	rospy.loginfo("hdg_d : {:}".format(hdg_d))
	rospy.loginfo("skid_adj : {:}".format(skid_adj))
	rospy.loginfo("skid_p : {:}".format(skid_p))
	rospy.loginfo("skid_i : {:}".format(skid_i))
	rospy.loginfo("skid_d : {:}".format(skid_d))
	rospy.loginfo("vel_setpoint : {:}".format(vel_setpoint))
	rospy.loginfo("vel_p : {:}".format(vel_p))
	rospy.loginfo("vel_i : {:}".format(vel_i))
	rospy.loginfo("vel_d : {:}".format(vel_d))

	# server.update_configuration({"max_start_str":1094})
	rospy.spin()