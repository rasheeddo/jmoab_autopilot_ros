#!/usr/bin/env python

import rospy
import rospkg
import rosparam
from dynamic_reconfigure.server import Server
from jmoab_autopilot_ros.cfg import LidarObstacleDetectionConfig
import os
import yaml
import argparse
import sys

def callback(config, level):
	# rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},{str_param}, {bool_param}""".format(**config))
	
	# rospy.loginfo("""Reconfigure Request: {p}, {i}, {d}""".format(**config))
	# print(config)
	print("Got parameters")
	print(config)
	return config


if __name__ == "__main__":

	rospy.init_node("lidar_obst_detect_server_node", anonymous = False)
	rospy.loginfo("Start lidar_obst_detect_server_node")

	server = Server(LidarObstacleDetectionConfig, callback)
	sv_node = "lidar_obst_detect_server_node"

	parser = argparse.ArgumentParser(description='Lidar obstacle detection server node')
	parser.add_argument('--param_file',
						help="A file path of LidarObstacleDetection.yaml, default is the one in cfg/")

	#args = parser.parse_args()
	args = parser.parse_args(rospy.myargv()[1:])	# to make it work on launch file
	param_file = args.param_file

	if param_file is None:
		print("Use jmoab_autopilot_ros/cfg/LidarObstacleDetection.yaml")
		rospack = rospkg.RosPack()
		jmoab_autopilot_ros_path = rospack.get_path("jmoab_autopilot_ros")
		yaml_name = "LidarObstacleDetection.yaml"
		yaml_path = os.path.join(jmoab_autopilot_ros_path, "cfg", yaml_name)
	else:
		yaml_path = param_file
		
	
	# https://answers.ros.org/question/169866/load-yaml-with-code/
	# load yaml file to rosparam server without running server on python
	f = open(yaml_path, 'r')
	yamlfile = yaml.load(f)
	rosparam.upload_params("/", yamlfile)

	# get parameter from rosparam server that we just loaded above
	enable_scan = rosparam.get_param(sv_node+"/enable_scan")
	front_scan_angle = rosparam.get_param(sv_node+"/front_scan_angle")
	front_scan_dist = rosparam.get_param(sv_node+"/front_scan_dist")
	back_scan_angle = rosparam.get_param(sv_node+"/back_scan_angle")
	back_scan_dist = rosparam.get_param(sv_node+"/back_scan_dist")

	rospy.loginfo("Load these parameters to server")
	rospy.loginfo("enable_scan : {:}".format(enable_scan))
	rospy.loginfo("front_scan_angle : {:}".format(front_scan_angle))
	rospy.loginfo("front_scan_dist : {:}".format(front_scan_dist))
	rospy.loginfo("back_scan_angle : {:}".format(back_scan_angle))
	rospy.loginfo("back_scan_dist : {:}".format(back_scan_dist))
	
	rospy.spin()