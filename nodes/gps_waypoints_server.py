#!/usr/bin/env python

import rospy
import rospkg
import rosparam
from dynamic_reconfigure.server import Server
from jmoab_autopilot_ros.cfg import GpsWaypointsConfig
import os
import yaml

def callback(config, level):
	# rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},{str_param}, {bool_param}""".format(**config))
	
	# rospy.loginfo("""Reconfigure Request: {p}, {i}, {d}""".format(**config))
	# print(config)
	return config

if __name__ == "__main__":
	rospy.init_node("gps_waypoints_server_node", anonymous = False)
	rospy.loginfo("Start gps_waypoints_server_node")

	server = Server(GpsWaypointsConfig, callback)

	sv_node = "gps_waypoints_server_node"
	rospack = rospkg.RosPack()
	jmoab_autopilot_ros_path = rospack.get_path("jmoab_autopilot_ros")
	yaml_name = "GpsWaypoints.yaml"
	yaml_path = os.path.join(jmoab_autopilot_ros_path, "cfg", yaml_name)

	# https://answers.ros.org/question/169866/load-yaml-with-code/
	# load yaml file to rosparam server without running server on python
	f = open(yaml_path, 'r')
	yamlfile = yaml.load(f)
	rosparam.upload_params("/", yamlfile)

	## get parameter from rosparam server that we just loaded above
	# max_start_str = rosparam.get_param(sv_node+"/max_start_str")
	# min_start_str = rosparam.get_param(sv_node+"/min_start_str")
	# str_adj = rosparam.get_param(sv_node+"/str_adj")
	# skid_adj = rosparam.get_param(sv_node+"/skid_adj")
	# max_start_thr = rosparam.get_param(sv_node+"/max_start_thr")
	# thr_adj = rosparam.get_param(sv_node+"/thr_adj")
	# goal_dist_thresh = rosparam.get_param(sv_node+"/goal_dist_thresh")
	# goal_ang_thresh = rosparam.get_param(sv_node+"/goal_ang_thresh")
	# x_track_error_start = rosparam.get_param(sv_node+"/x_track_error_start")
	# pid_hdg_out_thresh = rosparam.get_param(sv_node+"/pid_hdg_out_thresh")
	# pid_x_out_thresh = rosparam.get_param(sv_node+"/pid_x_out_thresh")
	# hdg_p = rosparam.get_param(sv_node+"/hdg_p")
	# hdg_i = rosparam.get_param(sv_node+"/hdg_i")
	# hdg_d = rosparam.get_param(sv_node+"/hdg_d")
	# cross_p = rosparam.get_param(sv_node+"/cross_p")
	# cross_i = rosparam.get_param(sv_node+"/cross_i")
	# cross_d = rosparam.get_param(sv_node+"/cross_d")

	# server.update_configuration({"max_start_str":1094})
	rospy.spin()