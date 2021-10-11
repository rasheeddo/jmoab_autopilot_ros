#!/usr/bin/env python

import rospy
import rospkg
from dynamic_reconfigure.server import Server
from jmoab_autopilot_ros.cfg import GpsWaypointsConfig
import os

def callback(config, level):
	# rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},{str_param}, {bool_param}""".format(**config))
	
	# rospy.loginfo("""Reconfigure Request: {p}, {i}, {d}""".format(**config))
	# print(config)
	return config

if __name__ == "__main__":
	rospy.init_node("gps_waypoints_server_node", anonymous = False)

	server = Server(GpsWaypointsConfig, callback)

	# server.update_configuration({"max_start_str":1094})
	rospy.spin()