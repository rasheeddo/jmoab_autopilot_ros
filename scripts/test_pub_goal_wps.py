#!/usr/bin/env python

import rospy
import os
import time
from jmoab_autopilot_ros.msg import GoalWaypoints


rospy.init_node("test_pub_node", anonymous=True)

wp_pub = rospy.Publisher("/goal_waypoints", GoalWaypoints, queue_size=10)
wp_msg = GoalWaypoints()

lat_list = [35.8413122410131137, 35.8413736652703747]
lon_list = [139.524293839931488, 139.524310603737831]
speed_list = [1.0, 1.5]
delay_list = [0.0, 0.0]


wp_msg.lat.data = lat_list
wp_msg.lon.data = lon_list
wp_msg.speed.data = speed_list
wp_msg.delay.data = delay_list


rate = rospy.Rate(20) # 10hz

while not rospy.is_shutdown():

	wp_pub.publish(wp_msg)
	print(wp_msg)

	rate.sleep()