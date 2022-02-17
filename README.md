# JMOAB_Autopilot_ROS

This is an autonomous drive package for rover with jmoab-ros, please check [jmoab-ros repo here](https://github.com/rasheeddo/jmoab-ros).

This package is designed to work side by side with jmoab-ros package to provide a feature of autonomous drive in many applications.

It could run with the real actual robot with JMOAB and also in simulation world in Gazebo as Software In The Loop (SITL).

![](images/diagram.jpeg)

Currently, it supports an autonomous drive as following.

- [GPS waypoints navigation](https://github.com/rasheeddo/jmoab_autopilot_ros#gps-waypoints-navigation)

- [Greenhuse navigation](https://github.com/rasheeddo/jmoab_autopilot_ros#greenhouse-navigation)

- [Swarm control](https://github.com/rasheeddo/jmoab_autopilot_ros#swarm)

## GPS Waypoints Navigation

![](images/gps_navigation.png)

### Hardware

To run autonomous drive outdoor with GPS envioronment, you will need

- Ublox F9P GPS with a [setup here](https://github.com/rasheeddo/jmoab-ros#jmoab-with-f9p-gps)

- BNO055 9-axes orieintation sensor with a [setup here](https://github.com/rasheeddo/jmoab-ros#run-as-compass-mode)

and you need to GPS as close to the wheel base axis, or pivot point when turning, and Compass/IMU on the middle of the cart.

![](images/gps_compass.png)

### Algorithms

You can check on `nodes/gps_waypoints_navigation.py` for the code, I will explain roughly how does it work..

So first you will need a waypoints file as shown in `waypoints/mission.txt` for example. The format is same as Ardupilot as QGC WPL 110. So you can make the mission file from APM Planner and click on Save WP. I found that MAVLink has some issue of lat/lon decimal points, it's not accurate when transfer using MAVLink command, so I don't recommend to use "Write" or "Read" button on GCS.

Once the bot has waypoints file it will store on as array of `lat_target_list[], lon_targe_list[]`, then we gonna iterate with `target_wp` index counter to go to next watpoint. There are 5 step to make the bot go from current point to target waypoint.

![](images/step12.png)

1. Get the smallest angle to turn, and direction to turn.

2. Turn with constant sbus_steering speed, until it reahced the angke within threshold.

3. Get the distance how far it should go

4. Keep going to that point by using PID cross-track and heading controllers, if the bot is inside threshold of `x_track_error_start` then it will use PID heading, but if bot goes out of that threshold it will use PID cross-track. To calculate a related angle and distance variable, please check on two following images.

![](images/step4.png)

![](images/step34.png)

5. Once the bot reached the target waypoint within `goal_dist_thresh` threshold, it will iterate to next point and start with the step1 again.

Finally, if the robot can achieve all the point successfully, then it will just finish the mission and wait for user to switch back to auto mode again, as step 6.

### Run

- On the bot 

	- You will need to run gps, atcart, compass, RTK nodes as explained from jmoab-ros doc. 

	- On this package you will need to run `rosrun jmoab_autopilot_ros gps_waypoints_server.py` to start the parameters server. This is need when you firstly want to tune up the bot.

	- And on `jmoab_autopilot/nodes` you need to run a navigation script as `python gps_waypoints_navigation.py`, you can copy this script to your project directory and make some modification according to what you need.

- On the PC

	- Then you can run `rosrun rqt_reconfigure rqt_reconfigure` on you PC for GUI adjuster.

### Parameters

We are using `ros_dynamic_reconfigure`, so we can adjust the paramters lively from `rqt_reconfigure` GUI. Once you open it up it's like this,

![](images/gps_rqt_reconfigure.png)

The paramters in here are same as `cfg/GpsWaypoints.yaml`, so once you satisfied with tuning, you will need to update it manually on the bot on this file too (I will find the way for automate this later).

#### Steering

- max_start_str: 1054, this is a maximum sbus steering to make the cart start to turn right, each cart has different value depends on load

- min_start_str: 986, this is a minimum sbus steering to make the cart start to turn left, each cart has different value depends on load

- str_adj: 100, this is an adjuster sbus value to use with output PID mapping, so max output pid will use this full range +/- max/min_start_str

- skid_adj: 80, this is a pivot turn adjuster sbus value, so it will be +/- on max/min_start_str for right/left pivot-turning

- str_mid: 969

#### Throttle

- max_start_thr: 1080, this is a maximum sbus throttle to make the cart start to move forward.

- thr_adj: 100, this is an adjuster sbus value to + on max_start_thr, to use as constant speed during navigation.

- thr_slowest: 1140, this is the slowest sbus throttle in case of short distance navigation, we don't want the bot to move too fast when short range.

- thr_mid: 1024, a middle sbus throttle, no need to change this.

#### General Judgement Threshold

- goal_dist_thresh: 0.2, this is a radius of waypoints, it means if the cart is closed to target waypoint within this radius, it's considered as reached the point.

- goal_ang_thresh: 1.0, this is a threshold angle when cart is turning as pivot, so if difference between goal target angle and current angle is under this, it's considered as arrival at goal target angle.

- x_track_error_start: 0.12, this is a cross-track distance from the route to make the PID cross-track controller starts, when x_track_error less than this, the cart will use PID heading controller instead.

- x_track_repose_dist: 0.40, this is the maximum cross-track distance that tell the cart is too far from the route and it will start re-target heading again.

- pid_hdg_out_thresh: 0.7, if `diff_hdg` is less than this, it won't use PID heading and just keep driving straight.

- pid_x_out_thresh: 0.1, if `x_track_error` is less than this, it won't use PID cross-track, it's better to make it same as x_track_error_start or not higher.

#### PID gain

- hdg_p: 40.0, P gain of heading control

- hdg_i: 0.0, I gain of heading control

- hdg_d: 0.0, D gain of heading control

- cross_p: 150.0, P gain of cross-track control

- cross_i: 10.0, I gain of cross-track control

- cross_d: 0.0, D gain of cross-track control

### Tips

- If the heading is not good, the bot will move like snake and has a hard time to go back to the route, so please calibrate the heading offset again [see here](https://github.com/rasheeddo/jmoab-ros/blob/master/example/compass_calibration_step.md).

![](images/bad_hdg.png)


## Greenhouse Navigation

The greenhouse navigation is using only lidar and IMU to move around in the greenhouse.

The navigation pattern is using the a step sequence below

1. Wall-following: where the bot is using either left or right wall distance to measure how far from itself and try to keep the distance

2. Lane-changing: where the bot is outside the lane and try to change to another lane

3. Re-heading: when the bot arrived on the new lane, it will adjust the heading again

4. Wall-following(2): this is similar as first step

5. U-turning: where the bot `front_stop_dist` found some wall all of obstacle, it will turn around and count as one round completed.

![](images/greenhouse_sim.png)

The bot will follow this pattern forever, so for higher application you will need to check on `shelf_counter` inside the code, and stop whenever the last shelf of greenhouse got completed.

The parameters are showing,

![](images/greenhouse_rqt_reconfigure.png)


## Swarm

We can control multiple robots in the same time!

### Simple-Follower

This is an example of simple trailer follower of three bots, the first bot is master and controlled by user. The second follows the first bot, and the third follows the second.

![](images/multi_robots.png)

It has been simulated on [atcart_basic_gazebo](https://github.com/rasheeddo/atcart_basic_gazebo#multi-robots) model.

![](images/swarm_test.png)

please check on [this video](https://youtu.be/W5WBQbkODnA) for a demonstraion.

#### Run

In case of tuning the robot parameters, so just simple one-to-one swarm follower

```sh
# I assume that you have launch the multi robots model from atcart_basis_gazebo
# 1st terminal, swarm parameters server
rosrun jmoab_autopilot_ros  swarm_simple_follow_server.py --param_file path/to/your/param_file

# 2nd terminal, run the one to one program
# robot1 is master, you control it.
# robot2 is follower
cd ~/catkin_ws/src/jmoab_autopilot_ros/nodes

python swarm_simple_follwer.py --param_file  path/to/your/param_file --master_ns robot1 --follower_ns robot2
```

In case of you satisfied with parameters, and want to expand the robot

```sh
# I assume that you have launch the multi robots model from atcart_basis_gazebo
# go to launch directory fist, and open the file swarm_simple_follower.launch
cd ~/catkin_ws/src/jmoab_autopilot_ros/launch
# you will see two group, each group has unique name and which master and follower, 
# and it's using the same swarm_simple_follower.py
# if you would like to have more bots, please feel free to copy the group and change the name inside.

roslaunch jmoab_autopilot_ros swarm_simple_follow_server.launch
_ 
```

#### Parameters

- max_start_str: normally it is default as 1048

- min_start_str: normally it is default as 1000

- str_adj: how much the range of steering adjustment in PID

- skid_adj: how much the range of skidding will be performed

- thr_slowest: the slowest throttle to make the cart move in safety speed

- goal_dist_thresh: how far you want the bot to keep dist away when close to master robot

- goal_ang_thresh: a threshold angle to consider the bot it pointing correctly to master

- hdg_p: P gain control of heading PID controller

- hdg_i: I gain control of heading PID controller

- hdg_d: D gain control of heading PID controller

- vel_p: P gain control of velocity PID controller

- vel_i: I gain control of velocity PID controller

- vel_d: D gain control of velocity PID controller

