cmake_minimum_required(VERSION 3.0.2)
project(auv-sim)

# TODO - set enc fortress
set(MSG_DEP_SET auv_msgs std_msgs geometry_msgs)

find_package(catkin REQUIRED COMPONENTS
	${MSG_DEP_SET}
	ros_ign_gazebo
	ros_ign_bridge
	gazebo_ros
	std_msgs
	controls
	propulsion
	rospy
	sensor_msgs
	state_estimation
	std_msgs
	tf
	vision
)

catkin_package(	CATKIN_DEPENDS 
	${MSG_DEP_SET}
	std_msgs
	controls
	gazebo_ros
	propulsion
	ros_ign_bridge
	ros_ign_gazebo
	rospy
	sensor_msgs
	state_estimation
	std_msgs
	tf
	vision
)

