cmake_minimum_required(VERSION 3.0.2)
project(auv-sim)

# TODO - set enc fortress
set(MSG_DEP_SET std_msgs geometry_msgs)

find_package(catkin REQUIRED COMPONENTS
	${MSG_DEP_SET}
	rospy
	ros_ign_gazebo
	ros_ign_bridge
	gazebo_ros
	std_msgs
	sensor_msgs
	tf
)

catkin_package(	CATKIN_DEPENDS 
	${MSG_DEP_SET}
	rospy
	ros_ign_gazebo
	ros_ign_bridge
	gazebo_ros
	std_msgs
	sensor_msgs
	tf
)

