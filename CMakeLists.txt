cmake_minimum_required(VERSION 3.0.2)
project(auv-sim)

# TODO - set enc fortress
set(MSG_DEP_SET std_msgs geometry_msgs)
set(ros_ign_gazebo_DIR "${CMAKE_SOURCE_DIR}/../../../auv-ign/install/share/ros_ign_gazebo/cmake")
set(ros_ign_DIR "${CMAKE_SOURCE_DIR}/../../../auv-ign/install/share/ros_ign/cmake")
set(ros_ign_image_DIR "${CMAKE_SOURCE_DIR}/../../../auv-ign/install/share/ros_ign_image/cmake")
set(ros_ign_bridge_DIR "${CMAKE_SOURCE_DIR}/../../../auv-ign/install/share/ros_ign_bridge/cmake")

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

