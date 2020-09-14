cmake_minimum_required(VERSION 3.0.2)
project(warmup_project)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    nav_msgs
    rospy
    rosbag
    sensor_msgs
    std_msgs
)
