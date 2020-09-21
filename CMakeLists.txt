cmake_minimum_required(VERSION 3.0.2)
project(warmup_project)

find_package(catkin REQUIRED
  COMPONENTS
    dynamic_reconfigure
    geometry_msgs
    laser_geometry
    laser_line_extraction
    nav_msgs
    rospy
    rosbag
    sensor_msgs
    std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/WallApproach.cfg
)
