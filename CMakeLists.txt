cmake_minimum_required(VERSION 3.0.2)
project(warmup_project)

find_package(catkin REQUIRED
  COMPONENTS
    dynamic_reconfigure
    geometry_msgs
    laser_geometry
    laser_line_extraction
    message_generation
    nav_msgs
    ros_numpy
    rospy
    rosbag
    sensor_msgs
    std_msgs
)
add_message_files(
  FILES
    JoystickInput.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/WallApproach.cfg
  cfg/ObstacleAvoidance.cfg
)
