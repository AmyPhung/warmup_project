#!/usr/bin/env python

"""
Obstacle avoidance using the dynamic window approach

Based on: https://github.com/goktug97/DynamicWindowApproach
"""

# ROS Imports
import rospy
import ros_numpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import PointCloud2, LaserScan
from warmup_project.cfg import ObstacleAvoidanceConfig
from dynamic_reconfigure.server import Server

# Python
import math
import numpy as np
import laser_geometry.laser_geometry as lg
import matplotlib.pyplot as plt

# Datatypes --------------------------------------------------------
class Rect():
    def __init__(self):
        self.xmin = 0.0
        self.ymin = 0.0
        self.xmax = 0.0
        self.ymax = 0.0

class Config():
    def __init__(self):
        self.maxSpeed = 0.0
        self.minSpeed = 0.0
        self.maxYawrate = 0.0
        self.maxAccel = 0.0
        self.maxdYawrate = 0.0
        self.velocityResolution = 0.0
        self.yawrateResolution = 0.0
        self.dt = 0.0
        self.predictTime = 0.0
        self.heading = 0.0
        self.clearance = 0.0
        self.velocity = 0.0
        self.base = Rect()

class DynamicWindow():
    def __init__(self):
        self.nPossibleV = 0
        possibleV = []
        self.nPossibleW = 0
        possibleW = []

# Path planner ------------------------------------------------------
class PathPlanner():
    def __init__(self):
        pass

    def calculateVelocityCost(self):
        pass

    def calculateHeadingCost(self):
        pass

    def calculateClearanceCost(self):
        pass

    def createDynamicWindow(self, twist, config):
        dw = DynamicWindow()

        minV = max(config.minSpeed, velocity.linearVelocity - config.maxAccel * config.dt)
        maxV = min(config.maxSpeed, velocity.linearVelocity + config.maxAccel * config.dt)
        minW = max(-config.maxYawrate, velocity.angularVelocity - config.maxdYawrate * config.dt);
        maxW = max(config.maxYawrate, velocity.angularVelocity + config.maxdYawrate * config.dt)

        dw.nPossibleV = (maxV - minV) / config.velocityResolution
        dw.nPossibleW = (maxW - minW) / config.yawrateResolution
        return dw


    def computeTwistCommand(self, pose, twist, goal, pcl, config):
        dw = createDynamicWindow(twist, config)


class ObstacleAvoidance():
    def __init__(self):
        self.ready = False
        rospy.init_node("obstacle_avoidance")
        rospy.loginfo("Starting obstacle avoidance node...")

        rate = rospy.get_param('~rate', 10)
        self.update_rate = rospy.Rate(rate)

        srv = Server(ObstacleAvoidanceConfig, self.paramCB)

        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odomCB)
        self.odom_msg = Odometry()

        robot_clearance = Rect()
        # Assume robot is a 40x40cm box with the center at 0,0
        robot_clearance.xmin = rospy.get_param('~xmin', 0.2)
        robot_clearance.ymin = rospy.get_param('~ymin', 0.2)
        robot_clearance.xmax = rospy.get_param('~xmax', 0.2)
        robot_clearance.ymax = rospy.get_param('~ymax', 0.2)

        self.config = Config()
        self.config.maxSpeed = rospy.get_param('~maxSpeed', 1.0)
        self.config.minSpeed = rospy.get_param('~minSpeed', -1.0)
        self.config.maxYawrate = rospy.get_param('~maxYawrate', 3.0)
        self.config.maxAccel = rospy.get_param('~maxAccel', 0.3)
        self.config.maxdYawrate = rospy.get_param('~maxdYawrate', 1.0)
        self.config.velocityResolution = rospy.get_param('~velocityResolution', 0.2)
        self.config.yawrateResolution = rospy.get_param('~yawrateResolution', 0.2)
        self.config.dt = rospy.get_param('~dt', 0.1)
        self.config.predictTime = rospy.get_param('~predictTime', 3.0)
        self.config.heading = rospy.get_param('~heading', 0.15)
        self.config.clearance = rospy.get_param('~clearance', 1.0)
        self.config.velocity = rospy.get_param('~velocity', 1.0)
        self.config.base = robot_clearance

        rospy.loginfo("Obstacle avoidance node setup complete!")
        self.ready = True

    def paramCB(self, config, level):
        if not self.ready:
            return config

        self.config.maxSpeed = config.maxSpeed
        self.config.minSpeed = config.minSpeed
        self.config.maxYawrate = config.maxYawrate
        self.config.maxAccel = config.maxAccel
        self.config.maxdYawrate = config.maxdYawrate
        self.config.velocityResolution = config.velocityResolution
        self.config.yawrateResolution = config.yawrateResolution
        self.config.dt = config.dt
        self.config.predictTime = config.predictTime
        self.config.heading = config.heading
        self.config.clearance = config.clearance
        self.config.velocity = config.velocity
        return config

    def odomCB(self, msg):
        self.odom_msg = msg

    def run(self):
        while not rospy.is_shutdown():
            # self.twist_pub.publish(twist_cmd)
            print(self.odom_msg)
            print(self.config.dt)
            self.update_rate.sleep()

if __name__ == "__main__":

    obstacle_avoidance = ObstacleAvoidance()
    obstacle_avoidance.run()
