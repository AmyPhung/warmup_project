#!/usr/bin/env python

"""
Obstacle avoidance using the dynamic window approach

Based on: https://github.com/goktug97/DynamicWindowApproach
"""

# ROS Imports
import rospy
import ros_numpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
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
    def __init__(self, config):
        self.config = config

    def calculateVelocityCost(self):
        pass

    def calculateHeadingCost(self):
        pass

    def calculateClearanceCost(self):
        pass

    def createDynamicWindow(self, twist, config):
        dw = DynamicWindow()

        minV = max(config.minSpeed, twist.linear.x - config.maxAccel * config.dt)
        maxV = min(config.maxSpeed, twist.linear.x + config.maxAccel * config.dt)
        minW = max(-config.maxYawrate, twist.angular.z - config.maxdYawrate * config.dt);
        maxW = max(config.maxYawrate, twist.angular.z + config.maxdYawrate * config.dt)

        print("AFDSFS")
        print(minV)
        print(maxV)
        print(minW)
        print(maxW)

        dw.nPossibleV = int((maxV - minV) / config.velocityResolution)
        dw.nPossibleW = int((maxW - minW) / config.yawrateResolution)
        print(dw.nPossibleV)
        print(dw.nPossibleW)
        return dw

    def projectMotion(self, pose, twist, dt):
        """ Predict where robot will be in the next timestep """
        new_pose = Pose()
        eul_yaw = euler_from_quaternion([pose.orientation.x,
                                         pose.orientation.y,
                                         pose.orientation.z,
                                         pose.orientation.w])
        yaw = eul_yaw[2] + twist.angular.z * dt;
        quat = quaternion_from_euler(0,0,yaw)
        new_pose.orientation.x = quat[0]
        new_pose.orientation.y = quat[1]
        new_pose.orientation.z = quat[2]
        new_pose.orientation.w = quat[3]

        new_pose.position.x = pose.position.x + twist.linear.x * math.cos(yaw) * dt;
        new_pose.position.y = pose.position.y + twist.linear.y * math.sin(yaw) * dt;
        return new_pose;

    def computeTwistCommand(self, pose, twist, goal, pcl, config):
        """ Compute best twist command with the dynamic window implementation

        Args:
            pose (geometry_msgs/Pose): Current robot pose in /odom
            twist (geometry_msgs/Twist): Current twist from /odom
            goal (geometry_msgs/Pose): Goal pose in /odom
            pcl (sensor_msgs/PointCloud2): PCL generated from laser in robot frame
            config (Config): Settings to use for computation
        """
        print("Here!")


        dw = self.createDynamicWindow(twist, config)
        print(dw.nPossibleV)
        for i in range(dw.nPossibleV):
            for j in range(dw.nPossibleW):
                pPose = self.projectMotion(pose, twist, config.dt)
                print(pPose)
                # return pPose


class ObstacleAvoidance():
    def __init__(self):
        self.ready = False
        rospy.init_node("obstacle_avoidance")
        rospy.loginfo("Starting obstacle avoidance node...")

        rate = rospy.get_param('~rate', 10)
        self.update_rate = rospy.Rate(rate)

        srv = Server(ObstacleAvoidanceConfig, self.paramCB)

        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.projection_pub = rospy.Publisher("/projected_pose", Pose, queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odomCB)
        self.odom_msg = Odometry()
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goalCB)
        self.goal_msg = Pose()
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laserCB)
        self.laser_msg = LaserScan()

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
        self.config.maxAccel = rospy.get_param('~maxAccel', 3)
        self.config.maxdYawrate = rospy.get_param('~maxdYawrate', 6.0)
        self.config.velocityResolution = rospy.get_param('~velocityResolution', 0.05)
        self.config.yawrateResolution = rospy.get_param('~yawrateResolution', 0.05)
        self.config.dt = rospy.get_param('~dt', 0.1)
        self.config.predictTime = rospy.get_param('~predictTime', 3.0)
        self.config.heading = rospy.get_param('~heading', 0.15)
        self.config.clearance = rospy.get_param('~clearance', 1.0)
        self.config.velocity = rospy.get_param('~velocity', 1.0)
        self.config.base = robot_clearance

        # Path planner is used to compute next command
        self.path_planner = PathPlanner(self.config)

        # Used to create pointcloud from laser data
        self.lp = lg.LaserProjection()

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

        # Update path planner
        self.path_planner.config = self.config
        return config

    def odomCB(self, msg):
        self.odom_msg = msg

    def goalCB(self, msg):
        self.goal_msg = msg

    def laserCB(self, msg):
        self.laser_msg = msg

    def run(self):
        while not rospy.is_shutdown():
            # self.twist_pub.publish(twist_cmd)
            # print(self.odom_msg)
            # print(self.config.dt)
            pcl_msg = self.lp.projectLaser(self.laser_msg)

            twist_cmd = self.path_planner.computeTwistCommand(
                self.odom_msg.pose.pose, self.odom_msg.twist.twist,
                self.goal_msg, pcl_msg, self.config)

            # self.projection_pub.publish(pPose)
            self.update_rate.sleep()

if __name__ == "__main__":

    obstacle_avoidance = ObstacleAvoidance()
    obstacle_avoidance.run()
