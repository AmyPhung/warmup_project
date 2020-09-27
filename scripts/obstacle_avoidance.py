#!/usr/bin/env python

"""
Obstacle avoidance (simple edition)

Originallly attempted to do something more complicated but then ran out of
time. This implementation attempts to drive towards a goal but the output
heading is influenced by detected points in the laser scan


        compute offset from desired position

        constant velocity

        for each point in a -45 to 45 degree cone, add heading adjustment

"""

# ROS Imports
import rospy
import ros_numpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler, \
    quaternion_multiply, quaternion_conjugate, unit_vector
from sensor_msgs.msg import PointCloud2, LaserScan
from warmup_project.cfg import ObstacleAvoidanceConfig
from dynamic_reconfigure.server import Server

# Python
import sys
import math
import numpy as np
import laser_geometry.laser_geometry as lg
import matplotlib.pyplot as plt

def computeAngle(vec1, vec2):
    unit_vec1 = vec1 / np.linalg.norm(vec1)
    unit_vec2 = vec2 / np.linalg.norm(vec2)
    dot_product = np.dot(unit_vec1, unit_vec2)
    angle = np.arccos(dot_product)
    sign = np.sign(np.cross(unit_vec1, unit_vec2))
    return sign * angle

def computeDistance(pt1, pt2):
    dist = math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)
    return dist

class ObstacleAvoidance():
    def __init__(self):
        self.ready = False
        rospy.init_node("obstacle_avoidance")
        rospy.loginfo("Starting obstacle avoidance node...")

        rate = rospy.get_param('~rate', 10)
        self.update_rate = rospy.Rate(rate)
        self.visualize = rospy.get_param('~visualize', True)

        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odomCB)
        self.odom_msg = Odometry()
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goalCB)
        self.goal_msg = None
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laserCB)
        self.laser_msg = LaserScan()

        self.k1 = rospy.get_param('~k1', 0.7)
        self.k2 = rospy.get_param('~k2', -0.005)
        self.lin_vel = rospy.get_param('~linear_velocity', 0.5)

    def odomCB(self, msg):
        self.odom_msg = msg

    def goalCB(self, msg):
        rospy.loginfo("Goal Received!")
        self.goal_msg = msg

    def laserCB(self, msg):
        self.laser_msg = msg

    def computeHeadingOffset(self, pose, goal):
        """ Compute heading offset between current robot heading and
        goal position.

        Args:
            pose (Pose): Robot pose message from /odom, relative to world frame
            goal (PoseStamped): Robot goal, relative to world frame

        Returns:
            angle_offset (float): Robot heading offset, relative to robot.
            Range capped at [-pi, pi]. Negative if goal is to the right,
            positive if to the left """

        dx = goal.pose.position.x - pose.position.x
        dy = goal.pose.position.y - pose.position.y
        desired_heading_odom = math.atan2(dy, dx)

        current_heading_odom = euler_from_quaternion([pose.orientation.x,
                                                       pose.orientation.y,
                                                       pose.orientation.z,
                                                       pose.orientation.w])
        angle_offset = desired_heading_odom - current_heading_odom[2];

        # Handle discontinuity at pi
        if angle_offset > math.pi:
            angle_offset = angle_offset - 2*math.pi
        elif angle_offset < -math.pi:
            angle_offset = angle_offset + 2*math.pi

        return angle_offset

    def computeLinearVelocity(self, pose, goal):
        dist = computeDistance([goal.pose.position.x, goal.pose.position.y],
                               [pose.position.x, pose.position.y])
        if dist < 0.05: # If robot is within 5cm of goal, stop
            return 0
        # Make velocity command proportional to distance
        vel_cmd = dist * self.lin_vel

        # Cap veloctiy command
        if vel_cmd > self.lin_vel:
            return self.lin_vel
        elif vel_cmd < 0:
            return 0
        return vel_cmd

    def computeTrajectoryAdjustment(self):
        # Only consider objects in a -60 to 60 degree cone
        # (60 deg = approx 1 rad)
        if self.laser_msg.angle_increment == 0:
            rospy.logwarn_once("No valid lidar data")
            return 0

        # Lidar is mounted backwards, so "rotate" ranges
        ranges = self.laser_msg.ranges[180:] + self.laser_msg.ranges[:181]

        start_idx = \
            int((-1-self.laser_msg.angle_min)/self.laser_msg.angle_increment)
        end_idx = len(ranges) - \
            int((self.laser_msg.angle_max-1)/self.laser_msg.angle_increment)

        heading_adjustment = 0

        for idx in range(start_idx, end_idx):
            angle = self.laser_msg.angle_min + \
                idx*self.laser_msg.angle_increment
            distance = ranges[idx]

            # Avoid divide by zero error
            if abs(angle) < 0.001:
                continue

            if not np.isinf(distance):
                adj = (1/distance) * 3*(1/angle)
                heading_adjustment += adj

        return heading_adjustment

    def computeTwistCmd(self):
        # target_heading = computeAngle((1,0), target_position)
        heading_offset = self.computeHeadingOffset(self.odom_msg.pose.pose,
                                                   self.goal_msg)

        obstacle_factor = self.computeTrajectoryAdjustment()

        vel = self.computeLinearVelocity(self.odom_msg.pose.pose,
                                         self.goal_msg)

        cmd = Twist()
        cmd.linear.x = vel
        cmd.angular.z = self.k1*heading_offset + self.k2*obstacle_factor

        return cmd

    def run(self):
        while not rospy.is_shutdown():
            if self.goal_msg == None:
                rospy.loginfo_once("Waiting for goal...")
                continue

            cmd = self.computeTwistCmd()
            self.twist_pub.publish(cmd)
            self.update_rate.sleep()

if __name__ == "__main__":
    obstacle_avoidance = ObstacleAvoidance()
    obstacle_avoidance.run()
