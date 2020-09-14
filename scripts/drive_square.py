#!/usr/bin/env python

import math
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

poses = [(0,0), (1,0), (1,1), (0,1), (0,0)]
# Once reaching each pose, the robot will stop and turn before moving forwards
# towards the next waypoint

def computeDistance(pt1, pt2):
    """ Compute 2D distance between two points """
    dist = math.sqrt((pt1[0] - pt2[0])**2 +
                     (pt1[1] - pt2[1])**2)
    return dist

def computeAngularOffset(desired_heading, current_angle):
    """ Compute delta between two euler angles, accounting for
    discontinuity at pi """
    error = desired_heading - current_angle
    # Handle discontinuity at pi
    if error > math.pi:
        error = error - 2*math.pi
    elif error < -math.pi:
        error = error + 2*math.pi
    return error

def extractEulerAngle(odom):
    """ Extract euler yaw from odometry message """
    orientation_list = [odom.pose.pose.orientation.x,
                        odom.pose.pose.orientation.y,
                        odom.pose.pose.orientation.z,
                        odom.pose.pose.orientation.w]

    (r, p, y) = euler_from_quaternion (orientation_list)

    return y # Yaw in radians

def computeDesiredAngle(prev_pose, next_pose):
    """ Compute angle between two poses with respect to world frame """
    v1 = np.array([1,0])
    v2 = np.array(np.array(next_pose) - np.array(prev_pose))
    v2 = v2 / np.linalg.norm(v2)
    dot_product = np.dot(v1, v2)
    cross_product = np.cross(v1, v2)
    angle = np.arccos(dot_product)

    if cross_product < 0: angle = -angle # Retain sign
    return angle # In radians


class P2PNavigation():
    """ Compute and publish twist commands to navigate between a list of
    input poses. Will drive to each pose, stop, and turn to face next pose
    before moving forwards. Uses odometry on /odom to obtain current
    position """

    def __init__(self, poses):
        rospy.init_node("p2p_navigation")

        self.poses = poses

        rate = rospy.get_param('~rate', 10)
        self.update_rate = rospy.Rate(rate)

        self.tolerance =  rospy.get_param('~tolerance', 0.05)

        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odomCB)
        self.odom_msg = Odometry()

        self.current_goal = 0
        self.desired_heading = 0
        self.state = 0 # 0 for turning, 1 for navigation

    def atWaypoint(self):
        odom_pos_2d = (self.odom_msg.pose.pose.position.x,
                       self.odom_msg.pose.pose.position.y)

        d = computeDistance(odom_pos_2d, self.poses[self.current_goal])

        if d < self.tolerance:
            return True
        else:
            return False

    def odomCB(self, msg):
        self.odom_msg = msg

    def computeAngularCmd(self, odom_msg, desired_heading):
        current_angle = extractEulerAngle(odom_msg)
        angular_offset = computeAngularOffset(desired_heading, current_angle)
        cmd = angular_offset * 0.9  # Proportional control
        return cmd

    def computeLinearCmd(self, odom_msg, current_goal):
        odom_pos_2d = (self.odom_msg.pose.pose.position.x,
                       self.odom_msg.pose.pose.position.y)
        linear_offset = computeDistance(odom_pos_2d, current_goal)
        cmd = linear_offset * 0.9 # Proportional control
        if cmd > 1: cmd = 1 # Cap command
        return cmd

    def run(self):
        while not rospy.is_shutdown():
            # Check if waypoint has been reached
            if self.atWaypoint() == True:
                rospy.loginfo("Reached waypoint " + str(self.current_goal))
                self.current_goal += 1
                self.state = 0 # Make robot turn before going to next point

                # Check if all waypoints have been reached
                if self.current_goal == len(self.poses):
                    rospy.loginfo("All waypoints reached. Exiting p2p node...")
                    break

                # Compute next desired heading
                prev_pose = self.poses[self.current_goal - 1]
                next_pose = self.poses[self.current_goal]
                self.desired_heading = computeDesiredAngle(prev_pose, next_pose)

            # Create twist message
            twist_cmd = Twist()

            # Check what state we're in
            if self.state == 0: # turning
                twist_cmd.angular.z = self.computeAngularCmd(self.odom_msg,
                    self.desired_heading)
                # Switch to driving state when close enough
                if twist_cmd.angular.z < self.tolerance:
                    self.state = 1
            else: # driving
                twist_cmd.angular.z = self.computeAngularCmd(self.odom_msg,
                    self.desired_heading)
                twist_cmd.linear.x = self.computeLinearCmd(self.odom_msg,
                    self.poses[self.current_goal])

            self.twist_pub.publish(twist_cmd)
            self.update_rate.sleep()

if __name__ == "__main__":
    p2p = P2PNavigation(poses)
    p2p.run()
