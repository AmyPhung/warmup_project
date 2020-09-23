#!/usr/bin/env python
"""
Wall Follower

Commands robot to drive parallel to the nearest wall. Uses the package
laser_line_extraction to detect line segments in a laser scan. Also takes
parameters from a ROS dynamic reconfigure server to tune parameters in
real time.

ROS Parameters:
- rate = update rate of node
- forward_vel = Constant velocity to drive at
- follow_dist = (currently unused) - plan to use this to set distance from wall
- kp1 = Proportional control to keep parallel heading to wall
- kp2 = (currently unusued) - plan to use this to adjust distance from wall

TODO:
+ reimplement line segment detection
+ incorporate distance from wall parameter
"""
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2, LaserScan
from laser_line_extraction.msg import LineSegmentList
from warmup_project.cfg import WallApproachConfig
from dynamic_reconfigure.server import Server
import numpy as np
import math

def lineseg_dists(p, a, b):
    """Cartesian distance from point to line segment

    Edited to support arguments as series, from:
    https://stackoverflow.com/a/54442561/11208892

    Args:
        - p: np.array of single point, shape (2,) or 2D array, shape (x, 2)
        - a: np.array of shape (x, 2)
        - b: np.array of shape (x, 2)
    """
    # normalized tangent vectors
    d_ba = b - a
    d = np.divide(d_ba, (np.hypot(d_ba[:, 0], d_ba[:, 1])
                           .reshape(-1, 1)))

    # signed parallel distance components
    # rowwise dot products of 2D vectors
    s = np.multiply(a - p, d).sum(axis=1)
    t = np.multiply(p - b, d).sum(axis=1)

    # clamped parallel distance
    h = np.maximum.reduce([s, t, np.zeros(len(s))])

    # perpendicular distance component
    # rowwise cross products of 2D vectors
    d_pa = p - a
    c = d_pa[:, 0] * d[:, 1] - d_pa[:, 1] * d[:, 0]

    return np.hypot(h, c)

def computeAngle(vec1, vec2):
    unit_vec1 = vec1 / np.linalg.norm(vec1)
    unit_vec2 = vec2 / np.linalg.norm(vec2)
    dot_product = np.dot(unit_vec1, unit_vec2)
    angle = np.arccos(dot_product)
    sign = np.sign(np.cross(unit_vec1, unit_vec2))
    return sign * angle

class WallFollower():
    def __init__(self):
        rospy.init_node("wall_follower")
        rate = rospy.get_param('~rate', 10)
        self.update_rate = rospy.Rate(rate)

        # Load ROS params
        self.forward_vel =  rospy.get_param('~forward_vel', 0.2)
        self.follow_dist =  rospy.get_param('~follow_dist', 0.5)
        self.kp1 =  rospy.get_param('~kp1', 0.4)
        self.kp2 =  rospy.get_param('~kp2', 0.4)

        # Start Dynamic Reconfigure Server
        srv = Server(WallApproachConfig, self.paramCB)

        # Publishers/subscribers
        self.wall_detection_sub = rospy.Subscriber('/line_segments', LineSegmentList,
            self.wallDetectionCB, queue_size=1)
        self.wall_detection_msg = LineSegmentList()

        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def paramCB(self, config, level):
        self.forward_vel =  config.forward_vel
        self.follow_dist =  config.follow_dist
        self.kp1 =  config.kp1
        self.kp2 =  config.kp2
        return config

    def wallDetectionCB(self, msg):
        self.wall_detection_msg = msg

    def parseLineSegmentList(self, lsl_msg):
        starts, ends = [], []

        for msg in lsl_msg.line_segments:
            # Filter nan line segments
            if math.isnan(msg.radius):
                continue
            starts.append(msg.start)
            ends.append(msg.end)

        return np.array(starts), np.array(ends)

    def findNearestWall(self, wall_msg):
        starts, ends = self.parseLineSegmentList(wall_msg)

        if len(starts) == 0 and len(ends) == 0:
            return [], []

        distances = lineseg_dists((0,0), starts, ends)

        idx = np.argmin(distances)
        return starts[idx], ends[idx]

    def computeAngularCommand(self, wall_start, wall_end):
        robot_vec = np.array((1,0))

        heading_offset_1 = computeAngle(robot_vec, wall_start - wall_end)
        heading_offset_2 = computeAngle(robot_vec, wall_end - wall_start)

        heading_offset = min(abs(heading_offset_1), (heading_offset_2))

        # TODO: Make PID
        ang_cmd = heading_offset * self.kp1
        return ang_cmd


    def run(self):
        while not rospy.is_shutdown():
            wall_start, wall_end = self.findNearestWall(self.wall_detection_msg)

            # Check for wall detections
            if len(wall_start) > 0:
                ang_cmd = self.computeAngularCommand(wall_start, wall_end)
                vel_cmd = self.forward_vel # TODO: make this not constant
                twist_msg = Twist()
                twist_msg.linear.x = vel_cmd
                twist_msg.angular.z = ang_cmd

                self.twist_pub.publish(twist_msg)
            self.update_rate.sleep()

if __name__ == "__main__":
    wall_follower = WallFollower()
    wall_follower.run()
