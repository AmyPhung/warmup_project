#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
from laser_line_extraction.msg import LineSegmentList
import numpy as np

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

class WallFollower():
    def __init__(self):
        rospy.init_node("wall_follower")
        rate = rospy.get_param('~rate', 10)
        self.update_rate = rospy.Rate(rate)

        self.follow_dist =  rospy.get_param('~follow_dist', 0.5)
        self.visualize =  rospy.get_param('~visualize', True)

        self.wall_detection_sub = rospy.Subscriber('/line_segments', LineSegmentList,
            self.wallDetectionCB, queue_size=1)
        self.wall_detection_msg = LineSegmentList()

    def wallDetectionCB(self, msg):
        self.wall_detection_msg = msg

    def parseLineSegmentList(lsl_msg):
        for msg in lsl_msg:
            print(msg)

    def findNearestWall(self, wall_msg):
        # wall_starts =
        # lineseg_dists(p, a, b)
        pass

    def run(self):
        while not rospy.is_shutdown():
            self.findNearestWall(self.wall_detection_msg)

            self.update_rate.sleep()

if __name__ == "__main__":
    wall_follower = WallFollower()
    wall_follower.run()
