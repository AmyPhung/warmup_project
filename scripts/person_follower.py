#!/usr/bin/env python
"""
Follow a person based on lidar detections. Classifies a person as the nearest
cluster to the robot.

Initially attempted to use a convex hull to compute the area of the object but
this implementation was much less stable, so that code is commented out.

ROS Parameters:
- kp1 = proportional constant for postiion
- kp2 = proportional constant for angle
- rate = update rate
- debug = whether to display graphs & excess print statements
"""
import rospy
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
import ros_numpy
import numpy as np
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
import math
# from scipy.spatial import ConvexHull

def computeCOM(pts):
    avg_x = np.mean(pts[:, 0])
    avg_y = np.mean(pts[:, 1])
    return [avg_x, avg_y]

def computeDistance(pt1, pt2):
    dist = math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)
    return dist

def computeAngle(vec1, vec2):
    unit_vec1 = vec1 / np.linalg.norm(vec1)
    unit_vec2 = vec2 / np.linalg.norm(vec2)
    dot_product = np.dot(unit_vec1, unit_vec2)
    angle = np.arccos(dot_product)
    sign = np.sign(np.cross(unit_vec1, unit_vec2))
    return sign * angle

# def computeArea(pts):
#     hull = ConvexHull(pts)
#     return hull.area

class PersonFollower(object):
    def __init__(self):
        rospy.init_node("person_follower")
        rate = rospy.get_param('~rate', 10)
        self.update_rate = rospy.Rate(rate)
        self.debug = rospy.get_param('~debug', False)

        self.kp1 = rospy.get_param('~kp1', 0.5)
        self.kp2 = rospy.get_param('~kp2', 1)

        self.person_pub = rospy.Publisher("/person_position", PointStamped, queue_size=10)
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laserCB)
        self.laser_msg = LaserScan()

        self.lp = lg.LaserProjection()

    def laserCB(self, msg):
        self.laser_msg = msg

    def computeClusters(self, X):
        # Compute DBSCAN
        db = DBSCAN(eps=0.3, min_samples=10).fit(X)
        core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
        core_samples_mask[db.core_sample_indices_] = True
        labels = db.labels_

        # Number of clusters in labels, ignoring noise if present.
        n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
        n_noise = list(labels).count(-1)

        return labels, n_clusters, n_noise, core_samples_mask

    def plotResults(self, X, labels, core_samples_mask):
        # Black removed and is used for noise instead.
        unique_labels = set(labels)
        colors = [plt.cm.Spectral(each)
                  for each in np.linspace(0, 1, len(unique_labels))]
        for k, col in zip(unique_labels, colors):
            if k == -1:
                # Black used for noise.
                col = [0, 0, 0, 1]

            class_member_mask = (labels == k)

            xy = X[class_member_mask & core_samples_mask]
            plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
                     markeredgecolor='k', markersize=14)

            xy = X[class_member_mask & ~core_samples_mask]
            plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
                     markeredgecolor='k', markersize=6)
        plt.show()

    def run(self):
        while not rospy.is_shutdown():
            pcl_msg = self.lp.projectLaser(self.laser_msg)

            arr = np.asarray(ros_numpy.numpify(pcl_msg))
            # Only process non-empty scans
            if len(arr) != 0:
                # Compute clusters in array
                arr = np.array([list(arr) for arr in arr])
                X = arr[:,0:2]
                labels, n_clusters, n_noise, core_samples_mask = self.computeClusters(X)

                if self.debug:
                    print('Estimated number of clusters: %d' % n_clusters)
                    print('Estimated number of noise points: %d' % n_noise)
                    self.plotResults(X, labels, core_samples_mask)

                # Find nearest cluster
                centers = []
                distances = []
                for label in np.unique(labels):
                    # Ignore noise points
                    if label != -1:
                        filtered_pts = X[labels == label]
                        COM = computeCOM(filtered_pts)
                        centers.append(COM)
                        dist = computeDistance([0,0], COM)
                        distances.append(dist)

                centers = np.array(centers)
                distances = np.array(distances)

                if len(centers) == 0:
                    self.update_rate.sleep()
                    continue

                target_idx = np.argmin(distances)
                target_position = centers[target_idx]
                target_distance = distances[target_idx]
                target_heading = computeAngle((-1,0), target_position) # Using a -1 because the lidar is on backwards and I don't want to deal with transforms :)

                # Compute command
                twist_cmd = Twist()
                twist_cmd.linear.x = target_distance * self.kp1
                twist_cmd.angular.z = target_heading * self.kp2
                self.twist_pub.publish(twist_cmd)

                # Publish person position
                person_point = PointStamped()
                person_point.header.frame_id = "laser_link"
                person_point.header.stamp = rospy.Time.now()
                person_point.point.x = target_position[0]
                person_point.point.y = target_position[1]
                self.person_pub.publish(person_point)

            self.update_rate.sleep()


if __name__ == "__main__":
    person_follower = PersonFollower()
    person_follower.run()
