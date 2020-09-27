#!/usr/bin/env python3

import rospy

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from laser_line_extraction.msg import LineSegmentList
from geometry_msgs.msg import Point
import numpy as np



class RvizMarkers():
    def __init__(self):
        rospy.init_node('test_vis')
        rate = rospy.get_param('~rate', 10)
        self.update_rate = rospy.Rate(rate)
        self.line_vis_pub = rospy.Publisher('line_segment_markerarray', MarkerArray, queue_size=10)

        # Publishers/subscribers
        self.wall_detection_sub = rospy.Subscriber('/line_segments', LineSegmentList,
            self.wallDetectionCB, queue_size=1)
        self.wall_detection_msg = LineSegmentList()

        self.max_i = 0

    def wallDetectionCB(self, msg):
        self.wall_detection_msg = msg

    def visualizeLineSegments(self):
        marker_array = MarkerArray()

        real_idx = 0

        for i, line in enumerate(self.wall_detection_msg.line_segments):
            marker = Marker();

            # Ignore invalid points
            if np.isnan(line.start[0]):
                continue
            else:
                start_point = Point()
                start_point.x = line.start[0]
                start_point.y = line.start[1]
                end_point = Point()
                end_point.x = line.end[0]
                end_point.y = line.end[1]

                marker = Marker();
                marker.header.frame_id = self.wall_detection_msg.header.frame_id;
                marker.header.stamp = rospy.Time.now();
                marker.ns = "wall_detection";
                marker.id = real_idx;
                marker.type = Marker.LINE_STRIP;
                marker.action = Marker.ADD;
                # marker.pose.position.x = 1;
                # marker.pose.position.y = 1;
                # marker.pose.position.z = 1;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.03;
                marker.color.a = 1.0; # Don't forget to set the alpha!
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.points = [start_point, end_point]
                marker_array.markers.append(marker)
                real_idx += 1

                if real_idx > self.max_i:
                    self.max_i = real_idx

        # Delete old "ghost" markers
        for i in range(real_idx, self.max_i+3):
            marker = Marker();
            marker.header.frame_id = self.wall_detection_msg.header.frame_id;
            marker.header.stamp = rospy.Time.now();
            marker.ns = "wall_detection";
            marker.id = i;
            marker.action = Marker.DELETE;
            marker_array.markers.append(marker)

        self.max_i = real_idx
        self.line_vis_pub.publish(marker_array);

    def run(self):
        while not rospy.is_shutdown():
            self.vis_msg = MarkerArray()
            self.visualizeLineSegments()
            self.update_rate.sleep

if __name__ == "__main__":
    rviz_markers = RvizMarkers()
    rviz_markers.run()
