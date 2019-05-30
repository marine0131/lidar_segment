#! /usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math


class LaserBpDetection():
    def __init__(self):
        scan_topic = rospy.get_param("~scan", "scan")
        self.scan_frame = rospy.get_param("~frame", "laser")
        self.separate_dist = rospy.get_param("~separate_distance", 0.3)
        self.cluster_std = rospy.get_param("~cluster_std", 0.1)
        self.LAMBDA = 0.07
        self.SIGMA = 0.005
        rospy.Subscriber(scan_topic, LaserScan, self.scan_callback)
        self.center_marker_pub = rospy.Publisher('center_markers', Marker, queue_size=1)
        self.marker_pub = rospy.Publisher('markers', Marker, queue_size=1)

        while not rospy.is_shutdown():
            rospy.spin()

    def scan_callback(self, msg):
        theta = msg.angle_increment
        angle_min = msg.angle_min

        # ranges to cartisian
        points = self.cartesian_transformation(msg.ranges, angle_min, theta)

        # filter, delete isolated point
        m = 1
        for i in range(1, len(points)-1):
            # dist_next = self.euclidean_distance(points[i], points[i+1])
            dist_last = np.sqrt(np.sum((points[m-1] - points[m])**2, axis=0))
            dist_next = np.sqrt(np.sum((points[m] - points[m+1])**2, axis=0))
            if dist_last > self.separate_dist and dist_next > self.separate_dist:
                points = np.delete(points, m, 0)
            else:
                m = m+1

        # find break points and index
        segment = [0]
        break_point_list = []
        for i in range(2, len(points)):
            Dthd = (math.sqrt(points[i-1][0]**2 + points[i-1][1]**2) *
                    (math.sin(theta) / math.sin(self.LAMBDA - theta)) + 3 * self.SIGMA)
            dist = np.sqrt(np.sum((points[i] - points[i-1])**2, axis=0))

            if dist > Dthd:
                segment.append(i)
                break_point_list.append(points[i-1])
                break_point_list.append(points[i])

        # extract legs and calculate center coor
        center_points = []
        for i, ind in enumerate(segment[0:-1]):
            seg_array = points[segment[i]: segment[i+1]]
            if len(seg_array) > 5:
                cp = np.average(seg_array, axis=0)
                std = np.std(seg_array, 0)
                if std[0] < self.cluster_std and std[1] < self.cluster_std:
                    center_points.append(cp)
        self.marker_pub.publish(self.setting_marker(break_point_list,
                                                    Marker.POINTS, 'green',[0.1, 0.1]))
        self.center_marker_pub.publish(self.setting_marker(center_points,
                                                           Marker.POINTS, 'blue', [0.1,0.1]))

    # transform ranges to cartesian x y and change to numpy array
    def cartesian_transformation(self, ranges, angle_min, angle_increment):
        pnts = []
        for i, r in enumerate(ranges):
            if r != 0 and r < 100:
                pnts.append([r*math.cos(angle_min + i * angle_increment),
                             r*math.sin(angle_min + i * angle_increment)])
        return np.array(pnts)

    # def euclidean_distance(self, p1, p2):
    #     return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)

    def setting_marker(self, point_list, _type, color, scale):
        marker = Marker()
        marker.header.frame_id = self.scan_frame
        marker.id = 0
        marker.type = _type
        marker.lifetime = rospy.Duration(1.0)
        if color == 'red':
            marker.color.r = 1.0
        elif color == 'green':
            marker.color.g = 1.0
        elif color == 'blue':
            marker.color.b = 1.0
        marker.color.a = 1.0
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]

        for point in point_list:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            marker.points.append(p)

        # rospy.loginfo(len(marker.points))
        return marker


if __name__ == '__main__':
    rospy.init_node("laser_breakpoint_detection")
    LaserBpDetection()
