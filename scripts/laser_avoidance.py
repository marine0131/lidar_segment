#! /usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Twist
import math


class LaserBpDetection():
    def __init__(self, lamda, sigma):
        scan_topic = rospy.get_param("~scan", "scan")
        vel_topic = rospy.get_param("~vel_topic", "cmd_vel")
        self.scan_frame = rospy.get_param("~frame", "laser")
        self.SHOW_LINE = rospy.get_param("~show_line", "True")
        self.LAMBDA = lamda
        self.SIGMA = sigma
        rospy.Subscriber(scan_topic, LaserScan, self.scan_callback)
        self.center_marker_pub = rospy.Publisher('center_markers', Marker, queue_size=1)
        self.marker_pub = rospy.Publisher('markers', Marker, queue_size=1)
        self.vel_pub = rospy.Publisher(vel_topic, Twist, queue_size=1)
        self.count = 0
        while not rospy.is_shutdown():
            rospy.spin()

    def scan_callback(self, msg):
        if self.count == 5:
            self.count = 0
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
                if dist_last > 0.2 and dist_next > 0.2:
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
            for i, ind in enumerate(segment[1:-1]):
                seg_array = points[segment[i]: segment[i+1]]
                if len(seg_array) > 3:
                    cp = np.average(seg_array, axis=0)
                    std = np.std(seg_array, 0)
                    if std[0] < 0.1 and std[1] < 0.1 and abs(cp[1]) < 1.0:
                        center_points.append(cp)
            # fitting a line
            pos_y = []
            neg_y = []
            for pp in center_points:
                if pp[0] > 0:
                    if pp[1] > 0:
                        pos_y.append(pp)
                    else:
                        neg_y.append(pp)

            pos_y = np.array(pos_y)
            neg_y = np.array(neg_y)
            z_pos = np.polyfit(pos_y[:, 0], pos_y[:, 1], 1)
            z_neg = np.polyfit(neg_y[:, 0], neg_y[:, 1], 1)
            rospy.loginfo(z_pos)
            rospy.loginfo(z_neg)

            if self.SHOW_LINE:
                line = []
                line.append([0, z_pos[1]])
                line.append([3, 3 * z_pos[0] + z_pos[1]])
                line.append([3, 3 * z_neg[0] + z_neg[1]])
                line.append([0, z_neg[1]])
                self.marker_pub.publish(self.setting_marker(line, Marker.LINE_STRIP, 'red', [0.01, 0.01]))


            vel_msg = Twist()
            dist = [abs(z_pos[1]) / math.sqrt(z_pos[0]**2+1), -abs(z_neg[1]) / math.sqrt(z_neg[0]**2+1)]
            vel_msg.angular.z = (z_pos[0] + z_neg[0]) / 2.0
            vel_msg.linear.x = 0.3
            vel_msg.linear.y = (dist[0] + dist[1]) / 5

            # self.vel_pub.publish(vel_msg)
            rospy.loginfo(vel_msg)

            # self.marker_pub.publish(self.setting_marker(break_point_list,
                                        # Marker.POINTS, 'green'))
            self.center_marker_pub.publish(self.setting_marker(center_points, Marker.POINTS, 'blue', [0.05,0.05]))
        else:
            self.count = self.count + 1

    # transform ranges to cartesian x y and change to numpy array
    def cartesian_transformation(self, ranges, angle_min, angle_increment):
        pnts = []
        for i, r in enumerate(ranges):
            if not r == 0:
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
    lamda = 0.10
    sigma = 0.005
    LaserBpDetection(lamda, sigma)
