#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

class ScanSubscriber:
    def __init__(self):
        self._scan_data = None
        self._threshold = 1  # Adjust this threshold as needed

        rospy.init_node('scan_topic_subscriber', log_level=rospy.INFO)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.spin()

    def scan_callback(self, msg):
        self._scan_data = msg
        rospy.loginfo("Scan data received")

    def get_scan_data(self):
        """ Returns the newest scan data """
        return self._scan_data

    def four_sector_detection(self):
        """ Detects in which four directions there is an obstacle based on scan data """
        if self._scan_data is None or not self._scan_data.ranges:
            rospy.logwarn("No scan data received or scan data is empty.")
            return self.convert_to_dict("nothing")

        ranges = self._scan_data.ranges
        n = len(ranges)
        if n < 360:
            rospy.logwarn("Incomplete scan data received.")
            return self.convert_to_dict("nothing")

        front = min(ranges[0:45] + ranges[315:360]) if ranges[0:45] + ranges[315:360] else float('inf')
        left = min(ranges[45:135]) if ranges[45:135] else float('inf')
        back = min(ranges[135:225]) if ranges[135:225] else float('inf')
        right = min(ranges[225:315]) if ranges[225:315] else float('inf')

        axis_list = [right, front, back, left]
        min_distance = min(axis_list)
        max_axis_index = axis_list.index(min_distance)
        significant_value = min_distance < self._threshold

        if significant_value:
            if max_axis_index == 0:
                message = "right"
            elif max_axis_index == 1:
                message = "front"
            elif max_axis_index == 2:
                message = "back"
            elif max_axis_index == 3:
                message = "left"
            else:
                message = "unknown_direction"
        else:
            message = "nothing"

        return self.convert_to_dict(message)

    def convert_to_dict(self, message):
        """ Converts the given message to a dictionary telling in which direction there is a detection """
        detection_dict = {
            "front": message == "front",
            "left": message == "left",
            "right": message == "right",
            "back": message == "back"
        }
        return detection_dict

if __name__ == "__main__":
    scan_subscriber = ScanSubscriber()
