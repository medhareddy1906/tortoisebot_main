#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
import math

class OdometryAnalysis:
    def __init__(self):
        pass

    def get_distance_moved(self, odometry_data_list):
        distance = None
        if len(odometry_data_list) >= 2:
            start_odom = odometry_data_list[0]
            end_odom = odometry_data_list[-1]
            start_position = start_odom.pose.pose.position
            end_position = end_odom.pose.pose.position
            rospy.loginfo("start_position ==>" + str(start_position))
            rospy.loginfo("end_position ==>" + str(end_position))
            distance_vector = self.create_vector(start_position, end_position)
            rospy.loginfo("Distance Vector ==>" + str(distance_vector))
            distance = self.calculate_length_vector(distance_vector)
            rospy.loginfo("Distance ==>" + str(distance))
        else:
            rospy.logerr("Odom array doesn't have the minimum number of elements = " + str(len(odometry_data_list)))
        return distance

    def create_vector(self, p1, p2):
        distance_vector = Vector3()
        distance_vector.x = p2.x - p1.x
        distance_vector.y = p2.y - p1.y
        distance_vector.z = p2.z - p1.z
        return distance_vector

    def calculate_length_vector(self, vector):
        length = math.sqrt(math.pow(vector.x, 2) + math.pow(vector.y, 2) + math.pow(vector.z, 2))
        return length

    def check_if_out_maze(self, goal_distance, odom_result_array):
        distance = self.get_distance_moved(odom_result_array)
        rospy.loginfo("Distance Moved=" + str(distance))
        # To exit we consider that each square in the floor is around 0.5m, therefore to exit correctly
        # distance has to be sqrt (6*5 + 5*4) = 7.8
        if distance is None:
            return False
        return distance > goal_distance

if __name__ == '__main__':
    rospy.init_node('odometry_analysis_node')
    odometry_analysis = OdometryAnalysis()
    
    # Example usage
    goal_distance = 7.8
    odom_result_array = []  # Populate this list with Odometry messages
    result = odometry_analysis.check_if_out_maze(goal_distance, odom_result_array)
    rospy.loginfo("Result: " + str(result))
