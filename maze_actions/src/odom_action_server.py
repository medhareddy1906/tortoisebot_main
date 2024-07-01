#!/usr/bin/env python3

import rospy
import actionlib
from maze_actions.msg import record_odomFeedback, record_odomResult, record_odomAction
from nav_msgs.msg import Odometry
from odom_sub import OdomTopicReader
from geometry_msgs.msg import Vector3
import math

class RecordOdomClass:
    def __init__(self, goal_distance):
        """
        Initializes the action server and starts it.
        """
        self._goal_distance = goal_distance
        self._seconds_recording = 360
        self._result = record_odomResult()
        self._odom_reader_object = OdomTopicReader()

        # Creates the action server
        self._as = actionlib.SimpleActionServer("/rec_odom_as", record_odomAction, self.goal_callback, False)
        self._as.start()

    def clean_variables(self):
        """
        Cleans variables for the next call
        """
        self._result = record_odomResult()

    def reached_distance_goal(self):
        """
        Returns True if the distance moved from the first instance of recording until now has reached the self._goal_distance
        """
        return self.check_if_out_maze(self._goal_distance, self._result.result_odom_array)

    def goal_callback(self, goal):
        success = True
        rate = rospy.Rate(1)
        for i in range(self._seconds_recording):
            rospy.loginfo("Recording Odom index=" + str(i))
            # Check that the preempt (cancellation) has not been requested by the action client
            if self._as.is_preempt_requested():
                rospy.logdebug('The goal has been cancelled/preempted')
                # Set the client in a preempted state (goal cancelled)
                self._as.set_preempted()
                success = False
                # End the action loop
                break
            else:
                # Build the next feedback msg to be sent
                if not self.reached_distance_goal():
                    rospy.logdebug('Reading Odometry...')
                    self._result.result_odom_array.append(self._odom_reader_object.get_odomdata())
                else:
                    rospy.logwarn('Reached distance Goal')
                    # End the action loop
                    break
            rate.sleep()

        # If successful, then publish the final result. If not successful, do not publish anything in the result
        if success:
            self._as.set_succeeded(self._result)

        # Clean the Result Variable
        self.clean_variables()

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
        if distance is None:
            return False
        return distance > goal_distance

if __name__ == '__main__':
    rospy.init_node('record_odom_action_server_node')
    RecordOdomClass(goal_distance=25.0)
    rospy.spin()
