#!/usr/bin/env python3

import rospy
import actionlib
from std_srvs.srv import Trigger, TriggerRequest
from maze_actions.msg import record_odomGoal, record_odomFeedback, record_odomResult, record_odomAction
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
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
        distance_vector = Twist()
        distance_vector.linear.x = p2.x - p1.x
        distance_vector.linear.y = p2.y - p1.y
        distance_vector.linear.z = p2.z - p1.z
        return distance_vector

    def calculate_length_vector(self, vector):
        length = math.sqrt(math.pow(vector.linear.x, 2) + math.pow(vector.linear.y, 2) + math.pow(vector.linear.z, 2))
        return length

def check_if_out_maze(goal_distance, odom_result_array):
    odom_analysis_object = OdometryAnalysis()
    distance = odom_analysis_object.get_distance_moved(odom_result_array)
    rospy.loginfo("Distance Moved=" + str(distance))
    if distance is None:
        return False
    return distance > goal_distance

class ControlTortoisebot(object):
    def __init__(self, goal_distance):
        self._goal_distance = goal_distance
        self.init_direction_service_client()
        self.init_rec_odom_action_client()
        self.init_move_tortoisebot_publisher()
        self._result = record_odomResult()

        self.current_yaw = 0.0
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw

    def get_current_yaw(self):
        return self.current_yaw

    def init_direction_service_client(self, service_name="/crash_direction_service"):
        rospy.loginfo('Waiting for Service Server')
        rospy.wait_for_service(service_name) # wait for the service to be running
        rospy.loginfo('Service Server Found...')
        self._direction_service = rospy.ServiceProxy(service_name, Trigger) # create the connection to the service
        self._request_object = TriggerRequest()

    def make_direction_request(self):
        result = self._direction_service(self._request_object) # send the request to the service
        return result.message

    def init_rec_odom_action_client(self):
        self._rec_odom_action_client = actionlib.SimpleActionClient('/rec_odom_as', record_odomAction)
        # waits until the action server is up and running
        rospy.loginfo('Waiting for action Server')
        self._rec_odom_action_client.wait_for_server()
        rospy.loginfo('Action Server Found...')
        self._rec_odom_action_goal = record_odomGoal()

    def send_goal_to_rec_odom_action_server(self):
        self._rec_odom_action_client.send_goal(self._rec_odom_action_goal, feedback_cb=self.rec_odom_feedback_callback)

    def rec_odom_feedback_callback(self, feedback):
        rospy.loginfo("Rec Odom Feedback feedback ==>" + str(feedback))

    def rec_odom_finished(self):
        has_finished = (self._rec_odom_action_client.get_state() >= 2)
        return has_finished

    def get_result_rec_odom(self):
        return self._rec_odom_action_client.get_result()

    def init_move_tortoisebot_publisher(self):
        # Initialize the publisher to the /cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist_object = Twist()
        self.linearspeed = 0.2
        self.angularspeed = 0.5

    def rotate_90_degrees(self, direction):
        initial_yaw = self.get_current_yaw()
        target_yaw = initial_yaw + (math.pi / 2 if direction == "left" else -math.pi / 2)

        if target_yaw > math.pi:
            target_yaw -= 2 * math.pi
        elif target_yaw < -math.pi:
            target_yaw += 2 * math.pi

        while abs(target_yaw - self.get_current_yaw()) > 0.01:
            self.twist_object.linear.x = 0.0
            self.twist_object.angular.z = self.angularspeed if direction == "left" else -self.angularspeed
            self.cmd_vel_pub.publish(self.twist_object)
            rospy.sleep(0.01)

        # Stop the robot after rotation
        self.twist_object.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist_object)

    def move_tortoisebot(self, direction):
        if direction == "forwards":
            self.twist_object.linear.x = self.linearspeed
            self.twist_object.angular.z = 0.0
        elif direction == "right":
            self.rotate_90_degrees("right")
            self.twist_object.linear.x = 0.0
            self.twist_object.angular.z = 0.0
        elif direction == "left":
            self.rotate_90_degrees("left")
            self.twist_object.linear.x = 0.0
            self.twist_object.angular.z = 0.0
        elif direction == "backwards":
            self.twist_object.linear.x = -self.linearspeed
            self.twist_object.angular.z = 0.0
        elif direction == "stop":
            self.twist_object.linear.x = 0.0
            self.twist_object.angular.z = 0.0
        else:
            pass

        # Publish the Twist message
        self.cmd_vel_pub.publish(self.twist_object)

    def got_out_maze(self, odom_result_array):
        return check_if_out_maze(self._goal_distance, odom_result_array)

if __name__ == '__main__':
    rospy.init_node("main_node", log_level=rospy.INFO)
    controltortoisebot_object = ControlTortoisebot(goal_distance=25.0)
    rate = rospy.Rate(10)
    controltortoisebot_object.send_goal_to_rec_odom_action_server()

    while not controltortoisebot_object.rec_odom_finished():
        direction_to_go = controltortoisebot_object.make_direction_request()
        rospy.loginfo(direction_to_go)
        controltortoisebot_object.move_tortoisebot(direction_to_go)
        rate.sleep()

    odom_result = controltortoisebot_object.get_result_rec_odom()
    odom_result_array = odom_result.result_odom_array

    if controltortoisebot_object.got_out_maze(odom_result_array):
        rospy.loginfo("Out of Maze")
    else:
        rospy.loginfo("In Maze")

    rospy.loginfo("Tortoisebot Maze test Finished")

    rospy.spin()
