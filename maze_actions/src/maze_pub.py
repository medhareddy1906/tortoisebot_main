#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class CmdVelPub:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('cmd_vel_publisher_node')

        # Initialize the publisher to the /cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist_object = Twist()
        self.linearspeed = 0.2
        self.angularspeed = 0.5

        # Global variable to control the main loop
        self.ctrl_c = False

        # Register the shutdown hook
        rospy.on_shutdown(self.shutdownhook)

        # Rate at which to run the loop
        self.rate = rospy.Rate(1)  # 1 Hz

    def move_robot(self, direction):
        if direction == "forwards":
            self.twist_object.linear.x = self.linearspeed
            self.twist_object.angular.z = 0.0
        elif direction == "right":
            self.twist_object.linear.x = 0.0
            self.twist_object.angular.z = -self.angularspeed
        elif direction == "left":
            self.twist_object.linear.x = 0.0
            self.twist_object.angular.z = self.angularspeed
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

    def shutdownhook(self):
        self.ctrl_c = True
        self.move_robot(direction="stop")
        rospy.loginfo("Shutdown time!")

    def main_loop(self):
        while not self.ctrl_c:
            # Move the robot forwards
            self.move_robot(direction="forwards")
            self.rate.sleep()

if __name__ == "__main__":
    cmd_vel_pub = CmdVelPub()
    cmd_vel_pub.main_loop()
