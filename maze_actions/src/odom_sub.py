#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry

class OdomTopicReader:
    def __init__(self, topic_name='/odom'):
        self._topic_name = topic_name
        self._odomdata = Odometry()
        self._sub = rospy.Subscriber(self._topic_name, Odometry, self.topic_callback)

    def topic_callback(self, msg):
        self._odomdata = msg
        rospy.logdebug(self._odomdata)

    def get_odomdata(self):
        """
        Returns the newest odom data
        """
        return self._odomdata

if __name__ == "__main__":
    rospy.init_node('odom_topic_subscriber', log_level=rospy.INFO)
    odom_reader_object = OdomTopicReader()

    rospy.loginfo(odom_reader_object.get_odomdata())

    rate = rospy.Rate(0.5)
    ctrl_c = False

    def shutdownhook():
        global ctrl_c
        rospy.loginfo("shutdown time!")
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        data = odom_reader_object.get_odomdata()
        rospy.loginfo(data)
        rate.sleep()
