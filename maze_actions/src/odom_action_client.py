#!/usr/bin/env python3

import rospy
import time
import actionlib
from maze_actions.msg import record_odomGoal, record_odomFeedback, record_odomResult, record_odomAction
from nav_msgs.msg import Odometry

# Definition of the feedback callback. This will be called when feedback
# is received from the action server. It just prints a message indicating
# a new message has been received.
def feedback_callback(feedback):
    rospy.loginfo("Rec Odom Feedback feedback ==> " + str(feedback))

def count_seconds(seconds):
    for i in range(seconds):
        rospy.loginfo("Seconds Passed => " + str(i))
        time.sleep(1)

def main():
    # Initializes the action client node
    rospy.init_node('record_odom_action_client_node')
    
    # Create the connection to the action server
    client = actionlib.SimpleActionClient('/rec_odom_as', record_odomAction)
    rate = rospy.Rate(1)
    
    # Waits until the action server is up and running
    rospy.loginfo('Waiting for action Server')
    client.wait_for_server()
    rospy.loginfo('Action Server Found...')
    
    # Creates a goal to send to the action server
    goal = record_odomGoal()
    
    # Sends the goal to the action server, specifying which feedback function to call when feedback received
    client.send_goal(goal, feedback_cb=feedback_callback)
    
    # Get the state of the action server (1 if active, 2 when finished)
    state_result = client.get_state()
    rospy.loginfo("state_result: " + str(state_result))
    
    while state_result < 2:
        rospy.loginfo("Waiting to finish: ")
        rate.sleep()
        state_result = client.get_state()
        rospy.loginfo("state_result: " + str(state_result))
    
    rospy.loginfo("[Result] State: " + str(state_result))
    
    if state_result == 4:
        rospy.logerr("Something went wrong on the Server Side")
    elif state_result == 3:
        rospy.logwarn("There is a warning on the Server Side")
    
    rospy.loginfo("[Result] State: " + str(client.get_result()))

if __name__ == '__main__':
    main()
