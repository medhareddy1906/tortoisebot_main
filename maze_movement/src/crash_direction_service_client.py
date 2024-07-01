#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerRequest

def main():
    # Initialize a ROS node with the name 'crash_direction_service_client'
    rospy.init_node('crash_direction_service_client')
    
    # Define the service name
    service_name = "/crash_direction_service"
    
    # Wait for the service to be available
    rospy.wait_for_service(service_name)
    
    # Create a service proxy to call the service
    direction_service = rospy.ServiceProxy(service_name, Trigger)
    
    # Create a request object
    request_object = TriggerRequest()
    
    # Set the rate at which to call the service (5 Hz)
    rate = rospy.Rate(5)
    
    # Variable to control the shutdown process
    ctrl_c = False
    
    # Define a shutdown hook
    def shutdownhook():
        global ctrl_c
        rospy.loginfo("Shutdown time!")
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)
    
    # Loop to call the service and log the results
    while not ctrl_c:
        try:
            # Call the service and get the result
            result = direction_service(request_object)
            
            # Log the result
            if result.success:
                rospy.logwarn("Success = " + str(result.success))
                rospy.logwarn("Direction To Go = " + str(result.message))
            else:
                rospy.loginfo("Success = " + str(result.success))
                rospy.loginfo("Direction To Go = " + str(result.message))
            
            # Sleep for a while before calling the service again
            rate.sleep()
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    main()
