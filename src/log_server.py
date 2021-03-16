#!/usr/bin/env python3

from package_turtlesim_ogu.srv import log_frequency
from package_turtlesim_ogu.srv import log_frequencyRequest
from package_turtlesim_ogu.srv import log_frequencyResponse
import rospy
 
def handle_log_server(req):
    if req.mod < 0:
        return log_frequencyResponse(0)
    else:
        return log_frequencyResponse(req.mod)
   
def log_server():
    rospy.init_node('log_frequency')
    s = rospy.Service('log_frequency', log_frequency, handle_log_server)
    print("Log Server ready")

   
if __name__ == "__main__":
    log_server()
    rospy.spin()