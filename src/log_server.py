#!/usr/bin/env python3

from package_turtlesim_ogu.srv import log_frequency
from package_turtlesim_ogu.srv import log_frequencyRequest
from package_turtlesim_ogu.srv import log_frequencyResponse
import rospy
 
def handle_log_server(req):
    if req.log_interval < 0:
        return log_frequencyResponse('...Log interval is set to: {t} [sec]...'.format(t=req.log_interval))
    else:
        return log_frequencyResponse('...Log interval is set to: ')
   
def log_server():
    rospy.init_node('log_frequency_service_node')
    s = rospy.Service('log_frequency', log_frequency, handle_log_server)
    print("Log Server ready")

   
if __name__ == "__main__":
    log_server()
    rospy.spin()