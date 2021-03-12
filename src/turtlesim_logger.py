#!/usr/bin/env python3

import rospy
import message_filters
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from package_turtlesim_ogu.msg import Aggregation
from package_turtlesim_ogu.srv import log_frequency
from package_turtlesim_ogu.srv import log_frequencyRequest
from package_turtlesim_ogu.srv import log_frequencyResponse
import time
import json

def aggregation_callback(agg_msg):
    global modus
    print(agg_msg)
    print('...log...')
    log_to_JSON_File(convert_msg_2_json(agg_msg))
    print('...wake...')

def log_to_JSON_File(jsonObject):
    global log_file
    global rate
    ''' relative to package path '''
    log_file = open('log_file.json', 'a')
    log_file.write(json.dumps(jsonObject))
    log_file.write('\n')
    log_file.close()
    time.sleep(log_service_request_client(modus))

def convert_msg_2_json(agg_msg):
    jsonObject = dict()
    jsonObject = { 
        'mean_values' : {
            'x' : agg_msg.mean_values.x,
            'y' : agg_msg.mean_values.y,
            'theta' : agg_msg.mean_values.theta,
            'linear_velocity' : agg_msg.mean_values.linear_velocity,
            'angular_velocity' : agg_msg.mean_values.angular_velocity
        },
        'max_values' : {
            'x' : agg_msg.max_values.x,
            'y' : agg_msg.max_values.y,
            'theta' : agg_msg.max_values.theta,
            'linear_velocity' : agg_msg.max_values.linear_velocity,
            'angular_velocity' : agg_msg.max_values.angular_velocity
        },
        'min_values' : {
            'x' : agg_msg.min_values.x,
            'y' : agg_msg.min_values.y,
            'theta' : agg_msg.min_values.theta,
            'linear_velocity' : agg_msg.min_values.linear_velocity,
            'angular_velocity' : agg_msg.min_values.angular_velocity
        }

    }
    return jsonObject

def log_service_request_client(mod):
    rospy.wait_for_service('log_frequency')
    try:
        log_client_obj = rospy.ServiceProxy('log_frequency', log_frequency)
        response = log_client_obj(mod)
        return response.log_interval
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

log_file = None
modus = None
rate = None

if __name__ == '__main__':
    modus = 1
    print('...start aggregation...')
    rospy.init_node('turtlesim_logger')
    rospy.Subscriber('aggregation_pose_values', Aggregation, aggregation_callback)
    print("...create log file...")
    with open('log_file.json', 'w') as fp:
        pass

    rospy.spin()
        
    