#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from package_turtlesim_ogu.msg import Aggregation
from package_turtlesim_ogu.srv import log_frequency
from package_turtlesim_ogu.srv import log_frequencyRequest
from package_turtlesim_ogu.srv import log_frequencyResponse
import time
import datetime
import json
import os
import rospkg

class Logger():

    def __init__(self, interval):
        if interval < 0 :
            self.__log_interval = 1
            is_set_log_interval = self.__log_frequency(interval)
            print(is_set_log_interval)
            print('...log interval wrong...\n ...cannot be negativ...\n... set to default=1...\n ...please adjust "log_interval" in config/params.yaml...')
        else:
            self.__log_interval = interval
            is_set_log_interval = self.__log_frequency(interval)
            print(is_set_log_interval + '{t} [sec]...'.format(t=self.__log_interval))

        self.__aggregated_msg = Aggregation()
    
    def __log_frequency(self, interval):
        return self.__log_service_request_client(interval)

    def __log_service_request_client(self, mod):
        rospy.wait_for_service('log_frequency')
        try:
            log_client_obj = rospy.ServiceProxy('log_frequency', log_frequency)
            response = log_client_obj(mod)
            return response.log_text
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    
    def Sub2Topic(self, topic, data_class, cb):
        rospy.Subscriber(topic, data_class, cb)
    
    def aggregation_cb(self, msg):
        self.setPose(msg.pose)
        self.setColor(msg.color)
        self.setVel(msg.cmd_vel)
    
    def setPose(self, pose):
        self.__aggregated_msg.pose = pose

    def getPose(self):
        return self.__aggregated_msg.pose

    def setVel(self, vel):
        self.__aggregated_msg.cmd_vel = vel

    def getVel(self):
        return self.__aggregated_msg.cmd_vel

    def setColor(self, color):
        self.__aggregated_msg.color = color

    def getColor(self):
        return self.__aggregated_msg.color

    def getAggMsg(self):
        return (self.getPose(), self.getVel(), self.getColor())

    @property
    def log_interval(self):
        return self.__log_interval


class JsonConverter():

    def __init__(self):
        self.__rospack = rospkg.RosPack()
        self.__package_path = self.__rospack.get_path('package_turtlesim_ogu')

    def convert_msg_2_json_object(self, vel, pos, col):
        date = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        temp = {'timestamp': None,
                'cmd_vel':{'linear':{'x' : None, 'y' : None, 'z' : None},
                           'angular':{'x' : None, 'y' : None, 'z' : None}},
                'pose':   {'x' : None,
                           'y' : None,
                           'theta' : None,
                           'linear_velocity' : None,
                           'angular_velocity' : None},
                'color':  {'r' : None,
                           'g' : None,
                           'b' : None}}

        temp['timestamp'] = date
        temp['cmd_vel']['linear']['x']  = vel.linear.x
        temp['cmd_vel']['linear']['y']  = vel.linear.y
        temp['cmd_vel']['linear']['z']  = vel.linear.z
        temp['cmd_vel']['angular']['x'] = vel.angular.x
        temp['cmd_vel']['angular']['y'] = vel.angular.y
        temp['cmd_vel']['angular']['z'] = vel.angular.z

        temp['pose']['x'] =                pos.x
        temp['pose']['y'] =                pos.y
        temp['pose']['theta'] =            pos.theta
        temp['pose']['linear_velocity'] =  pos.linear_velocity
        temp['pose']['angular_velocity'] = pos.angular_velocity

        temp['color']['r'] = col.r
        temp['color']['g'] = col.g
        temp['color']['b'] = col.b

        return temp
    
    def append_2_log_file(self, json_object):
      try:
        log = open(self.__package_path +'/Docs/Logs/logfile.json', 'a')
        log.write(json.dumps(json_object))
        log.write('\n')
        log.close()
      except:
        print('could not create file')

    @property
    def logfile(self):
        return self.__logfile
    
if __name__ == '__main__':
    rospy.init_node('turtlesim_logger')
    logger = Logger(interval=rospy.get_param('/log_interval'))
    logger.Sub2Topic('aggregation_values', Aggregation, logger.aggregation_cb)
    jConverter = JsonConverter()
    
    while not rospy.is_shutdown():
        time.sleep(float(logger.log_interval))
        pose, vel, color = logger.getAggMsg()
        temp_json = jConverter.convert_msg_2_json_object(vel, pose ,color)
        jConverter.append_2_log_file(temp_json)
    
    
    
    
    