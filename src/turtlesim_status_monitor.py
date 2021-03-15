#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose, Color
from package_turtlesim_ogu.msg import Aggregation

class TurtleState():
    def __init__(self):
        self.__pose = Pose()
        self.__cmd_vel = Twist()
        self.__color = Color()
        self.__aggregate = Aggregation()
        self.__publisher = rospy.Publisher('aggregation_values', Aggregation, queue_size=1)
    
    def Sub2Topic(self, topic, data_type, cb):
        rospy.Subscriber(topic, data_type, cb)
    
    def PubAggregation(self, pose, cmd_vel, color):
        self.__aggregate.cmd_vel = cmd_vel
        print('agg_cmd_vel: ', self.__aggregate.cmd_vel)
        self.__aggregate.color = color
        print('agg_color: ', self.__aggregate.color)
        self.__aggregate.pose  = pose
        print('agg_pose: ', self.__aggregate.pose)
        self.__publisher.publish(self.__aggregate)

    def pose_cb(self, msg):
        self.__pose = msg
    
    def cmd_vel_cb(self, msg):
        self.__cmd_vel = msg
    
    def color_cb(self, msg):
        self.__color = msg

    @property
    def pose(self):
        return self.__pose

    @property
    def cmd_vel(self):
        return self.__cmd_vel
    
    @property
    def color(self):
        return self.__color   

        
def main():
    rospy.init_node('turtlesim_status_monitor')
    tState = TurtleState()
    tState.Sub2Topic('turtle1/pose', Pose, tState.pose_cb)
    tState.Sub2Topic('turtle1/cmd_vel', Twist, tState.cmd_vel_cb)
    tState.Sub2Topic('turtle1/color_sensor', Color, tState.color_cb)    

    while not rospy.is_shutdown():
        tState.PubAggregation(pose=tState.pose, cmd_vel=tState.cmd_vel, color=tState.color)
    
if __name__ == '__main__':
    main()
    

