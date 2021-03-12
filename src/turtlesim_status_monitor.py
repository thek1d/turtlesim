#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from package_turtlesim_ogu.msg import Aggregation

class PoseRobot():
    def __init__(self):
        self.__x = 0.0
        self.__y = 0.0
        self.__theta = 0.0
        self.__linear_velocity = 0.0
        self.__angular_velocity = 0.0

    @property
    def x(self):
        return self.__x 
        
    @x.setter
    def x(self, x):
        self.__x = x
        
    @property
    def y(self):
        return self.__y
    
    @y.setter
    def y(self, y):
        self.__y = y
    
    @property
    def theta(self):
        return self.__theta
    
    @theta.setter
    def theta(self, theta):
        self.__theta = theta
    
    @property
    def linear_velocity(self):
        return self.__linear_velocity
    
    @linear_velocity.setter
    def linear_velocity(self, linear_velocity):
        self.__linear_velocity = linear_velocity

    @property
    def angular_velocity(self):
        return self.__angular_velocity
    
    @angular_velocity.setter
    def angular_velocity(self, angular_velocity):
        self.__angular_velocity = angular_velocity

class TurtleState():
    def __init__(self):
        self.pose = PoseRobot()
        self.poseList = [None] * 10 # creating list with 10 elements
        self.counter = 0

def pose_callback(msg, agg_pub):
    global tState, tAggregation
    tState.pose.x = msg.x
    tState.pose.y = msg.y
    tState.pose.theta = msg.theta
    tState.pose.linear_velocity = msg.linear_velocity
    tState.pose.angular_velocity = msg.angular_velocity
    aggregate_list(tState)
    agg_pub.publish(tAggregation)

def aggregate_list(tState):
    global tAggregation
    ''' first of all creating a list '''
    if tState.counter % 10 != 0 or tState.counter == 0:
        tState.poseList[tState.counter] = tState.pose
        tState.counter = tState.counter + 1
    else:
        tState.counter = 0
        ''' aggregation '''
        calc_and_set_mean(tState.poseList, tAggregation)
        calc_and_set_min(tState.poseList, tAggregation)
        calc_and_set_max(tState.poseList, tAggregation)

def calc_and_set_mean(list, aggregation):
    temp_x = 0
    temp_y = 0
    temp_theta = 0
    temp_linear_velocity = 0
    temp_angular_velocity = 0

    for elem in list:
        temp_x = temp_x + elem.x
        temp_y = temp_y + elem.y
        temp_theta = temp_theta + elem.theta
        temp_linear_velocity = temp_linear_velocity + elem.linear_velocity
        temp_angular_velocity = temp_angular_velocity + elem.angular_velocity

    x = temp_x / len(list)
    y = temp_y / len(list)
    theta = temp_theta / len(list)
    linear_velocity = temp_linear_velocity / len(list)
    angular_velocity = temp_angular_velocity / len(list)
    
    aggregation.mean_values.x = x
    aggregation.mean_values.y = y
    aggregation.mean_values.theta = theta
    aggregation.mean_values.linear_velocity = linear_velocity
    aggregation.mean_values.angular_velocity = angular_velocity
    
def calc_and_set_min(list, aggregation):
    min_x = 0
    min_y = 0
    min_theta = 0
    min_linear_velocity = 0
    min_angular_velocity = 0

    for elem in list:
        if min_x < elem.x :
            min_x = elem.x
        if min_y < elem.y :
            min_y = elem.y
        if min_theta < elem.theta:
            min_theta = elem.theta
        if min_linear_velocity < elem.linear_velocity:
            min_linear_velocity = elem.linear_velocity
        if min_angular_velocity < elem.angular_velocity:
            min_angular_velocity = elem.angular_velocity
    
    aggregation.min_values.x = min_x
    aggregation.min_values.y = min_y
    aggregation.min_values.theta = min_theta
    aggregation.min_values.linear_velocity = min_linear_velocity
    aggregation.min_values.angular_velocity = min_angular_velocity

def calc_and_set_max(list, aggregation):
    max_x = 0
    max_y = 0
    max_theta = 0
    max_linear_velocity = 0
    max_angular_velocity = 0

    for elem in list:
        if max_x > elem.x :
            max_x = elem.x
        if max_y > elem.y :
            max_y = elem.y
        if max_theta > elem.theta:
            max_theta = elem.theta
        if max_linear_velocity > elem.linear_velocity:
            max_linear_velocity = elem.linear_velocity
        if max_angular_velocity > elem.angular_velocity:
            max_angular_velocity = elem.angular_velocity
    
    aggregation.max_values.x = max_x
    aggregation.max_values.y = max_y
    aggregation.max_values.theta = max_theta
    aggregation.max_values.linear_velocity = max_linear_velocity
    aggregation.max_values.angular_velocity = max_angular_velocity
    

tState = None
tAggregation = None
    
if __name__ == '__main__':
    tState = TurtleState()
    tAggregation = Aggregation()
    rospy.init_node('turtlesim_status_monitor')
    agg_pub = rospy.Publisher('aggregation_pose_values', Aggregation, queue_size=1)
    rospy.Subscriber('turtle1/pose', Pose, pose_callback, agg_pub)
    rospy.spin()
    

