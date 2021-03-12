#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose

key_mapping = { 'w': [0, 1],  'x':[0, -1],
                'a': [-1, 0], 'd': [1, 0],
                's': [0, 0] }

g_last_twist = None
g_last_pose = None

def keys_callback(msg, twist_pub):
    global g_last_twist
    if len(msg.data) == 0 or msg.data[0] not in key_mapping:
        return # unknown key
    vels = key_mapping[msg.data[0]]
    g_last_twist.angular.z = vels[0]
    g_last_twist.linear.x  = vels[1]
    twist_pub.publish(g_last_twist)


if __name__ == '__main__':
    rospy.init_node('keys_to_twist')
    twist_pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('keys', String, keys_callback, twist_pub)
    g_last_twist = Twist()
    rate = rospy.Rate(2)
    
    while not rospy.is_shutdown():
        twist_pub.publish(g_last_twist)
        rate.sleep()