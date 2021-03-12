#!/usr/bin/env python3

import sys, select, tty, termios
import rospy
from std_msgs.msg import String

if __name__ == '__main__':

    ''' registering topic '''
    key_pub = rospy.Publisher(name='keys', data_class=String, queue_size=1)

    ''' register client node '''
    rospy.init_node('turtlesim_control')

    ''' set update intervall of topic to 500ms'''
    rate = rospy.Rate(2)

    ''' storing standard behaviour of stdin (line behaviour)'''
    old_attr = termios.tcgetattr(sys.stdin)

    ''' altering behaviour of stdin to character not stream '''
    tty.setcbreak(fd=sys.stdin.fileno())

    print('Publishing key strokes. Press Ctrl-C to exit...')

    while not rospy.is_shutdown():
        ''' key pressed? '''
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            key_pub.publish(sys.stdin.read(1)) # publish keystroke

        ''' go sleep after key pressed '''
        rate.sleep()
    
    ''' set back to standard behaviour '''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
