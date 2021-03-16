#!/usr/bin/env python3

import sys, select, tty, termios
import rospy
from std_msgs.msg import String
from package_turtlesim_ogu.srv import turtlesim_keyboard_control
from package_turtlesim_ogu.srv import turtlesim_keyboard_controlRequest
from package_turtlesim_ogu.srv import turtlesim_keyboard_controlResponse

def keyboard_service_request_client(keyboard_control):
        rospy.wait_for_service('turtlesim_keyboard_control')
        try:
            keyboard_control_client_obj = rospy.ServiceProxy('turtlesim_keyboard_control', turtlesim_keyboard_control)
            response = keyboard_control_client_obj(keyboard_control)
            return response.keyboard_control_state
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

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
        keyboard_control_state = rospy.get_param('turtlesim_keyboard_control')
        keyboard_state = keyboard_service_request_client(keyboard_control=keyboard_control_state)
        
        if keyboard_state == 0 or keyboard_state == 1:
            ''' key pressed? '''
            if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
                key_pub.publish(sys.stdin.read(1)) # publish keystroke

        ''' go sleep after key pressed '''
        rate.sleep()
    
    ''' set back to standard behaviour '''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
