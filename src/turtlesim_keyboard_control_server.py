#!/usr/bin/env python3

from package_turtlesim_ogu.srv import turtlesim_keyboard_control
from package_turtlesim_ogu.srv import turtlesim_keyboard_controlRequest
from package_turtlesim_ogu.srv import turtlesim_keyboard_controlResponse
import rospy
 
class TurtlesimKeyboardControlState():
    def __init__(self):
        self.__keyboard_control = False
    
    @property
    def keyboard_control(self):
        return self.__keyboard_control
    
    @keyboard_control.setter
    def keyboard_control(self, state):
        self.__keyboard_control = state


def handle_turtlesim_control_server(req):
    global turtlesim_keyboard_control_state
    if req.keyboard_control and turtlesim_keyboard_control_state.keyboard_control:
        ''' keyboard already given '''
        return turtlesim_keyboard_controlResponse(0)
    elif req.keyboard_control and not turtlesim_keyboard_control_state.keyboard_control:
        ''' keyboard control on '''
        turtlesim_keyboard_control_state.keyboard_control = True
        return turtlesim_keyboard_controlResponse(1)
    elif not req.keyboard_control and turtlesim_keyboard_control_state.keyboard_control:
        ''' keyboard control abort '''
        turtlesim_keyboard_control_state.keyboard_control = False
        return turtlesim_keyboard_controlResponse(2)
    elif not req.keyboard_control and not turtlesim_keyboard_control_state.keyboard_control:
        ''' keyboard control already aborted '''
        return turtlesim_keyboard_controlResponse(3)
    else:
        ''' should not be the case '''
        return turtlesim_keyboard_controlResponse(-1)
   
def turtlesim_keyboard_control_server():
    rospy.init_node('turtlesim_control_service_node')
    s = rospy.Service('turtlesim_keyboard_control', turtlesim_keyboard_control, handle_turtlesim_control_server)

turtlesim_keyboard_control_state = None
   
if __name__ == "__main__":
    turtlesim_keyboard_control_server()
    turtlesim_keyboard_control_state = TurtlesimKeyboardControlState()
    rospy.spin()