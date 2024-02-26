#!/usr/bin/python3

import rospy
import numpy as np
import sys, tty, termios, signal, select
from maxon_epos_msgs.msg import MotorState, MotorStates
from geometry_msgs.msg import Point
from cc_model.msg import CableActuation
import copy

MOTOR_NUM = 4
CABEL_LEN = 2.0
CABLE_OFFSET_ANGLE = np.pi/4 # angle between first cable and x-axis from our frame of reference
D_CABLE2CENTER = 0.1 # cable to center diameter
D_MOTOR2CABLE = 10.33 # cable to center diameter

positionBindings = {
    'q': (-1,1),
    'w': (0,1),
    'e': (1,1),
    'a': (-1,0),
    'd': (1,0),
    'z': (-1,-1),
    'x': (0,-1),
    'c': (1,-1)
}

class CatheterController():
    def __init__(self):
        rospy.on_shutdown(self.shutDown)
        rospy.init_node("demo_catheter_control", anonymous=True)

        # values
        self.curr_motors_targ_pos = MotorStates()
        self.curr_cable_delta_pos = CableActuation()
        
        # init node function
        self.initNode()
    
    def shutDown(self):
        rospy.loginfo("Stopping instant of EPOS...")

    def quit(signum, frame):
        print('stop program')
        sys.exit()
    
    def callback(self, data):
        x = data.x
        y = data.y
        direction = np.arctan2(x,y)
        distance = np.sqrt(x**2 + y**2)

        key = getKey(0.1)

        if key != ' ':
            # positions are in terms of number of rotations
            target_position = getTensionsFromPosition(direction,distance)
            print(direction)

            self.setAllTargetPositions(target_position)
            self.motors_pub.publish(self.curr_motors_targ_pos)    

    def initNode(self):
        self.motors_pub = rospy.Publisher("/maxon_bringup/set_all_states", MotorStates, queue_size=1)
        self.joystick_sub = rospy.Subscriber("/joystick", Point, self.callback)
        # self.cables_pub = rospy.Publisher("/cmd_cabledata", CableActuation, queue_size=1)
        each_motor_ctrl_state = MotorState()
        for i in range(MOTOR_NUM):
            self.curr_motors_targ_pos.states.append(copy.deepcopy(each_motor_ctrl_state))
        self.curr_motors_targ_pos.states[0].motor_name = "motor_1"
        self.curr_motors_targ_pos.states[1].motor_name = "motor_2"
        self.curr_motors_targ_pos.states[2].motor_name = "motor_3"
        self.curr_motors_targ_pos.states[3].motor_name = "motor_4"


    def setAllTargetPositions(self, target_position):
        self.curr_motors_targ_pos.states[0].position = target_position[0] * 2 * np.pi
        self.curr_motors_targ_pos.states[1].position = target_position[1] * 2 * np.pi
        self.curr_motors_targ_pos.states[2].position = target_position[2] * 2 * np.pi
        self.curr_motors_targ_pos.states[3].position = target_position[3] * 2 * np.pi
    
    def setAllTargetVelocities(self, target_velocity):
        self.curr_motors_targ_pos.states[0].velocity = target_velocity[0]
        self.curr_motors_targ_pos.states[1].velocity = target_velocity[1]
        self.curr_motors_targ_pos.states[2].velocity = target_velocity[2]
        self.curr_motors_targ_pos.states[3].position = target_velocity[3]

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def getTensionsFromPosition(direction, scale):
    tensions = np.array([scale * np.cos(direction - CABLE_OFFSET_ANGLE - i*np.pi/2) for i in range(4)])
    return tensions

def getCableLengthDeltasFromTensions(tensions):
    deltas = np.array([tension * 2 * np.pi * D_MOTOR2CABLE for tension in tensions])
    return deltas

def getCableLengthDeltasFromPosition(direction, scale):
    deltas = np.array([scale * np.cos(direction - CABLE_OFFSET_ANGLE - i*np.pi/2) for i in range(4)])
    return deltas

def getTensionsFromDeltas(deltas): 
    tensions = np.array([delta / (2 * np.pi * D_MOTOR2CABLE) for delta in deltas])
    return tensions


if __name__ == "__main__": 
    settings = termios.tcgetattr(sys.stdin)
    # rospy.loginfo("step 0")
    CCTRL = CatheterController()


    rospy.spin()