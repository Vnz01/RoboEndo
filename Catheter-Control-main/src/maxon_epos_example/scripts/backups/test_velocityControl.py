#!/usr/bin/python3

import rospy
import numpy as np
import sys
import signal
import copy
# from std_msgs.msg import Float32MultiArray
from maxon_epos_msgs.msg import MotorState, MotorStates
from cc_model.msg import CableActuation


MOTOR_NUM = 3
CABEL_LEN = 2.0
D_CABLE2CENTER = 0.1 # cable to center diameter
D_MOTOR2CABLE = 1 # cable to center diameter

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
    

    def initNode(self):
        self.motors_pub = rospy.Publisher("/maxon_bringup/set_all_states", MotorStates, queue_size=1)
        # self.cables_pub = rospy.Publisher("/cmd_cabledata", CableActuation, queue_size=1)
        each_motor_ctrl_state = MotorState()
        for i in range(MOTOR_NUM):
            self.curr_motors_targ_pos.states.append(copy.deepcopy(each_motor_ctrl_state))
        self.curr_motors_targ_pos.states[0].motor_name = "motor_1"
        self.curr_motors_targ_pos.states[1].motor_name = "motor_2"
        self.curr_motors_targ_pos.states[2].motor_name = "motor_3"
        self.curr_motors_targ_pos.states[3].motor_name = "motor_4"

        print(self.curr_motors_targ_pos.states[0].motor_name)


    def setAllTargetPositions(self, target_position):
        self.curr_motors_targ_pos.states[0].position = target_position[0] * 2 * np.pi
        self.curr_motors_targ_pos.states[1].position = target_position[1] * 2 * np.pi
        self.curr_motors_targ_pos.states[2].position = target_position[2] * 2 * np.pi
        # self.curr_motors_targ_pos.states[3].position = 10 * 2 * np.pi
    
    def setAllTargetVelocities(self, target_velocity):
        self.curr_motors_targ_pos.states[0].velocity = target_velocity[0]
        self.curr_motors_targ_pos.states[1].velocity = target_velocity[1]
        self.curr_motors_targ_pos.states[2].velocity = target_velocity[2]
        # self.curr_motors_targ_pos.states[3].position = 10 * 2 * np.pi

        # print(CCTRL.curr_motors_targ_pos.states[0].motor_name)
        


if __name__ == "__main__":

    # rospy.loginfo("step 0")
    CCTRL = CatheterController()

    sim_time = 100
    n_trajectory = np.linspace(0, 1 * np.pi, sim_time)

    step =  0
    try:
        while not rospy.is_shutdown():
            signal.signal(signal.SIGINT, quit)
            signal.signal(signal.SIGTERM, quit)

            theta = n_trajectory[step]

            delta_cable_13 =   2.0 + 1.0 * np.sin(theta) + CABEL_LEN
            delta_cable_24 =   1.0 + 2.0 * np.cos(theta) + CABEL_LEN

            motor_ctrl_13 =   delta_cable_13*0.01 / D_MOTOR2CABLE
            motor_ctrl_24 =   delta_cable_24*0.01 / D_MOTOR2CABLE

            
            # target_position = np.array([ motor_ctrl_13 , motor_ctrl_24,  - motor_ctrl_13])
            target_velocity= np.array([ 0.0 , 0.0,  0.0])

            CCTRL.setAllTargetVelocities(target_velocity)

            # CCTRL.curr_cable_delta_pos.cableLen[0] = delta_cable_13
            # CCTRL.curr_cable_delta_pos.cableLen[1] = delta_cable_24
            # CCTRL.curr_cable_delta_pos.cableLen[2] = - delta_cable_13
            # CCTRL.curr_cable_delta_pos.cableLen[3] = - delta_cable_24

            # CCTRL.curr_cable_delta_pos.cableLen[0] = 0.01
            # CCTRL.curr_cable_delta_pos.cableLen[1] = 0.01
            # CCTRL.curr_cable_delta_pos.cableLen[2] = - 0.01
            # CCTRL.curr_cable_delta_pos.cableLen[3] = - 0.01

            

            CCTRL.motors_pub.publish(CCTRL.curr_motors_targ_pos)
            rospy.sleep(0.2)
            # CCTRL.cables_pub.publish(CCTRL.curr_cable_delta_pos)
            print("sended goal")

    except rospy.ROSInterruptException:
        pass
    
