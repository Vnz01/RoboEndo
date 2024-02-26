#!/usr/bin/python3

import rospy
from pySerialTransfer import pySerialTransfer as txfer
import numpy as np
import sys, tty, termios, signal, select
from maxon_epos_msgs.msg import MotorState, MotorStates
from cc_model.msg import CableActuation
import copy
from pynput import mouse



######################################## SETTINGS DEFINITIONS ############################################################################

# kinematic parameters of the catheter
MOTOR_NUM = 4
CABEL_LEN = 2.0
CABLE_OFFSET_ANGLE = np.pi/4                 # angle between first cable and x-axis from our frame of reference
D_CABLE2CENTER = 0.1                         # cable to center diameter
D_MOTOR2CABLE = 10.33                        # cable to center diameter
CABLE_DISPLACEMENT_TENSIONING_SLOPE = 1      # rate at which input sigal converts to tensioning displacement 
CABLE_DISPLACEMENT_RELEASING_SLOPE = 1       # rate at which input sigal converts to releasing displacement 
TENSION_DIR = [-1,1,-1,1]                    # this converts the spooling directions so that +ve on setAllTargetPositions is tension, -ve is release
JOYSTICK_AVAILABLE = 0                       # SET THIS TO 1 if you are using Joystick, otherwise 0
KEYBOARD_AVAILABLE = 0                       # SET THIS TO 1 if you are using Keyboard, otherwise 0
MOUSE_AVAILABLE = 1                          # SET THIS TO 1 if you are using Mouse, otherwise 0
JOYSTICK_SERIALPORT = '/dev/ttyACM0'         # Serial port for joystick 
JOYSTICK_ZERO_XY_OFFSET = [0.06,0.07]        # makes reading from joystick as close as possible to 0 when the joystick is not touched

# Bindings that map a keyboard press to an individual motor directional turn (for 4 motors there are 8 keyboard presses)
keyboardControl = {
    'w': [0.05, 0],
    'a': [0, 0.05],
    's': [-0.05, 0],
    'd': [0, -0.05]
}


##################################### CLASS DEFINITIONS ############################################################################

# Catheter Class
# contains the initialization, shutdown sequences, set position, and set velocities for the catheter robot motors ONLY
class CatheterController():
    def __init__(self):
        rospy.on_shutdown(self.shutDown)
        rospy.init_node("demo_catheter_control", anonymous=True)

        # instantiate a desired_motor_states object to published
        self.desired_motor_state = MotorStates() 

        # start node for the controller loop
        self.initNode()

        # create an offset variable
        self.spool_zero_offset = np.zeros(4)
    
    def shutDown(self):
        rospy.loginfo("Stopping instant of EPOS...")

    def quit(signum, frame):
        print('stop program')
        sys.exit()
    

    def initNode(self):
        #set this node up to publish target motor states
        self.desired_motor_state_pub = rospy.Publisher("/maxon_bringup/set_all_states", MotorStates, queue_size=1)

        each_motor_ctrl_state = MotorState()
        for i in range(MOTOR_NUM):
            self.desired_motor_state.states.append(copy.deepcopy(each_motor_ctrl_state))
        self.desired_motor_state.states[0].motor_name = "motor_1"
        self.desired_motor_state.states[1].motor_name = "motor_2"
        self.desired_motor_state.states[2].motor_name = "motor_3"
        self.desired_motor_state.states[3].motor_name = "motor_4"

    def setAllTargetPositions(self, target_position):
        self.desired_motor_state.states[0].position = TENSION_DIR[0] * (target_position[0]+self.spool_zero_offset[0]) * 2 * np.pi 
        self.desired_motor_state.states[1].position = TENSION_DIR[1] * (target_position[1]+self.spool_zero_offset[1]) * 2 * np.pi 
        self.desired_motor_state.states[2].position = TENSION_DIR[2] * (target_position[2]+self.spool_zero_offset[2]) * 2 * np.pi 
        self.desired_motor_state.states[3].position = TENSION_DIR[3] * (target_position[3]+self.spool_zero_offset[3]) * 2 * np.pi 

    def zeroingSequence(self):
        print("REMOVE THE CATHETER before hitting 'w' key...")
        while(getKey(None)!='w'):
            rospy.sleep(0.02) #wait for a key press

        print("Zeroing in 1 seconds...")
        rospy.sleep(1) 
        print("Zeroing now.")
        target_position = np.zeros(4)
        CCTRL.setAllTargetPositions(target_position)
        CCTRL.desired_motor_state_pub.publish(CCTRL.desired_motor_state)    
        rospy.sleep(1)
        print("Replace the catheter. Then, press 'w' to continue...")
        while(getKey(None)!='w'):
            rospy.sleep(0.02) 
        print("Tensioning sequence.")
        for i in range(0,MOTOR_NUM):
            print("Tensioning Spool "+str(i)+" ...")
            while(True):
                a = getKey(0.01)
                if(a=='w'):
                    break
                if(a=='p'):
                    target_position[i] = target_position[i]+0.01
                    self.setAllTargetPositions(target_position)
                    CCTRL.desired_motor_state_pub.publish(CCTRL.desired_motor_state) 
                if(a=='m'):
                    target_position[i] = target_position[i]-0.01
                    self.setAllTargetPositions(target_position)
                    CCTRL.desired_motor_state_pub.publish(CCTRL.desired_motor_state)

            # set the offset
            #self.spool_zero_offset[i] = target_position[i]
            
            #loosen that spool
            #target_position[i] = -0.01
            rospy.sleep(1)  
        print("Completed tensioning. Returning to Zero...")
        target_position = np.zeros(4)
        self.setAllTargetPositions(target_position)
        CCTRL.desired_motor_state_pub.publish(CCTRL.desired_motor_state) 

        print("Completed. The robot is now active.")
        return 0

    def inverse_kinematics_constant_curvature(self,theta_x,theta_y):
        # uses constant curvature assumption to find the actuation for all 4 tendons
        # theta_x is angle of curvature in x axis, whereas theta_y is angle of curvature in y
        # The equationworks out to simply offset_x = D_CABLE2CENTER*theta_x
        # We consider a potential difference in releasing and tensioning tension despite it ideally being the same.

        # convert angular displacement in x to cable displacement on 2 cables
        if theta_x > 0:
            # the robot is moving in direction of spool1 (which is tensioning), so it should tension faster than spool2
            spool1_targ_angle = CABLE_DISPLACEMENT_TENSIONING_SLOPE * D_CABLE2CENTER * theta_x 
            spool2_targ_angle = CABLE_DISPLACEMENT_RELEASING_SLOPE * D_CABLE2CENTER * -theta_x 
        else:
            # the robot is moving in direction of spool2 (which is tensioning), so it should tension faster than spool1
            spool1_targ_angle = CABLE_DISPLACEMENT_RELEASING_SLOPE * D_CABLE2CENTER * theta_x 
            spool2_targ_angle = CABLE_DISPLACEMENT_TENSIONING_SLOPE * D_CABLE2CENTER * -theta_x 

        # convert angular dispalcement in y to cable displacement on other 2 cables
        if theta_y > 0:
            # the robot is moving in direction of spool3 (which is tensioning), so it should tension faster than spool4
            spool3_targ_angle = CABLE_DISPLACEMENT_TENSIONING_SLOPE * D_CABLE2CENTER * theta_y 
            spool4_targ_angle = CABLE_DISPLACEMENT_RELEASING_SLOPE * D_CABLE2CENTER * -theta_y 
        else:
            # the robot is moving in direction of spool4 (which is tensioning), so it should tension faster than spool3
            spool3_targ_angle = CABLE_DISPLACEMENT_RELEASING_SLOPE * D_CABLE2CENTER * theta_y 
            spool4_targ_angle = CABLE_DISPLACEMENT_TENSIONING_SLOPE * D_CABLE2CENTER * -theta_y 

        return np.array([spool1_targ_angle, spool2_targ_angle, spool3_targ_angle, spool4_targ_angle])



# getKey function get a keyboard presss
def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key






class MouseClass():

    def __init__(self):
        self.offset_x = 1980
        self.offset_y = 1080
        self.offset_angle = 0
        # Create a mover object that keep mouse centered on screen
        # so that it never reaches a wall
        self.mover = mouse.Controller()

        # main
        self.mover.position = (self.offset_x,self.offset_y) #center the mouse
        self.normalized_x = 0
        self.normalized_y = 0

        #create mouse callback
        self.listener = mouse.Listener(on_move=self.on_move,on_click=self.on_click,on_scroll=self.on_scroll)

        self.listener.start()
        print("initialize mouse tracker")


    def on_move(self,x, y):
        Dx = x-self.offset_x
        Dy = y-self.offset_y
        Dist = np.sqrt(Dx**2 + Dy**2)
        if Dist > 1000: #move beyond a unit circle of 1000 pixels
            x = Dx/Dist*990 + self.offset_x
            y = Dy/Dist*990 + self.offset_y
            self.mover.position = (x,y)
            self.normalized_x = Dx/Dist*.990
            self.normalized_y = Dy/Dist*.990
        else:
            self.normalized_x = Dx/1000
            self.normalized_y = Dy/1000

    def on_click(self,x, y, button, pressed):
        print('{0} at {1}'.format('Pressed' if pressed else 'Released',(x, y)))
        if not pressed:
            # Stop listener
            print('UnPressed')
            return True #change to false to stop listening

    def on_scroll(self, x, y, dx, dy):
        self.offset_angle += 5*dy/180*np.pi
        print('Angle Change = ', np.round(self.offset_angle/np.pi*180))
#        print('Scrolled {0} at {1}'.format('down' if dy < 0 else 'up', (x, y)))










######################################## MAIN PROGRAM ############################################################################

if __name__ == "__main__": 
    settings = termios.tcgetattr(sys.stdin)

    # Instantiate a Catheter object
    CCTRL = CatheterController()

    # Set the target position of the pulleys to zero (remember that zero is the position the last time the maxon drivers were power-cycled)
    target_position = np.zeros(4) 

    # zeroing sequence
    CCTRL.zeroingSequence()

    try:
        #Start Control Loop
        # things to set before the loop
        offset_theta = 0 # this accounts for angle offsets between joystick and output of the catheter
        x = 0 #assume x,y of a unit circle
        y = 0 #assume x,y of a unit circle


        # Start joystick control
        if JOYSTICK_AVAILABLE == 1:
            link = txfer.SerialTransfer(JOYSTICK_SERIALPORT)
            link.open()
            rospy.sleep(2) # allow some time for the Arduino to completely reset

        # Start mouse control
        if MOUSE_AVAILABLE == 1:
            mouse_object = MouseClass()


        while not rospy.is_shutdown():
            signal.signal(signal.SIGINT, quit)
            signal.signal(signal.SIGTERM, quit)

            ### Convert joystick to 4 cable displacements

            ## Get joystick input

            if JOYSTICK_AVAILABLE == 1:
                # Send a float (purely for handshaking, value has no importance)
                float_ = 1.0
                float_size = link.tx_obj(float_, 0)
                link.send(float_size)
                
                # Wait for a response and report any errors while receiving packets
                while not link.available():
                    if link.status < 0:
                        if link.status == txfer.CRC_ERROR:
                            print('ERROR: CRC_ERROR')
                        elif link.status == txfer.PAYLOAD_ERROR:
                            print('ERROR: PAYLOAD_ERROR')
                        elif link.status == txfer.STOP_BYTE_ERROR:
                            print('ERROR: STOP_BYTE_ERROoffset_angleR')
                        else:
                            print('ERROR: {}'.format(link.status))
                
                # Parse response float
                v_x = link.rx_obj(obj_type=type(float_),
                                        obj_byte_size=float_size,
                                        start_pos=0)
                v_y = link.rx_obj(obj_type=type(float_),
                                        obj_byte_size=float_size,
                                        start_pos=float_size)           

                v_x = v_x / 500 + JOYSTICK_ZERO_XY_OFFSET[0] # just scales input to at most (-1,1)
                v_y = v_y / 500 + JOYSTICK_ZERO_XY_OFFSET[1] # just scales input to at most (-1,1)

                # avoid integrating really small numbers and overtensioning spool
                if abs(v_x) < 0.012:
                    v_x = 0

                if abs(v_y) < 0.012:
                    v_y = 0
                
                x = x + 0.005*v_x  #velocity control mode
                y = y + 0.005*v_y  #velocity control mode
                

            elif KEYBOARD_AVAILABLE:
                # (Get KEY to mimic joystick input for now)
                key = getKey(None)
                if key in keyboardControl: 
                    x += keyboardControl[key][1]
                    y += keyboardControl[key][0]
                elif key == ' ': 
                    #reset to neutral position
                    x = 0
                    y = 0
                elif key == 'q':
                    offset_theta += 0.01
                    print('angle offset = ', 180./np.pi*(offset_theta))
                elif key == 'e':
                    offset_theta -= 0.01
                    print('angle offset = ', 180./np.pi*(offset_theta))
                elif ord(key) == 27: 
                    break

            elif MOUSE_AVAILABLE:
                # update the mouse locations from the mouse_object class
                x = mouse_object.normalized_x
                y = mouse_object.normalized_y
                offset_theta = mouse_object.offset_angle

            else:
                #error
                print("No Input Device Chosen")
                quit()

            # clamps input into within unit circle to prevent over-turning the spools
            if x**2+y**2 >=1:
                x = x/np.sqrt(x**2+y**2) 
                y = y/np.sqrt(x**2+y**2)

            # re-orient the desired pose  as needed
            x_rotated = x*np.cos(offset_theta) - y*np.sin(offset_theta)
            y_rotated = x*np.sin(offset_theta) + y*np.cos(offset_theta)

            # set target positions
            target_position = CCTRL.inverse_kinematics_constant_curvature(x_rotated*3,y_rotated*3)

            # Command motors to target position
            CCTRL.setAllTargetPositions(target_position)
            CCTRL.desired_motor_state_pub.publish(CCTRL.desired_motor_state)

            #print('published position:', target_position) # tendon displacement
            #print('x,y = ',x_rotated,y_rotated)

            # a sleep statement to have code check every 0.01 seconds for new keyboard command
            rospy.sleep(0.05)


    except rospy.ROSInterruptException:
        pass
