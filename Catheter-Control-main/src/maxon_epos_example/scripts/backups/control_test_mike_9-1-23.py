#!/usr/bin/python3
# Last Edited: 13Jul2023 by Mike

import rospy
from pySerialTransfer import pySerialTransfer as txfer
import numpy as np
import sys, tty, termios, signal, select
from maxon_epos_msgs.msg import MotorState, MotorStates
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
import tf
import copy
from pynput import mouse
from statemachine import StateMachine, State
from statemachine.contrib.diagram import DotGraphMachine
from PIL import Image

######################################## SETTINGS DEFINITIONS ############################################################################

# root directory for project
ROOT_DIR = '/home/arclab/catkin_ws/src/Catheter-Control'

# kinematic parameters of the catheter
MOTOR_NUM = 4
TENSION_DIR = [-1,1,-1,1]                   # this converts the spooling directions so that +ve on setAllTargetPositions is tension, -ve is release
KEYBOARD_AVAILABLE = 0                      # SET THIS TO 1 if you are using Keyboard, otherwise 0
MOUSE_AVAILABLE = 0                         # SET THIS TO 1 if you are using Mouse, otherwise 0
XBOX_AVAILABLE = 1                          # SET THIS TO 1 if you are using Xbox Controller, otherwise 0

# Calibration File Saver/Opener Parameters
LOAD_EXISTING_CALIBRATION = 0               # SET TO 1 if calibration exists.
GET_NEW_CALIBRATION_DATA =  1                # SET TO 1 if you want to Get new calibration data. Otherwise it will search  for calibration data.
CATHETER_ID = 'xxAL04302303'                  #  identification # for a given catheter; format: (initials of assembler)(MM)(DD)(YY)(unit ## made that day)
CALIBRATION_ID = '25Jun2023_MP2'            # date of calibration file to be saved/read

# Bindings that map a keyboard press to an individual motor directional turn (for 4 motors there are 8 keyboard presses)
keyboardControl = {
    'w': [0.05, 0],
    'a': [0, 0.05],
    's': [-0.05, 0],
    'd': [0, -0.05]
}

# Preopen and allocate memory for the mode images 
ModeImage_normal    = Image.open(ROOT_DIR+'/resources/Images/Control_Mode_Images/NormalMode.jpg')
ModeImage_scaleup   = Image.open(ROOT_DIR+'/resources/Images/Control_Mode_Images/ScaleUpMode.jpg')
ModeImage_scaledown = Image.open(ROOT_DIR+'/resources/Images/Control_Mode_Images/ScaleDownMode.jpg')
ModeImage_hold      = Image.open(ROOT_DIR+'/resources/Images/Control_Mode_Images/PositionHoldMode.jpg')

##################################### CLASS DEFINITIONS ############################################################################

# State Machine Class
# makes it easier to manage which state/mode the robot is in
class CatheterStateMachine(StateMachine):
    
    # States are defined here    
    Start_Up            = State("Start Up Mode",initial=True)
    Early_Exit          = State("Early Exit From Program",final=True)
    Calibration_Mode    = State("Calibration Mode")
    User_Selection_Mode = State("User Selection Menu")
    Zero_Robot          = State("Zero Robot Mode")
    Neutral_Robot       = State("Neutral Robot Mode")
    Normal_Mode         = State("Normal Control Mode")
    Position_Hold_Mode  = State("Position Hold Mode")
    ScaleDown_Mode      = State("Scaled Down Control Mode")
    ScaleUp_Mode        = State("Scaled Up Control Mode")
    Terminate_Program   = State("Terminate Program",final=True)

    # Events are defined here
    zero_event              = User_Selection_Mode.to(Zero_Robot)
    neutral_event           = User_Selection_Mode.to(Neutral_Robot)
    start_event             = User_Selection_Mode.to(Normal_Mode)
    switch2scaledown        = (     Normal_Mode.to(ScaleDown_Mode) |
                                 ScaleDown_Mode.to(Normal_Mode) |
                             Position_Hold_Mode.to(ScaleDown_Mode) )
    switch2scaleup          = (     Normal_Mode.to(ScaleUp_Mode) |
                                   ScaleUp_Mode.to(Normal_Mode) |
                             Position_Hold_Mode.to(ScaleUp_Mode) )
    switch2hold             = (       Normal_Mode.to(Position_Hold_Mode) |
                               Position_Hold_Mode.to(Normal_Mode) |
                                     ScaleUp_Mode.to(Position_Hold_Mode) | 
                                   ScaleDown_Mode.to(Position_Hold_Mode) )
    termination_event       = (User_Selection_Mode.to(Terminate_Program) |
                                       Normal_Mode.to(Terminate_Program) |
                                Position_Hold_Mode.to(Terminate_Program) |
                                    ScaleDown_Mode.to(Terminate_Program) |
                                      ScaleUp_Mode.to(Terminate_Program) )
    early_exit_event        = Calibration_Mode.to(Early_Exit)
    move_past_start_up      = (Start_Up.to(User_Selection_Mode,cond="check_for_calibration") |
                               Start_Up.to(Calibration_Mode,unless="check_for_calibration") )
    calibration_complete    = Calibration_Mode.to(User_Selection_Mode)
    back2menu               = (Position_Hold_Mode.to(User_Selection_Mode) |
                                    Neutral_Robot.to(User_Selection_Mode) |
                                       Zero_Robot.to(User_Selection_Mode) |
                                      Normal_Mode.to(User_Selection_Mode) |
                                   ScaleDown_Mode.to(User_Selection_Mode) |
                                     ScaleUp_Mode.to(User_Selection_Mode))
    
    # Here we define functions that run before / after a transition 
    #    AND / OR    functions that run upon entering / exiting a state
    def check_for_calibration(self):
        return True if LOAD_EXISTING_CALIBRATION==1 else False
    
    def on_enter_Start_Up(self):
        print('Entering',self.current_state.name)
        print('~ ~ ~ Catheter Control Program Has Begun! ~ ~ ~')
        if MOUSE_AVAILABLE == 1:
            print('        Mouse Control Mode Enabled')
        elif KEYBOARD_AVAILABLE == 1:
            print('        Keyboard Control Mode Enabled')
        elif XBOX_AVAILABLE == 1:
            print('        Xbox Controller Mode Enabled')
            print('        Press the [Start] button on the controller to continue')
            CCTRL.pause_for_user(0)

    def on_exit_Start_Up(self):
        print('Exiting',self.current_state.name)
        global calibration_table, tension_offsets, tension_slopes, detension_slopes
        calibration_table   = CCTRL.workspaceCalibration()
        tension_offsets     = calibration_table[0,:]
        tension_slopes      = calibration_table[2,:]
        detension_slopes    = calibration_table[3,:]

    def on_enter_User_Selection_Mode(self):
        print('Entering',self.current_state.name)
        print('You can choose the next step from the following options:')
        print('   [z]   : Zero the robot relative to initial bootup position')
        print('   [n]   : Move the robot to tensioned neutral position (uses the loaded calibration)')
        print('   [y]   : Exit this menu and proceed to robot control')
        print('   [Esc] : Terminate the program')
        
    def on_enter_Early_Exit(self):
        print('Entering',self.current_state.name)

    def on_enter_Calibration_Mode(self):
        print('Entering',self.current_state.name)

    def on_enter_Zero_Robot(self):
        print('Entering',self.current_state.name)

    def on_enter_Neutral_Robot(self):
        print('Entering',self.current_state.name)

    def on_enter_Normal_Mode(self):
        print('Entering',self.current_state.name)
        ModeImage_normal.show()

    def on_enter_Position_Hold_Mode(self):
        print('Entering',self.current_state.name)
        ModeImage_hold.show()

    def on_enter_ScaleDown_Mode(self):
        print('Entering',self.current_state.name)
        ModeImage_scaledown.show()

    def on_enter_ScaleUp_Mode(self):
        print('Entering',self.current_state.name)
        ModeImage_scaleup.show()

    def on_enter_Terminate_Program(self):
        print('Entering',self.current_state.name)




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

        self.if_back_to_neutral_sub = rospy.Subscriber('/if_back_to_neutral', Bool, self.callbackIfBackToNeutral, queue_size=1)
        self.IF_BACK_TO_NEUTRAL     = False

        self.joystick_sub     = rospy.Subscriber('/joy',Joy,self.callbackXboxController)
        self.xbuts_oldstate   = self.xbuts  = self.xbuts_flags  = [0] * 15
        self.xaxes_oldstate   = self.xaxes  = [0] * 8
        
    def callbackIfBackToNeutral(self, msg):
        self.IF_BACK_TO_NEUTRAL = msg.data

    def callbackXboxController(self, msg):
        self.xbuts_oldstate = self.xbuts 
        self.xaxes_oldstate = self.xaxes 
        self.xbuts          = msg.buttons
        self.xaxes          = msg.axes
        subtract_step       = np.subtract(self.xbuts,self.xbuts_oldstate)
        replace_neg_to_zero = [0 if i < 0 else i for i in subtract_step]
        self.xbuts_flags    = np.add(self.xbuts_flags,replace_neg_to_zero)
    
    def setAllTargetPositions(self, target_position):
        self.desired_motor_state.header.stamp = rospy.Time.now()
        self.desired_motor_state.states[0].position = TENSION_DIR[0] * (target_position[0]+self.spool_zero_offset[0]) * 2 * np.pi 
        self.desired_motor_state.states[1].position = TENSION_DIR[1] * (target_position[1]+self.spool_zero_offset[1]) * 2 * np.pi 
        self.desired_motor_state.states[2].position = TENSION_DIR[2] * (target_position[2]+self.spool_zero_offset[2]) * 2 * np.pi 
        self.desired_motor_state.states[3].position = TENSION_DIR[3] * (target_position[3]+self.spool_zero_offset[3]) * 2 * np.pi 

    def pause_for_user(self,pause_time):
        if pause_time>0:
            # pause for user for finite time span
            print(' Pausing Point enabled. Press [w] to proceed.')
            while(getKey(0.01)!='w'): pass 
            print('   [w] was pressed. Resuming in '+str(pause_time)+' seconds.')
            return
        else: 
            # wait for a 'Start' button press indefinitely without printout
            if XBOX_AVAILABLE:
                while(self.xbuts[7]==0):
                    rospy.sleep(0.002)
            else:
                while(getKey(None)!='w'):
                    rospy.sleep(0.002)


    def loadDataForCalculatingCalibration(self, filename):
        # attempt to load data file
        data_array = np.load(filename)
        
        # set up data structure
        motor_positions_dict        = {}
        workspace_angles = np.unique(list(data_array[:, 1]))
        for motor in range(MOTOR_NUM):
            motor_positions_dict[motor] = {'tension_positions':{} , 'detension_positions':{}}
            for angle in workspace_angles:
                    motor_positions_dict[motor]['tension_positions'][angle] = []
                    motor_positions_dict[motor]['detension_positions'][angle] = []

        for row in data_array:
            #attempt number  = row[0]
            angle = row[1]
            tension_wire = round(row[2])
            position_tension_wire = row[3]
            #delta_tw = row[4]
            detension_wire = round(row[5])
            position_detension_wire = row[6]
            #delta_dw = row[7]
            motor_positions_dict[tension_wire]['tension_positions'][angle].append(position_tension_wire)
            motor_positions_dict[detension_wire]['detension_positions'][angle].append(position_detension_wire)
        
        print('Finished Loading Calibration Data. Press [w] or START to continue...')
        rospy.sleep(1)
        self.pause_for_user(0)
        return motor_positions_dict
    
    def workspaceCalibration(self):
        
        # Check if calibration is necessary
        # Condition 1: calibration isavailable and you  wish to load the precomputed calibration
        if LOAD_EXISTING_CALIBRATION == 1:
            calibration_file_name = ROOT_DIR+'/resources/CalibrationData/'+ CATHETER_ID+'_'+CALIBRATION_ID+'_WorkspaceSlopeCalibration'+'.npy'
            calibration_table = np.load(calibration_file_name)
            print('! ! ! - An existing calibration file has been found - ! ! !')
            print('Found calibration table: ',calibration_table)
            print('The robot will move to calibrated neutral position')
            print("* * * Proceed by pressing the 'w' key or [Start] button * * *")            
            rospy.sleep(0.1)
            self.pause_for_user(0)
            
            target_position = calibration_table[0,:]
            CCTRL.setAllTargetPositions(target_position)
            CCTRL.desired_motor_state_pub.publish(CCTRL.desired_motor_state)  
            rospy.sleep(1)
            
            print("Tensioning Complete.")
            return calibration_table

        # Condition 2: You wish to gather calibration data
        motor_positions_dict        = {}
        if GET_NEW_CALIBRATION_DATA == 1:
            print('No existing calibration was provided -')
            print(' Let us make a new workspace calibration! :) ')
            print("Before tensioning, the robot will return to zero position (set when motors were powered on).")
            print("* * * Proceed by pressing the 'w' key or [Start] * * *")
            self.pause_for_user(0)
            print("   Zeroing now.")
            target_position = np.zeros(4)
            CCTRL.setAllTargetPositions(target_position)
            CCTRL.desired_motor_state_pub.publish(CCTRL.desired_motor_state)    
            rospy.sleep(1)
            
            print("Tensioning sequence is next.")
            print("* * * Proceed by pressing the 'w' key * * *")
            self.pause_for_user(0)
            print("   Tensioning sequence has begun.")
            print("   Press 'p' to increase tension. Press 'm' to decrease tension.")
            print("   Once the catheter has reached the requested position, proceed to the next step by pressing the 'w' key")

            num_attempts                = 2
            data_capture                = []                           
            cali_release_step           = 0.05                          # release step
            displacement_limit          = 0.5                           # in rotations
            step_size                   = 0.005                         # in rotations
            workspace_angles            = np.array([0,90])              # the distal tip angles we want to calibrate at
            wire_pairs                  = [(0,1),(1,0),(2,3),(3,2)]     
            neutral_position            = np.zeros(4)
            
            # set up data structure
            for motor in range(MOTOR_NUM):
                motor_positions_dict[motor] = {'tension_positions':{} , 'detension_positions':{}}
                for angle in workspace_angles:
                    motor_positions_dict[motor]['tension_positions'][angle] = []
                    motor_positions_dict[motor]['detension_positions'][angle] = []

            # Begin calibration loop
            for attempt_num in range(num_attempts):
                print('Attempt '+str(attempt_num+1)+' has begun with step size of '+str(round(step_size,5)))
                
                # for a pair of tendon wires
                for current_pair in wire_pairs:
                    tension_wire = current_pair[0]
                    detension_wire = current_pair[1]
                    print("   Tension Wire: "+str(tension_wire)+"   Detension Wire: "+str(detension_wire))

                    # we sweep across these angles
                    for angle in workspace_angles:
                        print('The current target angle is '+str(angle)+' degrees')

                        # if we have more than one sample of this setting, go back to the previous setting 
                        if attempt_num > 0:
                            #calculate average tension from pevious trials, and set a small step back in tension.
                            target_position[tension_wire]   = sum(motor_positions_dict[tension_wire]['tension_positions'][angle])/attempt_num - cali_release_step
                            target_position[detension_wire] = sum(motor_positions_dict[detension_wire]['detension_positions'][angle])/attempt_num - cali_release_step 
                            print('Moving to a position slightly untensioned from the average of previous positions')
                            print(target_position)

                            CCTRL.setAllTargetPositions(target_position)
                            CCTRL.desired_motor_state_pub.publish(CCTRL.desired_motor_state)    
                            rospy.sleep(0.1)

                        tension_start_angle = target_position[tension_wire]
                        detension_start_angle = target_position[detension_wire]

                        print('Begin keyboard inputs [p,m] for dominant tendon, [P, M] for non-dominant tendon, [w] to go to next].')
                        while(True): 
                            a = getKey(0.01)
                            if(a=='p' and -displacement_limit < target_position[tension_wire]+step_size < displacement_limit):
                                target_position[tension_wire] = target_position[tension_wire]+step_size
                            elif(a=='m' and -displacement_limit < target_position[tension_wire]-step_size < displacement_limit):
                                target_position[tension_wire] = target_position[tension_wire]-step_size
                            elif(a=='P' and -displacement_limit < target_position[detension_wire]+step_size < displacement_limit):
                                target_position[detension_wire] = target_position[detension_wire]+step_size
                            elif(a=='M' and -displacement_limit < target_position[detension_wire]-step_size < displacement_limit):
                                target_position[detension_wire] = target_position[detension_wire]-step_size
                            elif(a=='w'):
                                # recording of delta since starting the attempt, store values to calibration record
                                delta_on_tension_wire = target_position[tension_wire]-tension_start_angle
                                delta_on_detension_wire = target_position[detension_wire]-detension_start_angle
                                motor_positions_dict[tension_wire]['tension_positions'][angle].append(target_position[tension_wire])
                                motor_positions_dict[detension_wire]['detension_positions'][angle].append(target_position[detension_wire])
                                data_vector  = np.array([attempt_num+1,
                                                        angle,
                                                        tension_wire,
                                                        target_position[tension_wire],
                                                        delta_on_tension_wire,
                                                        detension_wire,
                                                        target_position[detension_wire],
                                                        delta_on_detension_wire])                                
                                data_capture.append(data_vector)

                                if angle == 0: #overwrite initialized neutral position with a measured value
                                    neutral_position[tension_wire] = target_position[tension_wire] - cali_release_step
                                    neutral_position[detension_wire] = target_position[detension_wire] - cali_release_step
                                
                                print('Releasing tendon wires to a neutral position: ', neutral_position)
                                CCTRL.setAllTargetPositions(neutral_position)
                                CCTRL.desired_motor_state_pub.publish(CCTRL.desired_motor_state)    
                                rospy.sleep(0.1)

                                target_position = np.copy(neutral_position)
                                print('[w] key pressed. Moving to next step.\n')
                                break 

                            # set the target position for motors
                            self.setAllTargetPositions(target_position)
                            CCTRL.desired_motor_state_pub.publish(CCTRL.desired_motor_state) 

                        rospy.sleep(0.1)  

            data_capture_file_name = ROOT_DIR+'/resources/CalibrationData/'+ CATHETER_ID+'_'+CALIBRATION_ID+'_SimpleWorkspaceDataCapture'
            np.save(data_capture_file_name,np.asarray(data_capture))
            np.savetxt(str(data_capture_file_name+'.csv'),np.asarray(data_capture),delimiter=",")
            print('Data Collection done. The captured data has been saved as '+data_capture_file_name+'.npy & ~.csv')

        else:
            #Calibration data exists
            calibrationdata_file_name = CATHETER_ID+'_'+CALIBRATION_ID+'_SimpleWorkspaceDataCapture'+'.npy'
            print('Calibration  Data Exists. Loading '+calibrationdata_file_name)
            motor_positions_dict = self.loadDataForCalculatingCalibration(calibrationdata_file_name)        

        ##########################
        ## Solve for Calibration  
        ##########################          

        # Find the Neutral Position
        print('Starting to Solve for Calibration.')
        self.pause_for_user(0)
        print('Finding lumped tension & detension neutral position for each wire')
        lumped_neutral = np.zeros(MOTOR_NUM)
        for motor in range(MOTOR_NUM):
            tension_positions = motor_positions_dict[motor]['tension_positions'][0]
            detension_positions = motor_positions_dict[motor]['detension_positions'][0]
            lumped_neutral[motor] = (sum(tension_positions)+sum(detension_positions))/(len(tension_positions)+len(detension_positions))

        print('Moving to calibrated neutral position: ', lumped_neutral)
        CCTRL.setAllTargetPositions(lumped_neutral)
        CCTRL.desired_motor_state_pub.publish(CCTRL.desired_motor_state)
        rospy.sleep(2)
        print('Robot has completed moving to the calibrated neutral position')
        pause = 2
        self.pause_for_user(pause)
        rospy.sleep(pause)

        print('.. Now finding TENSIONING slope for each wire ..')
        tension_slopes = np.zeros(MOTOR_NUM)

        # get the different angles attempted
        workspace_angles = np.unique(list(motor_positions_dict[motor]['tension_positions'].keys()))
        self.pause_for_user(0)
        for motor in range(MOTOR_NUM):
            print('   Motor ',motor)
            # Solve y=mx+c for m using least squares. In matrix form, y=Ap where A=[x 1] and p=[m;c]
            x = np.array([0])
            y = np.array(lumped_neutral[motor])
            for angle in motor_positions_dict[motor]['tension_positions']:
                # we will skip neutral (angle = 0) since that is calculated using both tension/detension values
                if angle != 0:
                    sampled_positions = motor_positions_dict[motor]['tension_positions'][angle]
                    y = np.append(y,np.array(sampled_positions))
                    x = np.append(x,np.ones(len(sampled_positions))*angle)
                
            #solve least squared fit
            A = np.vstack([x, np.ones(len(x))]).T
            m, c = np.linalg.lstsq(A, y, rcond=None)[0]
            tension_slopes[motor] = m
            print('computed tension slope:',m)
        
        rospy.sleep(1)
        
        print('. . .  Now finding detensioning slope for each wire  . . .')
        detension_slopes = np.zeros(MOTOR_NUM)
        for motor in range(MOTOR_NUM):
            print('   Motor ',motor)
            # Solve y=mx+c for m using least squares. In matrix form, y=Ap where A=[x 1] and p=[m;c]
            x = np.array([0])
            y = np.array(lumped_neutral[motor])
            for angle in motor_positions_dict[motor]['detension_positions']:
                # we will skip neutral (angle = 0) since that is calculated using both tension/detension values
                if angle != 0:
                    sampled_positions = motor_positions_dict[motor]['detension_positions'][angle]
                    y = np.append(y,np.array(sampled_positions))
                    x = np.append(x,np.ones(len(sampled_positions))*-angle)
                
            #solve least squared fit
            A = np.vstack([x, np.ones(len(x))]).T
            # print(A)
            m, c = np.linalg.lstsq(A, y, rcond=None)[0]
            detension_slopes[motor] = m

        pause = 1
        self.pause_for_user(pause)
        rospy.sleep(pause)

        print('determining the maximum tension/detension positions for each motor')
        max_tension = np.zeros(MOTOR_NUM)
        min_tension = np.zeros(MOTOR_NUM)
        for motor in range(MOTOR_NUM):
            max_tension[motor] = max(motor_positions_dict[motor]['tension_positions'][90])
            min_tension[motor] = min(motor_positions_dict[motor]['detension_positions'][90])
        print('The maximum/minimum motor values for the workspace have been set as:', max_tension, min_tension)
        print('This is done to prevent catheter damage')

        pause = 1
        self.pause_for_user(pause)
        rospy.sleep(pause)

        print('Saving calibrated slopes and maximum/minimum values in the workspace')
        calibration_file_name = ROOT_DIR+'/resources/CalibrationData/'+CATHETER_ID+'_'+CALIBRATION_ID+'_WorkspaceSlopeCalibration'+'.npy'
        calibration_table = np.vstack([lumped_neutral,lumped_neutral,tension_slopes,detension_slopes,max_tension,min_tension])
        np.save(calibration_file_name,calibration_table)
        np.savetxt(str(calibration_file_name+'.csv'),calibration_table,delimiter=",")
        
        print('The resolved calibration has been saved as '+calibration_file_name+'.npy & ~.csv')
        print('workspace calibration procedure is complete. Time to restart [Ctrl-C]?')
        rospy.sleep(1)
        pause = 1
        self.pause_for_user(pause)
        rospy.sleep(pause)
        return calibration_table

    def inverse_kinematics(self,x,y,max_tip_angle,tension_slopes,detension_slopes):
        # -1 < x,y < 1 represent  locations inside a unit circle, where r = 1 is the max_tip_angle
        # We consider a potential difference in releasing and tensioning tension despite it ideally being the same.

        # Code is written with the following layout for the wires:
        #  
        #          -y-axis      
        #      Q3     2     Q4
        #             |
        #       0 ----|---- 1   +x-axis
        #             |
        #      Q2     3     Q1
        #          +y-axis

        # using constant curvature model, 
        # (1) account for change in distyance d from tendons when targeting angle in-between
        # (2) next multiply, by the maximum tip angle.
        if (x**2 + y**2)-1 > 0.01:
            print(x**2 + y**2)
            print('Invalid Target for IK solver. (x,y) must be in a unit circle')
            return -1, np.array([0,0,0,0])
        
        offset_phi = np.pi/4                       # offset bending plane defined by x,y to align with diagonal tendons
        phi = np.arctan2(y,x) + offset_phi         # find the bending plane angle. Add an offset here
        theta = max_tip_angle*np.sqrt(x**2+y**2)   # find the angle of curvature
        theta_x = np.cos(phi)*theta                # find projected plane-corrected bending angle
        theta_y = np.sin(phi)*theta                # find projected plane-corrected bending angle

        # convert angular displacement in x to cable displacement on 2 cables
        if theta_x > 0:
            # the robot is moving in direction of spool1 (which is tensioning), so it should tension faster than spool2
            spool0_targ_angle = detension_slopes[0] * theta_x 
            spool1_targ_angle = tension_slopes[1] * theta_x 
        else: # then theta_x <= 0
            # the robot is moving in direction of spool2 (which is tensioning), so it should tension faster than spool1
            spool0_targ_angle = tension_slopes[0] * -theta_x 
            spool1_targ_angle = detension_slopes[1] * -theta_x 

        # convert angular dispalcement in y to cable displacement on other 2 cables
        if theta_y > 0:
            # the robot is moving in direction of spool3 (which is tensioning), so it should tension faster than spool4
            spool2_targ_angle = detension_slopes[2] * theta_y 
            spool3_targ_angle = tension_slopes[3] * theta_y 
        else: # then theta_y <= 0
            # the robot is moving in direction of spool4 (which is tensioning), so it should tension faster than spool3
            spool2_targ_angle = tension_slopes[2] * -theta_y 
            spool3_targ_angle = detension_slopes[3] * -theta_y 

        return 1, np.array([spool0_targ_angle, spool1_targ_angle, spool2_targ_angle, spool3_targ_angle])

    def selection_menu(self,calibration_table):
        while(True): 
            key = getKey(None)
            if(key=='z'):
                ModeMachine.zero_event()
                print('Moving robot to zero position was selected.')
                print('Press [z] again to begin.')
                while getKey(0.01)!='z': pass 
                target_position = np.zeros(4)
                self.setAllTargetPositions(target_position)
                self.desired_motor_state_pub.publish(self.desired_motor_state)    
                rospy.sleep(1)
                print('Robot moved to Zero')
                rospy.sleep(1)
                ModeMachine.back2menu()
            elif(key=='n'):
                ModeMachine.neutral_event()
                print('Moving robot to tensioned neutral position was selected.')
                print('Press [n] again to begin.')
                while getKey(0.01)!='n': pass 
                tension_offsets = calibration_table[0,:]
                self.setAllTargetPositions(tension_offsets)
                self.desired_motor_state_pub.publish(self.desired_motor_state)    
                rospy.sleep(1)
                print('Robot moved to Neutral')
                rospy.sleep(1)
                ModeMachine.back2menu()
            elif(key=='y'):
                print('Exiting menu and proceeding to robot control.')
                ModeMachine.start_event()
                break
            elif ord(key) == 27:
                ModeMachine.termination_event()
                quit()


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
        
        # Defines the center of the mouse circle, which is relative to the screen resolution
        self.offset_x = 1920/2
        self.offset_y = 1080/2
        
        # Create a mover object that keep mouse centered on screen
        # so that it never reaches a wall
        self.mover = mouse.Controller()
        self.mover.position = (self.offset_x,self.offset_y) #center the mouse
        
        # Initialize other variables
        self.offset_angle = 0
        self.normalized_x = 0
        self.normalized_y = 0

        # Create mouse callback
        self.listener = mouse.Listener(on_move=self.on_move,
                                       on_click=self.on_click,
                                       on_scroll=self.on_scroll)

        self.listener.start()
        print("Initializing mouse tracker")

    def on_move(self,x,y):
        # The following are measured in pixels
        # Note that (x,y)=(0,0) on the screen is at the top left corner
        # Relative to a circle centered at the screen center, quadrants are flipped across x-axis
        Dx = x-self.offset_x
        Dy = y-self.offset_y
        Dist = np.sqrt(Dx**2 + Dy**2)
        # print('true cursor position: ',x,y)
        # print('   Dx, Dy: ',Dx,Dy)
        # print('   distance: ',Dist)

        # If cursor is trying to move beyond a unit circle of 500 pixels
        if Dist > 500: 
            # we move the cursor to a position inside the circle 
            x = Dx/Dist*498 + self.offset_x
            y = Dy/Dist*498 + self.offset_y
            self.mover.position = (x,y)
            
            # Effectively creates a percentage scale of maximum movement
            Dx = x-self.offset_x
            Dy = y-self.offset_y
            self.normalized_x = Dx/495
            self.normalized_y = Dy/495
            
        else:
            # Effectively creates a percentage scale of maximum movement
            self.normalized_x = Dx/500
            self.normalized_y = Dy/500
        
    def on_click(self,x, y, button, pressed):
        print('{0} at {1}'.format('Pressed' if pressed else 'Released',(x, y)))
        if not pressed:
            # Stop listener
            
            return True # change to false to stop listening

    def on_scroll(self, x, y, dx, dy):
        # dy should be a +1/-1 ?
        self.offset_angle += 5*dy/180*np.pi # radians equivalent of 5 deg steps
        print('Angle Offset = ', round(self.offset_angle/np.pi*180,1),' deg')
#        print('Scrolled {0} at {1}'.format('down' if dy < 0 else 'up', (x, y)))

######################################## MAIN PROGRAM ############################################################################

if __name__ == "__main__": 
    settings = termios.tcgetattr(sys.stdin)

    # Instantiate a Catheter object
    CCTRL = CatheterController()

    # Instantiate a State Machine object
    ModeMachine = CatheterStateMachine()
    
    graph = DotGraphMachine(ModeMachine)
    dot = graph()
    dot.write_png("Catheter_State_Machine_ModeCheck1.png")

    ModeMachine.move_past_start_up()
    CCTRL.selection_menu(calibration_table)
        
    try: # Start Control Loop

        print("The robot is now active.")

        # Things to set before the loop
        offset_theta = 0    # this accounts for angle offsets between joystick and output of the catheter
        x = 0               # assume x,y of a unit circle
        y = 0               # assume x,y of a unit circle
        

        # Start mouse control
        if MOUSE_AVAILABLE == 1:
            mouse_object = MouseClass()
        if XBOX_AVAILABLE == 1: 
            xbuts_flags_ref     = CCTRL.xbuts_flags
            xbuts_flags_update  = np.subtract(CCTRL.xbuts_flags,xbuts_flags_ref)
            xbuts_flags_update_old = np.copy(xbuts_flags_update)
            
        while not rospy.is_shutdown():
            signal.signal(signal.SIGINT, quit)
            signal.signal(signal.SIGTERM, quit)

            if KEYBOARD_AVAILABLE:
                # (Get KEY to mimic joystick input)
                key = getKey(None)
                if key in keyboardControl: 
                    x += keyboardControl[key][1]
                    y += keyboardControl[key][0]
                elif key == ' ':
                    # reset to neutral position, established after tensioning
                    print('[Space bar] pressed. Resetting to neutral position.')
                    x = 0
                    y = 0
                elif key == 'q':
                    offset_theta += 5*np.pi/180 # radians equivalent of 5 deg steps
                    print('Angle Offset = ', round(180./np.pi*(offset_theta),1),' deg')
                elif key == 'e':
                    offset_theta -= 5*np.pi/180 # radians equivalent of 5 deg steps
                    print('Angle Offset = ', round(180./np.pi*(offset_theta),1),' deg')
                elif ord(key) == 27:
                    break

            elif MOUSE_AVAILABLE:
                if CCTRL.IF_BACK_TO_NEUTRAL:
                    x = 0
                    y = 0
                    mouse_object.normalized_x = 0
                    mouse_object.normalized_y = 0
                    mouse_object.mover.position = (mouse_object.offset_x,mouse_object.offset_y) #center the mouse
                else:
                    # Update the mouse locations from the mouse_object class
                    # These x & y values are %s of maximum allowable movement
                    x = mouse_object.normalized_x
                    y = mouse_object.normalized_y
                    offset_theta = mouse_object.offset_angle
                    max_tip_angle = 90

            elif XBOX_AVAILABLE:

                xbuts_flags_update = np.subtract(CCTRL.xbuts_flags,xbuts_flags_ref)

                x = CCTRL.xaxes[0]
                y = CCTRL.xaxes[1]

                # Unwarp and clamp Joystick (the joystick x,y maps to more like a box than a circle). 
                # Use calibrated warping which will clamp values to inside the unit circle
                joy_theta = np.arctan2(y,x)
                joy_radius = np.sqrt(x**2 + y**2)
                joy_scaling_factor = 0.4659*np.abs(np.sin(joy_theta))+0.4549*np.abs(np.cos(joy_theta))+0.4989
                r_clamped = np.min([1,joy_radius/joy_scaling_factor]) 
                x = r_clamped*np.cos(joy_theta)
                y = r_clamped*np.sin(joy_theta)

                if CCTRL.xbuts[1]==1 and CCTRL.xbuts[6]==1:
                    ModeMachine.termination_event()
                    quit()

                if CCTRL.xbuts[3]==1 and (xbuts_flags_update[13]-xbuts_flags_update_old[13]==1):
                    offset_theta += 5/180*np.pi
                    print('Angle Offset = ', round(offset_theta/np.pi*180,1),' deg')
                elif CCTRL.xbuts[3]==1 and (xbuts_flags_update[14]-xbuts_flags_update_old[14]==1):
                    offset_theta -= 5/180*np.pi
                    print('Angle Offset = ', round(offset_theta/np.pi*180,1),' deg')

                if ModeMachine.Normal_Mode.is_active:
                    max_tip_angle = 90

                    if xbuts_flags_update[2]%2==1:
                        # if blue [X] button flag is raised
                        ModeMachine.switch2scaledown()
                    elif xbuts_flags_update[1]%2==1:
                        # if red [B] button flag is raised
                        ModeMachine.switch2scaleup()
                    elif xbuts_flags_update[4]%2==1:
                        # if left bumper [LB] button flag is raised
                        x_hold = x
                        y_hold = y
                        xbuts_flags_onhold = np.copy(xbuts_flags_update)
                        previous_state_hold = ModeMachine.current_state
                        ModeMachine.switch2hold()

                elif ModeMachine.ScaleDown_Mode.is_active:
                    max_tip_angle = 60

                    if xbuts_flags_update[2]%2==0:
                        # if blue [X] button flag is raised
                        xbuts_flags_ref = np.add(xbuts_flags_update,xbuts_flags_ref)
                        ModeMachine.switch2scaledown()
                    elif xbuts_flags_update[4]%2==1:
                        # if left bumper [LB] button flag is raised
                        x_hold = x
                        y_hold = y
                        xbuts_flags_onhold = np.copy(xbuts_flags_update)
                        previous_state_hold = ModeMachine.current_state
                        ModeMachine.switch2hold()

                elif ModeMachine.ScaleUp_Mode.is_active:
                    max_tip_angle = 145

                    if xbuts_flags_update[1]%2==0:
                        # if red [B] button flag is raised
                        xbuts_flags_ref = np.add(xbuts_flags_update,xbuts_flags_ref)
                        ModeMachine.switch2scaleup()
                    elif xbuts_flags_update[4]%2==1:
                        # if left bumper [LB] button flag is raised
                        x_hold = x
                        y_hold = y
                        xbuts_flags_onhold = np.copy(xbuts_flags_update)
                        previous_state_hold = ModeMachine.current_state
                        ModeMachine.switch2hold()

                elif ModeMachine.Position_Hold_Mode.is_active:
                    x = x_hold
                    y = y_hold
                    if xbuts_flags_update[4]%2==0:
                        xbuts_flags_update = np.copy(xbuts_flags_onhold)
                        if ModeMachine.Normal_Mode == previous_state_hold:
                            ModeMachine.switch2hold()
                        elif ModeMachine.ScaleDown_Mode == previous_state_hold:
                            ModeMachine.switch2scaledown()
                        elif ModeMachine.ScaleUp_Mode == previous_state_hold:
                            ModeMachine.switch2scaleup()
                
                xbuts_flags_update_old = np.copy(xbuts_flags_update)

            else:
                #error
                print("No Input Device Chosen")
                quit()

            # Re-orient the desired pose as needed, note these are coordinates in a circular space
            x_rotated_circle = x*np.cos(offset_theta) - y*np.sin(offset_theta)
            y_rotated_circle = y*np.sin(offset_theta) + y*np.cos(offset_theta)

            # Set target positions
            found_ik, ik_solution = CCTRL.inverse_kinematics(x_rotated_circle,y_rotated_circle, 
                                                             max_tip_angle, tension_slopes,
                                                             detension_slopes)
            target_position = ik_solution+tension_offsets if found_ik==1 else 0
            # Command motors to target position
            CCTRL.setAllTargetPositions(target_position)
            CCTRL.desired_motor_state_pub.publish(CCTRL.desired_motor_state)

            # A sleep statement to have code check every X seconds for new command
            rospy.sleep(0.002)

    except rospy.ROSInterruptException:
        pass