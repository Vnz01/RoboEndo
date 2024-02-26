#!/usr/bin/python3
# Last Edited: 26Mar2023 by Alexander Luke

import rospy
from pySerialTransfer import pySerialTransfer as txfer
import numpy as np
import sys, tty, termios, signal, select
from maxon_epos_msgs.msg import MotorState, MotorStates
from mag_trakstar.msg import TrakstarMsg
from std_msgs.msg import Bool
import tf
from cc_model.msg import CableActuation
import copy
from pynput import mouse

######################################## SETTINGS DEFINITIONS ############################################################################

# kinematic parameters of the catheter
MOTOR_NUM = 4
TENSION_DIR = [-1,1,-1,1]                   # this converts the spooling directions so that +ve on setAllTargetPositions is tension, -ve is release
KEYBOARD_AVAILABLE = 0                      # SET THIS TO 1 if you are using Keyboard, otherwise 0
MOUSE_AVAILABLE = 1                         # SET THIS TO 1 if you are using Mouse, otherwise 0
JOYSTICK_SERIALPORT = '/dev/ttyACM0'        # Serial port for joystick 
JOYSTICK_ZERO_XY_OFFSET = [0.06,0.07]       # makes reading from joystick as close as possible to 0 when the joystick is not touched

# Calibration File Saver/Opener Parameters
EXISTING_CALIBRATION = 1                    # SET TO 1 if there is an existing file with calibration parameters you want to use, otherwise 0
CATHETER_ID = 'CR04222301'                  # identification # for a given catheter; format: (initials of assembler)(MM)(DD)(YY)(unit ## made that day)
CALIBRATION_ID = '24Apr2023_MP1'            # date of calibration file to be saved/read _ MotorPack #

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

        self.em_sesoe_raw_sub = rospy.Subscriber('/trakstar/raw_data', TrakstarMsg, self.callbackEMSensorRaw, queue_size=1)
        self.tf_br_EMbase_sensor = tf.TransformBroadcaster()

        self.if_back_to_neutral_sub = rospy.Subscriber('/if_back_to_neutral', Bool, self.callbackIfBackToNeutral, queue_size=1)
        self.IF_BACK_TO_NEUTRAL = False

    def callbackIfBackToNeutral(self, msg):
        self.IF_BACK_TO_NEUTRAL = msg.data

    def callbackEMSensorRaw(self, msg):
        em_sensor_raw = msg

        ### EM Sensor : ID 0
        id_num = 0
        emSensorID0_raw_tvec = em_sensor_raw.sensor_pose[id_num].position
        em_sensor_id0_quat = em_sensor_raw.sensor_pose[id_num].orientation
        self.broadTFEMBaseToSensor(emSensorID0_raw_tvec, em_sensor_id0_quat, 'EM_Sensor_ID0')

        ### EM Sensor : ID 1
        # emSensorID1_raw_tvec = em_sensor_raw.sensor_pose[1].position
        # em_sensor_id1_quat = em_sensor_raw.sensor_pose[1].orientation
        # self.broadTFEMBaseToSensor(emSensorID1_raw_tvec, em_sensor_id1_quat, 'EM_Sensor_ID1')
    
    def broadTFEMBaseToSensor(self, tvec, quat, id_string):    
        self.tf_br_EMbase_sensor.sendTransform((tvec.x, tvec.y, tvec.z), (quat.x, quat.y, quat.z, quat.w), 
                                                rospy.Time.now(), id_string, 'EM_Base')

    def setAllTargetPositions(self, target_position):
        self.desired_motor_state.header.stamp = rospy.Time.now()
        self.desired_motor_state.states[0].position = TENSION_DIR[0] * (target_position[0]+self.spool_zero_offset[0]) * 2 * np.pi 
        self.desired_motor_state.states[1].position = TENSION_DIR[1] * (target_position[1]+self.spool_zero_offset[1]) * 2 * np.pi 
        self.desired_motor_state.states[2].position = TENSION_DIR[2] * (target_position[2]+self.spool_zero_offset[2]) * 2 * np.pi 
        self.desired_motor_state.states[3].position = TENSION_DIR[3] * (target_position[3]+self.spool_zero_offset[3]) * 2 * np.pi 

    def pause_for_user(self,pause_time):
        if pause_time>0:
            # pause for user for finite time span
            print(' ')
            print(' Pausing Point enabled. Press [w] to proceed.')
            while(True): 
                a = getKey(0.01)
                if(a=='w'):
                    print('   [w] was pressed. Resuming in '+str(pause_time)+' seconds.')
                    return
        else: 
            # wait for a 'w' key press indefinitely without printout
            while(getKey(None)!='w'):
                rospy.sleep(0.02) 

    def workspaceCalibration(self):
        
        # Check if calibration is necessary
        if EXISTING_CALIBRATION == 1:
            calibration_file_name = CATHETER_ID+'_'+CALIBRATION_ID+'_WorkspaceNonlinearCalibration'+'.npy'
            # calibration_file_name = 'WorkspaceNonlinearCalibration_'+CALIBRATION_ID+'_'+CATHETER_ID+'.npy'
            calibration_table = np.load('/home/arclab/'+calibration_file_name)
            print('! ! ! - An existing calibration file has been found - ! ! !')
            print('Found calibration table: ',calibration_table)
            print(' ')
            print('The robot will move to calibrated neutral position')
            print("* * * Proceed by pressing the 'w' key * * *")

            self.pause_for_user(0)
            
            target_position = calibration_table[0,:]
            self.setAllTargetPositions(target_position)
            self.desired_motor_state_pub.publish(self.desired_motor_state)  
            rospy.sleep(1)

            print(" ")
            print("Tensioning Complete.")
        
        else: # Tensioning sequence
            print('- No existing calibration was provided -')
            print(' Let us make a new workspace calibration! :) ')

            print(" ")
            print("Before tensioning, the robot will return to zero position (set when motors were powered on).")
            print("* * * Proceed by pressing the 'w' key * * *")

            self.pause_for_user(0)

            print(" ")
            print("   Zeroing in 3 seconds...")
            rospy.sleep(1) 
            print("   Zeroing in 2 seconds...")
            rospy.sleep(1) 
            print("   Zeroing in 1 second ...")
            rospy.sleep(1) 
            print("   Zeroing now.")
            target_position = np.zeros(4)
            self.setAllTargetPositions(target_position)
            self.desired_motor_state_pub.publish(self.desired_motor_state)    
            rospy.sleep(1)

            print(" ")
            print("Tensioning sequence is next.")
            print("* * * Proceed by pressing the 'w' key * * *")
            
            self.pause_for_user(0)

            print(" ")
            print("   Tensioning sequence has begun.")
            print("   Press 'p' to increase tension. Press 'm' to decrease tension.")
            print("   Once the catheter has reached the requested position, proceed to the next step by pressing the 'w' key")

            num_attempts                = 2
            data_capture                = []                           
            cali_release_scale_factor   = 0.5
            displacement_limit          = 0.5                           # in rotations
            rotational_step_size        = 0.005                         # in rotations
            workspace_angles            = np.array([0,90])              # the distal tip angles we want to calibrate at
            wire_pairs                  = [(0,1),(1,0),(2,3),(3,2)]     
            neutral_position            = np.zeros(4)
            
            motor_positions_dict        = {}
            for motor in range(MOTOR_NUM):
                motor_positions_dict[motor] = {'tension_positions':{} , 'detension_positions':{}}
                for angle in workspace_angles:
                    motor_positions_dict[motor]['tension_positions'][angle] = []
                    motor_positions_dict[motor]['detension_positions'][angle] = []

            # Begin calibration loop
            for attempt_num in range(num_attempts):

                print(' ')
                step = rotational_step_size
                print('Attempt '+str(attempt_num+1)+' has begun with step size of '+str(round(step,5)))
                
                # for a pair of tendon wires
                for current_pair in wire_pairs:

                    tension_wire = current_pair[0]
                    detension_wire = current_pair[1]
                    print(' ')
                    print("   Tension Wire: "+str(tension_wire)+"   Detension Wire: "+str(detension_wire))

                    # we sweep across these angles
                    for angle in workspace_angles:
                        
                        print(' ')
                        print('The current target angle is '+str(angle)+' degrees')

                        if motor_positions_dict[motor]['tension_positions'][angle] != []:
                            
                            print(' ')
                            print('!! Well, would you look at that. We have been here before')
                            print(' ')
                            print('Last time we tried to tension Motor '+str(tension_wire)+' and detension Motor '+str(detension_wire)+' we achieved')
                            print(motor_positions_dict[tension_wire]['tension_positions'][angle])
                            print(motor_positions_dict[detension_wire]['detension_positions'][angle])
                            target_position[tension_wire]   = sum(motor_positions_dict[tension_wire]['tension_positions'][angle])/ \
                                                                len(motor_positions_dict[tension_wire]['tension_positions'][angle])* \
                                                                cali_release_scale_factor
                            target_position[detension_wire] = sum(motor_positions_dict[detension_wire]['detension_positions'][angle])/ \
                                                                len(motor_positions_dict[detension_wire]['detension_positions'][angle])* \
                                                                cali_release_scale_factor
                            print('Moving to a position close to the average of those previous positions')
                            print(target_position)

                            self.setAllTargetPositions(target_position)
                            self.desired_motor_state_pub.publish(self.desired_motor_state)    
                            rospy.sleep(0.1)
                            
                            print('Successfully moved!')

                        tw_start_angle = target_position[tension_wire]
                        dw_start_angle = target_position[detension_wire]
                        print('Tension wire start position: ',tw_start_angle,'      ;      Detension wire start position: ',dw_start_angle)

                        print(' ')
                        print('Begin keyboard inputs')

                        while(True): 

                            a = getKey(0.01)

                            if(a=='w'):

                                # recording of delta since starting the attempt, store values to calibration record
                                delta_tw = target_position[tension_wire]-tw_start_angle
                                delta_dw = target_position[detension_wire]-dw_start_angle
                                print('Tensioned Delta: '+str(round(delta_tw,6))+' of a rotation ; Detensioned Delta: '+str(round(delta_dw,6))+' of a rotation')

                                motor_positions_dict[tension_wire]['tension_positions'][angle].append(target_position[tension_wire])
                                motor_positions_dict[detension_wire]['detension_positions'][angle].append(target_position[detension_wire])

                                print('List of tension values: ',motor_positions_dict[tension_wire]['tension_positions'][angle])
                                print('List of detension values: ',motor_positions_dict[detension_wire]['detension_positions'][angle])

                                data_vector  = np.array([attempt_num+1,
                                                         angle,
                                                         tension_wire,
                                                         target_position[tension_wire],
                                                         delta_tw,
                                                         detension_wire,
                                                         target_position[detension_wire],
                                                         delta_dw])
                                print('New Data Vector: ',data_vector)
                                
                                data_capture.append(data_vector)
                                print('Full Data Capture so far:')
                                print(np.asarray(data_capture))

                                if angle == 0:
                                    neutral_position[tension_wire] = tw_start_angle + delta_tw*cali_release_scale_factor
                                    neutral_position[detension_wire] = dw_start_angle + delta_dw*cali_release_scale_factor

                                print(' ')
                                print('Releasing tendon wires to a neutral position ....')
                                # target_position[tension_wire] = delta_tw*cali_release_scale_factor
                                # target_position[detension_wire] = delta_dw*cali_release_scale_factor
                                print('New Target: ',neutral_position)
                                self.setAllTargetPositions(neutral_position)
                                self.desired_motor_state_pub.publish(self.desired_motor_state)    
                                rospy.sleep(0.1)

                                target_position = np.copy(neutral_position)
                                print('            [w] key pressed. Moving to next step.')
                                break 

                            if(a=='p' and -displacement_limit<=target_position[tension_wire]+step<=displacement_limit):

                                target_position[tension_wire] = target_position[tension_wire]+step
                                print('            + [lo p] key pressed. New Target: ',target_position)
                                self.setAllTargetPositions(target_position)
                                self.desired_motor_state_pub.publish(self.desired_motor_state) 

                            if(a=='m' and -displacement_limit<=target_position[tension_wire]-step<=displacement_limit):

                                target_position[tension_wire] = target_position[tension_wire]-step
                                print('            - [lo m] key pressed. New Target: ',target_position)
                                self.setAllTargetPositions(target_position)
                                self.desired_motor_state_pub.publish(self.desired_motor_state)

                            if(a=='P' and -displacement_limit<=target_position[detension_wire]+step<=displacement_limit):

                                target_position[detension_wire] = target_position[detension_wire]+step
                                print('            + [UP P] key pressed. New Target: ',target_position)
                                self.setAllTargetPositions(target_position)
                                self.desired_motor_state_pub.publish(self.desired_motor_state) 

                            if(a=='M' and -displacement_limit<=target_position[detension_wire]-step<=displacement_limit):

                                target_position[detension_wire] = target_position[detension_wire]-step
                                print('            - [UP M] key pressed. New Target: ',target_position)
                                self.setAllTargetPositions(target_position)
                                self.desired_motor_state_pub.publish(self.desired_motor_state)

                        rospy.sleep(0.1)  

            data_capture_file_name = CATHETER_ID+'_'+CALIBRATION_ID+'_SimpleWorkspaceDataCapture'
            np.save(data_capture_file_name,np.asarray(data_capture))
            np.savetxt(str(data_capture_file_name+'.csv'),np.asarray(data_capture),delimiter=",")
            print(' ')
            print('The captured data has been saved as '+data_capture_file_name+'.npy & ~.csv')
            rospy.sleep(5)

            print(' ')
            print('Working to find the true neutral position of the robot')
            neutral_positions = {}
            for motor in range(MOTOR_NUM):
                print('Tension values for Motor ',motor,' at 0 degrees:')
                neutral_positions[motor] = motor_positions_dict[motor]['tension_positions'][0]
                print(neutral_positions[motor])
                target_position[motor] = sum(neutral_positions[motor])/num_attempts
                print('Averaged neutral tension position: ',target_position[motor])

            print(' ')
            print('Moving to calibrated neutral position')
            print(target_position)

            self.setAllTargetPositions(target_position)
            self.desired_motor_state_pub.publish(self.desired_motor_state)
            rospy.sleep(5)
            print('Robot has completed moving to the calibrated neutral position')
            print(" ")
            print("Calibration Data Collection Process is now Complete.")

            rospy.sleep(5)
            print(' ')
            print('Finding lumped tension & detension neutral position for each wire')
            lumped_neutral = np.zeros(4)
            for motor in range(MOTOR_NUM):
                print('     - - - Motor '+str(motor)+' - - -')
                tension_positions = motor_positions_dict[motor]['tension_positions'][0]
                detension_positions = motor_positions_dict[motor]['detension_positions'][0]
                print('tension positions:',tension_positions)
                print('detension positions:',detension_positions)
                lumped_neutral[motor] = (sum(tension_positions)+sum(detension_positions))/(len(tension_positions)+len(detension_positions))
            print(' ')
            print(' The lumped neutral position for each respective motor:')
            print(lumped_neutral)

            pause = 5
            self.pause_for_user(pause)
            rospy.sleep(pause)

            print(' ')
            print('.. Now finding TENSIONING slope for each wire ..')
            tension_slopes = np.zeros(MOTOR_NUM)
            for motor in range(MOTOR_NUM):
                print('   Motor ',motor)
                # Solve y=mx+c for m using least squares. In matrix form, y=Ap where A=[x 1] and p=[m;c]
                y = list(motor_positions_dict[motor]['tension_positions'][90])
                y.insert(0,lumped_neutral[motor])
                x = np.ones(len(y))*90
                y = np.asarray(y)
                x[0] = 0
                print('x values:',x)
                print('y values:',y)
                print('A matrix:')
                A = np.vstack([x, np.ones(len(x))]).T
                print(A)
                m, c = np.linalg.lstsq(A, y, rcond=None)[0]
                print('lumped neutral position:',str(lumped_neutral[motor]))
                print('computed y-intercept:',str(c))
                tension_slopes[motor] = m
                print('computed tension slope:',m)
                if round(c,5)==round(lumped_neutral[motor],5):
                    print(' YAY! the least squares y-intercept and the lumped neutral position intercept are in fact the same!')
                    print('when rounded to the fifth decimal place')
                else:
                    print(' uh oh.... the least squares y-intercept and the lumped neutral position intercept are NOT the same ')
            print(tension_slopes)
            
            pause = 5
            self.pause_for_user(pause)
            rospy.sleep(pause)

            print(' ')
            print('. . .  Now finding detensioning slope for each wire  . . .')
            detension_slopes = np.zeros(MOTOR_NUM)
            for motor in range(MOTOR_NUM):
                print('   Motor ',motor)
                # Solve y=mx+c for m using least squares. In matrix form, y=Ap where A=[x 1] and p=[m;c]
                y = list(motor_positions_dict[motor]['detension_positions'][90])
                y.insert(0,lumped_neutral[motor])
                x = np.ones(len(y))*-90
                y = np.asarray(y)
                x[0] = 0
                print('x values:',x)
                print('y values:',y)
                print('A matrix:')
                A = np.vstack([x, np.ones(len(x))]).T
                print(A)
                m, c = np.linalg.lstsq(A, y, rcond=None)[0]
                print('lumped neutral position:',str(lumped_neutral[motor]))
                print('computed y-intercept:',str(c))
                detension_slopes[motor] = m
                print('computed tension slope:',m)
                if round(c,5)==round(lumped_neutral[motor],5):
                    print(' YAY! the least squares y-intercept and the lumped neutral position intercept are in fact the same!')
                    print('when rounded to the fifth decimal place')
                else:
                    print(' uh oh.... the least squares y-intercept and the lumped neutral position intercept are NOT the same ')
            print(detension_slopes)

            pause = 1
            self.pause_for_user(pause)
            rospy.sleep(pause)

            print(' ')
            print('determining the maximum tension/detension positions for each motor')
            max_tension = np.zeros(MOTOR_NUM)
            min_tension = np.zeros(MOTOR_NUM)
            num_attempts                = 2
            compensation_data_capture   = []
            combined_angles             = np.array([45,135,225,315])
            tension_sets                = [(2,1),(0,2),(3,0),(1,3)]            
            detension_sets              = [(3,0),(1,3),(2,1),(0,2)]    

            for i in range(len(tension_sets)):
                angle = combined_angles[i]
                x = np.cos(np.radians(angle))
                y = np.sin(np.radians(angle))
                tension_wire_A = tension_sets[i][0]
                tension_wire_B = tension_sets[i][1]
                detension_wire_A = detension_sets[i][0]
                detension_wire_B = detension_sets[i][1]

                print('The pair (x,y) = ('+str(round(x,3))+', '+str(round(y,3))+')')
                print('  ... means we are in Quadrant ',str(i+1))
                print('tension_wire_A = ',tension_wire_A,'        ;    tension_wire_B = ',tension_wire_B)
                print('detension_wire_A = ',detension_wire_A,'    ;    detension_wire_B = ',detension_wire_B)

                for attempt_num in range(num_attempts):

                    # move motors to the position given by (x,y)

                    # allow user to tighten tension_A and tension_B to move distal end to 90deg

                    # then allow user to tighten detension_A and detension_B until movement is seen at distal end

            # use new motor positions, (x,y), and tension/detension slopes to solve for quadrant compensation factor

            # save new quadrant compensation factors 

            







            print(' ')
            print('Saving calibrated slopes and maximum/minimum values in the workspace')
            calibration_file_name = CATHETER_ID+'_'+CALIBRATION_ID+'_WorkspaceNonlinearCalibration'
            calibration_table = np.vstack([target_position,lumped_neutral,tension_slopes,detension_slopes,max_tension,min_tension])
            np.save(calibration_file_name,calibration_table)
            np.savetxt(str(calibration_file_name+'.csv'),calibration_table,delimiter=",")
            print(' ')
            print('The captured data has been saved as '+data_capture_file_name+'.npy & ~.csv')
            rospy.sleep(1)
            print(' ')
            print(' Calibration process is complete !')

            pause = 1
            self.pause_for_user(pause)
            rospy.sleep(pause)
            
            print(' ')
            print('Workspace calibration procedure is complete. ')
            print(min_tension)
            print('This is done to prevent catheter damage')

            pause = 1
            self.pause_for_user(pause)
            rospy.sleep(pause)

            print(' ')
            num_attempts                = 2
            compensation_data_capture   = []
            combined_angles             = np.array([45,135,225,315])
            tension_sets                = [(2,1),(0,2),(3,0),(1,3)]            
            detension_sets              = [(3,0),(1,3),(2,1),(0,2)]    

            for i in range(len(tension_sets)):
                angle = combined_angles[i]
                x = np.cos(np.radians(angle))
                y = np.sin(np.radians(angle))
                tension_wire_A = tension_sets[i][0]
                tension_wire_B = tension_sets[i][1]
                detension_wire_A = detension_sets[i][0]
                detension_wire_B = detension_sets[i][1]

                print('The pair (x,y) = ('+str(round(x,3))+', '+str(round(y,3))+')')
                print('  ... means we are in Quadrant ',str(i+1))
                print('tension_wire_A = ',tension_wire_A,'        ;    tension_wire_B = ',tension_wire_B)
                print('detension_wire_A = ',detension_wire_A,'    ;    detension_wire_B = ',detension_wire_B)

                for attempt_num in range(num_attempts):

                    # move motors to the position given by (x,y)

                    # allow user to tighten tension_A and tension_B to move distal end to 90deg

                    # then allow user to tighten detension_A and detension_B until movement is seen at distal end

            # use new motor positions, (x,y), and tension/detension slopes to solve for quadrant compensation factor

            # save new quadrant compensation factors 

            







            print(' ')
            print('Saving calibrated slopes and maximum/minimum values in the workspace')
            calibration_file_name = CATHETER_ID+'_'+CALIBRATION_ID+'_WorkspaceNonlinearCalibration'
            calibration_table = np.vstack([target_position,lumped_neutral,tension_slopes,detension_slopes,max_tension,min_tension])
            np.save(calibration_file_name,calibration_table)
            np.savetxt(str(calibration_file_name+'.csv'),calibration_table,delimiter=",")
            print(' ')
            print('The captured data has been saved as '+data_capture_file_name+'.npy & ~.csv')
            rospy.sleep(1)
            print(' ')
            print(' Calibration process is complete !')

            pause = 1
            self.pause_for_user(pause)
            rospy.sleep(pause)
            
            print(' ')
            print('Workspace calibration procedure is complete. ')

        return calibration_table

    def inverse_kinematics(self,theta_x,theta_y,tension_slopes,detension_slopes,quadrant_compensations):
        # theta_x is angle of curvature in x axis, whereas theta_y is angle of curvature in y
        # We consider a potential difference in releasing and tensioning tension despite it ideally being the same.
        # Note that wires that are tensioning should have a positive resultant and detensioning should have negative

        # Code is written with the following layout for the wires: 
        #          y-axis
        #      Q2     2     Q1
        #             |
        #       0 ----|---- 1   x-axis
        #             |
        #      Q3     3     Q4

        quad1_comp_factor = quadrant_compensations[0,:]
        quad2_comp_factor = quadrant_compensations[1,:]
        quad3_comp_factor = quadrant_compensations[2,:]
        quad4_comp_factor = quadrant_compensations[3,:]

        if  (theta_x >= 0 and theta_y >= 0): 
            # cursor is in Quadrant 1, so we tension wires 2 & 1
            spool2_targ_angle = tension_slopes[2] * theta_y + quad1_comp_factor[2] * theta_x
            spool1_targ_angle = tension_slopes[1] * theta_x + quad1_comp_factor[1] * theta_y
            # and detension wires 3 & 0 
            spool3_targ_angle = detension_slopes[3] * theta_y + quad1_comp_factor[3] * theta_x
            spool0_targ_angle = detension_slopes[0] * theta_x + quad1_comp_factor[0] * theta_y
            
        elif(theta_x < 0 and theta_y >= 0): 
            # cursor is in Quadrant 2, so we tension wires 0 & 2
            spool0_targ_angle = tension_slopes[0] * theta_x + quad2_comp_factor[0] * theta_y
            spool2_targ_angle = tension_slopes[2] * theta_y + quad2_comp_factor[2] * theta_x
            # and detension wires 1 & 3 
            spool1_targ_angle = detension_slopes[1] * theta_x + quad2_comp_factor[1] * theta_y
            spool3_targ_angle = detension_slopes[3] * theta_y + quad2_comp_factor[3] * theta_x
            
        elif(theta_x < 0 and theta_y < 0): 
            # cursor is in Quadrant 3, so we tension wires 3 & 0
            spool3_targ_angle = tension_slopes[3] * theta_y + quad3_comp_factor[3] * theta_x
            spool0_targ_angle = tension_slopes[0] * theta_x + quad3_comp_factor[0] * theta_y
            # and detension wires 2 & 1 
            spool2_targ_angle = detension_slopes[2] * theta_y + quad3_comp_factor[2] * theta_x
            spool1_targ_angle = detension_slopes[1] * theta_x + quad3_comp_factor[1] * theta_y

        elif(theta_x >= 0 and theta_y < 0): 
            # cursor is in Quadrant 4, so we tension wires 1 & 3
            spool1_targ_angle = tension_slopes[1] * theta_x + quad4_comp_factor[1] * theta_y
            spool3_targ_angle = tension_slopes[3] * theta_y + quad4_comp_factor[3] * theta_x
            # and detension wires 0 & 2
            spool0_targ_angle = detension_slopes[0] * theta_x + quad4_comp_factor[0] * theta_y
            spool2_targ_angle = detension_slopes[2] * theta_y + quad4_comp_factor[2] * theta_x

        return np.array([spool0_targ_angle, spool1_targ_angle, spool2_targ_angle, spool3_targ_angle])

    def program_bootup(self,calibration_table):
        print(' ')
        print(' ')
        print('You can choose the next step from the following options:')
        print('   [z]   : Zero the robot relative to initial bootup position')
        print('   [n]   : Move the robot to tensioned neutral position (uses the loaded calibration)')
        print('   [y]   : Exit this menu and proceed to robot control')
        print('   [Esc] : Terminate the program')
        print(' ')
        while(True): 
            a = getKey(0.01)
            if(a=='z'):
                print('Moving robot to zero position was selected.')
                print('Press [z] again to begin.')
                while(True):
                    b = getKey(0.01)
                    if(b=='z'):
                        target_position = np.zeros(4)
                        self.setAllTargetPositions(target_position)
                        self.desired_motor_state_pub.publish(self.desired_motor_state)    
                        rospy.sleep(1)
                        print(' ')
                        print('Robot moved to Zero')
                        print(' ')
                        print('Returning to main menu')
                        rospy.sleep(2)
                        print(' ')
                        print('You can choose the next step from the following options:')
                        print('   [z]   : Zero the robot relative to initial bootup position')
                        print('   [n]   : Move the robot to tensioned neutral position (uses the loaded calibration)')
                        print('   [y]   : Exit this menu and proceed to robot control')
                        print('   [Esc] : Terminate the program')
                        break
            if(a=='n'):
                print('Moving robot to tensioned neutral position was selected.')
                print('Press [n] again to begin.')
                while(True):
                    b = getKey(0.01)
                    if(b=='n'):
                        tension_offsets = calibration_table[0,:]
                        self.setAllTargetPositions(tension_offsets)
                        self.desired_motor_state_pub.publish(self.desired_motor_state)    
                        rospy.sleep(1)
                        print(' ')
                        print('Robot moved to Neutral')
                        print(' ')
                        print('Returning to main menu')
                        rospy.sleep(2)
                        print(' ')
                        print('You can choose the next step from the following options:')
                        print('   [z]   : Zero the robot relative to initial bootup position')
                        print('   [n]   : Move the robot to tensioned neutral position (uses the loaded calibration)')
                        print('   [y]   : Exit this menu and proceed to robot control')
                        print('   [Esc] : Terminate the program')
                        break
            if(a=='y'):
                print(' ')
                print(' Exiting menu and proceeding to robot control.')
                break
            if ord(a) == 27:
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
        Dx = x-self.offset_x
        Dy = y-self.offset_y
        Dist = np.sqrt(Dx**2 + Dy**2)

        # If cursor is trying to move beyond a unit circle of 500 pixels
        if Dist > 500: 
            # we move the cursor to a position inside the circle 
            x = Dx/Dist*495 + self.offset_x
            y = Dy/Dist*495 + self.offset_y
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
            print(' ')
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

    print(' ')
    print('~ ~ ~ Catheter Control Program Has Begun! ~ ~ ~')
    if MOUSE_AVAILABLE == 1:
        print('        Mouse Control Mode Enabled')
    elif KEYBOARD_AVAILABLE == 1:
        print('        Keyboard Control Mode Enabled')

    calibration_table   = CCTRL.workspaceCalibration()
    tension_offsets     = calibration_table[0,:]
    tension_slopes      = calibration_table[2,:]
    detension_slopes    = calibration_table[3,:]
    print(tension_offsets)
    print(tension_slopes)
    print(detension_slopes)

    CCTRL.program_bootup(calibration_table)

    try: # Start Control Loop

        print("The robot is now active.")

        # Things to set before the loop
        offset_theta = 0    # this accounts for angle offsets between joystick and output of the catheter
        x = 0               # assume x,y of a unit circle
        y = 0               # assume x,y of a unit circle

        # Start mouse control
        if MOUSE_AVAILABLE == 1:
            mouse_object = MouseClass()
            
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
                    # The values need to be multiplied by 90deg to make into a deg movement for the catheter tip
                    x = mouse_object.normalized_x*90
                    y = mouse_object.normalized_y*90
                    offset_theta = mouse_object.offset_angle

            else:
                #error
                print("No Input Device Chosen")
                quit()

            # print('x=',x,' , y=',y)

            # Clamps input into within unit circle to prevent over-turning the spools
            # Only applicable for keyboard control, as mouse control already normalizes
            # if x**2+y**2 >=1:
            #     x = x/np.sqrt(x**2+y**2) 
            #     y = y/np.sqrt(x**2+y**2)
            #     print('Normalization occurred. x=',x,' , y=',y)

            # Re-orient the desired pose as needed
            x_rotated = x*np.cos(offset_theta) - y*np.sin(offset_theta)
            y_rotated = x*np.sin(offset_theta) + y*np.cos(offset_theta)

            # Set target positions
            target_position = CCTRL.inverse_kinematics(x_rotated,y_rotated,tension_slopes,detension_slopes)+tension_offsets

            # Command motors to target position
            CCTRL.setAllTargetPositions(target_position)
            CCTRL.desired_motor_state_pub.publish(CCTRL.desired_motor_state)

            #print('published position:', target_position) # tendon displacement
            #print('x,y = ',x_rotated,y_rotated)

            # A sleep statement to have code check every 0.1 seconds for new keyboard command
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass