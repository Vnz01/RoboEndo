# Catheter-Control
#### To run catheter code:
##### Step1 - Open a terminal, and run :
roslaunch maxon_epos_example example_maxon_epos.launch
If successful, last line should read "Motors Initialized"

##### Step2.1-1 - For Xbox Controller 
rosrun joy joy_node
		^- this one to enable the topic!

##### Step3 - Open another terminal, and run :
rosrun maxon_epos_example control_test.py

##### EXTRA FEATURES
##### move the the catheter to neutral position and pause controls (when in mouse mode) :
  rostopic pub -1 /if_back_to_neutral std_msgs/Bool "data: true"
##### resume controls, no changes in position (will remain at neutral until mouse is moved) :
  rostopic pub -1 /if_back_to_neutral std_msgs/Bool "data: false"
##### record control data for all 4 motors :
  rosbag record -O xxxx.bag --split --duration=30 /maxon_bringup/set_all_states /trakstar/raw_data(optional) .....(and more)
  
  ex. 
rosbag record -O 01Jun2023_UserStudyTrials_CC_EL04212301_23May2023_MP2_FixedWRobot_Real2.bag --split --duration=2h maxon_bringup/set_all_states /trakstar/raw_data

rosbag record -O 01Jun2023_UserStudyTrials_CC_CR04182301_30May2023_MP1_FreeNoRobot_Real2.bag --split --duration=2h maxon_bringup/set_all_states /trakstar/raw_data

rosbag record -O 01Jun2023_UserStudyTrials_CC_CR04182301_30May2023_MP1_FreeWRobot_Real2.bag --split --duration=2h maxon_bringup/set_all_states /trakstar/raw_data

  
rosbag record -O 02Jun2023_DeviceValidation_CR04182301_30May2023_MP2_ScaleUpMode_ROM_Circle2.bag --split --duration=2h maxon_bringup/set_all_states /trakstar/raw_data
  
##### replay recorded data
  rosplay xxxx_*.bagrosbag record -O 02Jun2023_DeviceValidation_CR04182301_30May2023_MP2_ScaleUpMode_ROM_Circle.bag --split --duration=2h maxon_bringup/set_all_states /trakstar/raw_data
	

#### Trouble Shooting
- make sure you have the following environment variables setup :
   source /opt/ros/noetic/setup.bash
   source /home/arclab/catkin_ws/devel/setup.bash

- make sure your python code is executable (showing green using "ls"), otherwise do :
   sudo chmod 777 your_file_name.py

=================================
To reinstall everything:
1. download newest EPOS Maxon Linux Drivers, and install using their shell script
2. download catkin_tools so that you have catkin build. A typical place to put it is in a new folder called catkin_ws (see catkin_tools readme)
3. download the git repos to build. Run catkin build (and run again if you change C++ files)
