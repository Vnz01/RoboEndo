# How to use?

## Using Two Adafruit M0 Boards

## Receiver MCU:

Program Receiver Side using the code found in /Receiver/ECE using Arduino IDE then plug in to your computer that will be connected to the Maxon Motors

> Optionally you can also do a quick program on platformio using the RF_Head Libary to just quickly read the data and print to serial. I made the switch from Arduino IDE to Platformio mid project, hence why Receiver is programmed through Arduino and Transmitter on platformio

## Transmitter MCU:

Program Transmitter Side using the code found in /Transmitter using Platformio

Edit Latency, I found 10 to work the best giving the controller and receiver to have the best performance.

## Pinout

En - Gnd: Slide Switch (Power Enabling)

13 - Gnd: Lock Switch (Position Control)

12 - Gnd: Momentary Button (Forward)

11 - Gnd: Momentary Button (Backward)

10 - Gnd: Analog Stick Switch (Start Button)

9 - Gnd: Power LED

6 - Gnd: Charging LED

3V3 - Analog Stick Power

GND - Analog Stick GND

A0 - Analog Stick X

A1 - Analog Stick Y

## Computer Side

Using Linux Ubuntu 20.04, install ROS2 Noetic. Download catkin. Following the readme inside /Catheter-Control-main/src/README.md, install the official EPOS Linux Driver. Check the EPOS Command Library PDF. I found the drivers on the japanese maxon motor website after looking at their products. EPOS Studio is for downloading drivers straight to the motors. Then you will be using catkin build not catkin_make to access all the directories. So following the other README located in /Catheter-Control-main/README.md scroll to the bottom. Copy the src file in /Catheter-Control-main to the src file created by catkin build and run your catkin build. To edit the code, look inside the Catheter-Control-main and find control_test.py and edit the if else conditional where NEW CONTROLLER is located.

## ROS Commands

Source the files -
source ~/catkin_ws/devel/setup.bash

Start ROS -
roslaunch maxon_epos_example example_maxon_epos.launch

Run ROSPY -
rosrun maxon_epos_example control_test.py

## Left Off

Controller working properly, there is commented code for a scroll wheel. Pins can be defined to any open pin. It adds to the existing string sent over serial. All the code just needs to be uncommented once scroll wheel is attached.

For the catheter side, look in Catheter-Control-main/src/maxon-epos-example/scripts/old/control_test.py

This file is the edited file and the one we use to control the catheter as we don't have libraries for the main control.py. In this file this is where it is defined for all the movements. To use the controller on your own file it is very modular, just make sure to read input from serial which is a constant string of data that just needs to be parsed.

I left off at implementing the new motor 5 for retracting and protracting. I included a 5th motor in the configuration file in motor_hd.yaml. To implement this, looking inside maxon_epos_driver it seems that it checks how many motors are included and sends commands. You need to define in maxon_epos_example how these commands are sent over ROS and how it will control the 5th motor.

Essentially to get a grasp of this I would restart on a new control_test.py and just read serial values sent by the controller, and try to redefine each movement instead of following the work I have.
