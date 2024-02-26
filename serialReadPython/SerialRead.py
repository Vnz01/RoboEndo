#!/usr/bin/env python
import serial
import rospy
from std_msgs.msg import String

ser = serial.Serial("/dev/ttyACM0", 9600)
rospy.init_node('random_value_publisher', anonymous=True)
pub = rospy.Publisher('random_topic', String, queue_size=10)
rate = rospy.Rate(60)

if __name__ == '__main__':
    print("Starting")
    while not rospy.is_shutdown():
        value = ser.readline().decode("utf-8")
        # Publish the random value to the 'random_topic'
        pub.publish(value)
        # Print the value for demonstration purposes
        rospy.loginfo("Published random value: %s", value)
