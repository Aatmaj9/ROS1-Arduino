#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import serial

def callback(data):
    command = data.data
    ser = serial.Serial('/dev/ttyUSB0', 9600)  # Change port accordingly
    ser.write(command.encode())

def subscriber():
    rospy.init_node('thruster_control_subscriber', anonymous=True)
    rospy.Subscriber('thruster_commands', String, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass
