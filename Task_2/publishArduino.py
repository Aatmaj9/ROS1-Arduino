#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32,Char
import sys, select, termios, tty

key_bindings = {
    'w': 1,
    'a': 2,
    's': 3,
    'd': 4,
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key if key in key_bindings.keys() else None

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('information')
    pub = rospy.Publisher('information', Int32 , queue_size=10)

    try:
        while not rospy.is_shutdown():
            key = getKey()
            if key:
                rospy.loginfo(key_bindings[key])
                pub.publish(key_bindings[key])
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
