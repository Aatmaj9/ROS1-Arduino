#!/usr/bin/env python3

import rospy 
#we are sneding Int32 messaage
from std_msgs.msg import Int32

#node name
nodeName='information'

#initializing subs node
rospy.init_node(nodeName,anonymous=True)

#our node is publishing messages to topicName
#specifying that the type of message we are publishing is (Int32)
publisher1=rospy.Publisher(topicName,Int32,queue_size=5)

ratePublisher=rospy.Rate(1)

intMessage=1

while not rospy.is_shutdown():
    rospy.loginfo(intMessage)
    publisher1.publish(intMessage)
    intMessage=intMessage+1
    ratePublisher.sleep( )