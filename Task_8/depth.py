
import rospy
from std_msgs.msg import Int32

nodeName = 'messagesubs'
topicName = 'depth_info'
command = 1

def callBackFunction(message):
    global command
    command = message.data

if _name_ == '_main_':
    rospy.init_node(nodeName, anonymous=True)
    pub = rospy.Publisher('information', Int32, queue_size=10)
    rospy.Subscriber(topicName, Int32, callBackFunction)
    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
        rospy.loginfo(command)
        pub.publish(command)
        rate.sleep()