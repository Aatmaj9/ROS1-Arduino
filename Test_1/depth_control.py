import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int32

nodeName = 'depth'
topicName = 'depth_info'

# Initialize ROS node and publisher
rospy.init_node(nodeName, anonymous=True)
pub = rospy.Publisher('thrust_info', Int32, queue_size=10)

# PID parameters for depth control
kp = 0.1  # Proportional gain
ki = 0.01  # Integral gain
kd = 0.001  # Derivative gain

# Initialize PID variables
integral_z = 0
prev_error_z = 0

# Callback function for receiving depth information
def callBackFunction(message):
    global depth
    depth = message.data
    # Call the depth_control function with the received depth message
    depth_control(depth)

# Subscribe to the depth_info topic
rospy.Subscriber(topicName, Int32, callBackFunction)
rate = rospy.Rate(10)

def depth_control(depth_reading):

    global integral_z, prev_error_z
    # Set the desired depth value you want to maintain
    setpoint_depth = 0.15

    # Compute PID error terms
    error_z = setpoint_depth - depth_reading

    # Update integral term
    integral_z += error_z

    # Update derivative term
    derivative_z = error_z - prev_error_z

    # Compute PID Control Signal
    pid_z = kp * error_z + ki * integral_z + kd * derivative_z

    # Scale PID values to thruster control parameters
    thruster_speed = 1500 + int(pid_z)

    # Clip thruster_speed within the acceptable range (1500 to 1900)
    thruster_speed = max(1500, min(1900, thruster_speed))

    # Publish the thruster speed to the 'information' topic
    pub.publish(int(thruster_speed))

    # Update previous error for the next iteration
    prev_error_z = error_z

# Callback function for when the node is shutdown
def shutdown_callback():
    rospy.loginfo("Node is shutting down.")
    # Publish 1500 to the thrust_info topic
    pub.publish(1500)

# Register the shutdown callback function
rospy.on_shutdown(shutdown_callback)

# Initialize timer
start_time = rospy.get_time()
timeout = 10  # Timeout in seconds

# Main control loop
rate = rospy.Rate(10)  # 10 Hz
while not rospy.is_shutdown():
    current_time = rospy.get_time()
    elapsed_time = current_time - start_time
    if elapsed_time >= timeout:
        rospy.loginfo("Timeout reached. Publishing 1500 to thrust_info topic.")
        # Publish 1500 to the thrust_info topic
        pub.publish(1500)
        break
    rate.sleep()