In this task we have to control thrusters using two Arduinos.
For controlling two arduinos we need two to run serial_node for both the ports. You can disable all the 
services and try running rosrun rosserial_python serial_node.py /dev/ttyACM0 and ACM1 together.

You will get a conflict and both nodes will not run simultaneously. We need to make a launch file which 
will start both of them simultaneously. So it wont work by making separate services for running the 
serial_nodes in two terminals.

Here we make a new launch file named arduino.launch and also edit the arduino_connect.service.
All other files almost remian same.
We also make two subscriber nodes(accompanied with 2 launch files and two service files) with different topics (info_back1 and info_back2) to recieve data from the 
arduinos seperately. Remember to upload different codes in both arduinos where the change will be 
in the topic name(info_back1 afor one and info_back2 for the other)

So finally both thrusters will run at same speed as the jetson publishes 1 on power on.
