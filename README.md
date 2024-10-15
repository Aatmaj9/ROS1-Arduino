# ROS1 with Jetson: Sensor Control and Beyond
We will be learning the basics of Jetson and how to integrate it with Arduino and various sensors like depth sensor, imu etc.
Here we will be working on Jetson Orin Nano with Jetpack 5.1.1 which has Ubuntu 20.04 based root system. The ROS version we will be using is ROS1 - Noetic.

# Setting up Jetson Nano

Requirements :
Monitor , Micro usb to usb for powering on jetson , hdmi cable for viewing jetson on monitor , SD card , Ethernet cable ( Router if required)

## Part 1 - Install Jetson OS 

### Method 1 - SD Card Image Method

#### Step1 : Flash the image on SD card 

Insert SD card in your laptop and download the image for nvidia website. Flash the image on the SD card using a flashing tool ( rufus )

More clarification regarding the image - 

JetPack includes Jetson Linux with bootloader, Linux kernel, Ubuntu desktop environment, and a complete set of libraries for acceleration of GPU computing, multimedia, graphics, and computer vision. It also includes samples, documentation, and developer tools for both host computer and developer kit, and supports higher level SDKs such as DeepStream for streaming video analytics, Isaac for robotics, and Riva for conversational AI. 

For Jetson Orin nano we downloaded Jetpack 5.1.1( as we were using ros1 )
It includes Jetson Linux 35.4.1 BSP with Linux Kernel 5.10, an Ubuntu 20.04 based root file system, a UEFI based bootloader, and OP-TEE as Trusted Execution Environment.
You can download it from here- 

https://developer.nvidia.com/embedded/jetpack-sdk-511

Note : If you want to upgrade to ROS 2 later we have to install Jetpack 6 which includes Jetson linux 36.2 which packs kernel 5.15 and Ubuntu 22.04 based root system.

https://developer.nvidia.com/embedded/jetpack-sdk-60dp 

Here we will be working on Jetpack 5.1.1 with Ubuntu 20.04 and ROS Npo

#### Step2 :  

Insert the SD card in jetson. Connect jetson to power source and ethernet for internet . Connect monitor and jetson using HDMI cable. Go through the initial setup on screen.
You can see Jetson OS on the monitor.

### Method 2 - SDK Manager

Note: You can also use the SDK manager on a host linux system to flash the image in jetson. It is easier to do as it shows the Jetson device as well as the recommended Jetpack for the device. Connect jetson to the host computer using the usb or usb c port on the Jetson.

## Part 2 - Setting up SSH

#### Step 1 : Downloading SSH

You have to download SSH on your laptop and jetson. Connect the jetson  to power source, monitor and ethernet.

Open terminal on jetson and install SSH using command

```
sudo apt-get install openssh-server
```

Now we have SSH installed on our jetson. To use it remotely we need to connect jetson with our laptop for that we need a static ip address of our jetson.

#### Step 2 : Connecting to Jetson

From routers website obtain jetsons ip address

192.168.0.1  - this is the router website for d-link you can see the Jetsons ip there

Connect laptop to jetson using SSH.

Open the linux terminal on your laptop. Make sure the Jetson and laptop are connected to the same network .

```
ssh -X <Username>@<Your_Jetson_IP>
```

Replace username with jetsons username and ip address you got.
You will be prompted to input the Jetson Nanos Password.
You now have remote access to your jetson nano.

Important points to note:
Use proper USB b cable for jetson nano. 
If lan port is not available near monitor use wifi router and long ethernet cables to connect them ( This is for jetsons without wifi module )

# Installing ROS Noetic on Jetson Nano

Follow the same steps as installing ROS Noetic on ubuntu 20.04

Follow this -
http://wiki.ros.org/noetic/Installation/Ubuntu

Now make a catkin workspace and catkin package named arduino_comm with rospy roscpp and std_msgs as dependencies. If you dont remember let us see how to do this -

```
source /opt/ros/noetic/setup.bash
mkdir -p ~/amogh_ws/src
cd ~/amogh_ws
catkin_make
source devel/setup.bash
cd ~/amogh_ws/src
catkin_create_pkg arduino_comm std_msgs rospy roscpp
cd ~/amogh_ws
catkin_make
source devel/setup.bash
```

# Communication between Arduino and Jetson using ROS

We now want to write publisher and subscriber nodes such that -
A ROS publisher node(on Jetson) which will publish messages to a topic to which Arduino will subscribe.
We need an Arduino IDE code which will interpret these messages, change them and publish them back to another topic.
We need a ROS subscriber node(on Jetson) which will subscribe to this topic and receive messages back from Arduino.

Note: We can setup publisher and subscriber on same node on Jetson which we will do while working with sensors but for learning purposes we will create two separate nodes here,

## Setting up the communication - 

The rosserial_arduino package helps you use ROS directly with Arduino IDE. Install it on Jetson using the command 


```
sudo apt-get install ros-noetic-rosserial-arduino
sudo apt-get install ros-noetic-rosserial
```

The preceding installation steps created the necessary libraries, now the following will create the ros_lib folder that the Arduino build environment needs to enable Arduino programs to interact with ROS.

You need to upload the Arduino code to Aruduino. You can either do this from Jetson or linux Arduino or from Windows.

You have to install Arduino ide on your laptop (Linux or windows) / Jetson - 

Note: The ros_lib library generated on your Jetson and the Arduino environment through which you are uploading the code should be the same. Otherwise it can cause errors.

<details>
<summary>Using Arduino IDE on Ubuntu 20.04 system </summary>

### Installing Arduino IDE on your Ubuntu 20.04 - 

Download the Linux 64 bits version for Arduino 1.8.18 from here - https://www.arduino.cc/en/software/OldSoftwareReleases

Follow the steps to install Arduino IDE here - https://docs.arduino.cc/software/ide-v1/tutorials/Linux/

After Installation a  directory will be formed where the Linux Arduino environment saves your sketches. Typically this is a directory called sketchbook or Arduino in your home directory. e.g cd ~/Arduino/libraries

You have to generate the ros_lib library and save it in the Linux Arduino environment.

You should have pre-installed ros noetic and rosserial_arduino package on your Ubuntu 20.04 laptop.

Generate the ros_lib folder -

```
cd Arduino/libraries
rm -rf ros_lib  
rosrun rosserial_arduino make_libraries.py
```
</details>

<details>
<summary>Arduino IDE on Windows</summary>

You need to copy the ros_lib  from Jetson to the Windows sytem's Arduino sketchbook/libraries folder.

Another alternative for windows is to install rosserial arduino lib directly from Arduino libraries in the Arduino IDE. Just open the Library Manager from the IDE menu in Sketch -> Include Library -> Manage Library. Then search for "rosserial" and install it.

Note: This is not recommended as the library through which the code is uploaded on the arduino and the ros_lib in Jetson may mismatch and cause errors. Instead copying the ros_lib from Jetson is a safer option.

</details>

<details>
<summary>Arduino IDE on Jetson</summary>

### Installing Arduino on Jetson

The steps are similar to that of Ubuntu -

Download the Arm64 version for Arduino 1.8.18 from here - https://www.arduino.cc/en/software/OldSoftwareReleases

Follow the steps to install Arduino IDE here - https://docs.arduino.cc/software/ide-v1/tutorials/Linux/

After Installation a  directory will be formed where the Linux Arduino environment saves your sketches. Typically this is a directory called sketchbook or Arduino in your home directory. e.g cd ~/Arduino/libraries

Generate the ros_lib folder -

```
cd Arduino/libraries
rm -rf ros_lib  
rosrun rosserial_arduino make_libraries.py
```
</details>

Upload your Arduino code ( You can upload the sample arduino code provided in Task 1 ) to the Arduino and connect it to Jetson.

Determine the port where arduino is connected - 
```ls /dev/ttyACM*```

The output will be the port at which the arduino is connected e.g /dev/ttyACM0

Open a terminal on jetson and run -
```
sudo chmod 666 /dev/ttyACM0
```

This command gives read and write permission to the ACM0 port where the arduino is connected.

Now start roscore on a new terminal- 
```
roscore
```
On another terminal run -

```
rosrun rosserial_python serial_node.py /dev/ttyACM0
```

After the above commands run successfully you will get a output like this - 

```
[INFO] [1728988013.575030]: ROS Serial Python Node
[INFO] [1728988013.584133]: Connecting to /dev/ttyACM0 at 57600 baud
[INFO] [1728988015.691279]: Requesting topics...
[INFO] [1728988015.716080]: Note: publish buffer size is 280 bytes
[INFO] [1728988015.718253]: Setup publisher on info_back [std_msgs/Int32]
[INFO] [1728988015.733503]: Note: subscribe buffer size is 280 bytes
[INFO] [1728988015.735901]: Setup subscriber on information [std_msgs/Int32]
 ```

You have successfully established connection between Jetson and Arduino. You can do a lot of interesting things with this. Let us look at it in the form of tasks - 

## Task 1 - Testing

Here are sample codes for testing - The publisher node publishes numbers automatically ,the arduino doubles it and returns the output to the subscriber node which prints the doubled value.

**Publisher Node**
```py
#!/usr/bin/env python3

import rospy
#we are sending Int32 message
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
```

**Subscriber Node**
```py
#!/usr/bin/env python3
import rospy

from std_msgs.msg import Int32

nodeName = 'messagesubs'

topicName = 'info_back'

def callBackFunction(message):
    print("From Arduino, we received %d"%message.data)

rospy.init_node(nodeName,anonymous=True)

rospy.Subscriber(topicName, Int32, callBackFunction)

rospy.spin( )
```

**Arduino IDE Code**
```ino
#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;

std_msgs::Int32 outputMessage;

ros::Publisher pub("info_back", &outputMessage);

void callBackFunction(const std_msgs::Int32 &inputMessage){
  outputMessage.data=2*inputMessage.data;
  pub.publish(&outputMessage);
}

ros::Subscriber<std_msgs::Int32> sub("information", &callBackFunction);

void setup() {
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
```
## Task 2 -  Controlling thrusters using your laptop with jetson and arduino.


1. Control a single thruster using keys on your laptop such that - W to start the thruster , A to gradually increase the anticlockwise speed, D to gradually increase the clockwise speed and S to stop it.

2. Controlling two thrusters for yaw motion - When A/D is pressed the AUV should turn left/right, for that both thrusters should rotate at same speed in opposite directions

3. Surge . When W key is pressed the AUV should move forward and when A is pressed it should move backward.

4. Pitch -  When the ‘up’ key is pressed the AUV should point downward while for the ‘down’ key it should turn upward.

5. Now we are left with heave, sway controls - Use left and right arrow keys for sway motion. Use Q and E for heave motion.

6. Finally add a kill switch K for the AUV.

# Auto running of nodes and commands

We cannot run our codes manually on Jetson, the nodes should auto run when our AUV i.e Jetson is switched on. Let us see how to do that -
Which are the main nodes which should run when the AUV is started ?

1. roscore - Essential to run all the ros nodes
2. rosserial - Essential for communication with Arduino
3. Other nodes - The subscriber and publisher nodes with which you communicate with Arduino and other sensors

## Step 1 


