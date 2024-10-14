# ROS-with-Jetson-for-AUVs
We will be learning the basics of Jetson and how to integrate it with Arduino and various sensors like depth sensor, imu etc.
Here we will be working on Jetson Orin Nano with Jetpack 5.1.1 which has Ubuntu 20.04 based root system. The ROS version we will be using is ROS1 - Noetic.

# Setting up Jetson Nano

Requirements :
Monitor , Micro usb to usb for powering on jetson , hdmi cable for viewing jetson on monitor , SD card , Ethernet cable ( Router if required)

## Part 1 - 

### Step1 : 

Insert SD card in your laptop and download the image for nvidia website. Flash the image on the SD card using a flashing tool ( rufus )

More clarification regarding the image - 

JetPack includes Jetson Linux with bootloader, Linux kernel, Ubuntu desktop environment, and a complete set of libraries for acceleration of GPU computing, multimedia, graphics, and computer vision. It also includes samples, documentation, and developer tools for both host computer and developer kit, and supports higher level SDKs such as DeepStream for streaming video analytics, Isaac for robotics, and Riva for conversational AI. 


For Jetson Orin nano we downloaded Jetpack 5.1.1( as we were using ros1 )
It includes Jetson Linux 35.4.1 BSP with Linux Kernel 5.10, an Ubuntu 20.04 based root file system, a UEFI based bootloader, and OP-TEE as Trusted Execution Environment.


Note : If you want to upgrade to ROS 2 later we have to install Jetpack 6 which includes Jetson linux 36.2 which packs kernel 5.15 and Ubuntu 22.04 based root system.


https://developer.nvidia.com/embedded/jetpack-sdk-60dp 

Here we will be working on Jetpack 5.1.1 with Ubuntu 20.04 and ROS Npo


Step2 :  Insert the SD card in jetson. Connect jetson to power source and ethernet for internet . Connect monitor and jetson using HDMI cable. Go through the initial setup on screen.
You can see Jetson OS on the monitor.

Note: You can also use the SDK manager on a host linux system to flash the image in jetson. It is easier to do as it shows the Jetson device as well as the recommended Jetpack for the device. Connect jetson to the host computer using the usb or usb c port on the Jetson.

Part 2 - But in competition you cannot use a Jetson while it is connected to a monitor so we need to remotely control it. For this we use SSH.

Step 1 : You have to download SSH on your laptop and jetson. Connect the jetson  to power source, monitor and ethernet.

Step 2 : Open terminal on jetson and install SSH using command

sudo apt-get install openssh-server.

Now we have SSH installed on our jetson. To use it remotely we need to connect jetson with our laptop for that we need a static ip address of our jetson.

Part 3 : Obtain jetsons ip address

From routers website obtain jetsons ip address

192.168.0.1  - this is the router website for d-link you can see the Jetsons ip there

Part 5 : Connect laptop to jetson using SSH.

Open the linux terminal on your laptop. Make sure the Jetson and laptop are connected to the same network .

ssh -X <Username>@<Your_Jetson_IP>

Replace username with jetsons username and ip address you got.
You will be prompted to input the Jetson Nanos Password.
You now have remote access to your jetson nano.

Important points to note:
Use proper USB b cable for jetson nano. 
If lan port is not available near monitor use wifi router and long ethernet cables to connect them ( This is for jetsons without wifi module )

Installing ROS on Jetson Nano
Follow the same steps as installing ROS Noetic on ubuntu 20.04

Follow this -
http://wiki.ros.org/noetic/Installation/Ubuntu

Also make a catkin workspace and catkin package named arduino_comm with rospy roscpp and std_msgs as dependencies

Installing VSCode on Jetson Nano
Download Arm64 .deb file on your jetson from VSCode website
Download Visual Studio Code - Mac, Linux, Windows

Run the following commands in jetson terminal
sudo dpkg -i <downloaded-package-name>.deb
sudo apt-get install -f

Open VSCode using ‘code’

Note: Installing Arduino is not required.

Communication between Arduino and Jetson using ROS

We now want to write publisher and subscriber nodes such that -
A ROS publisher node(on Jetson) which will publish messages to a topic to which Arduino will subscribe.
We need an Arduino IDE code which will interpret these messages, change them and publish them back to another topic.
We need a ROS subscriber node(on Jetson) which will subscribe to this topic and receive messages back from Arduino.

Note: We can setup publisher and subscriber on same node on Jetson which we will do while working with sensors but for learning purposes we will create two separate nodes here,

Setting up the communication - 

The rosserial_arduino package helps you use ROS directly with Arduino IDE. Install it on Jetson using the command 


sudo apt-get install ros-noetic-rosserial-arduino


sudo apt-get install ros-noetic-rosserial

The preceding installation steps created the necessary libraries, now the following will create the ros_lib folder that the Arduino build environment needs to enable Arduino programs to interact with ROS.


You need to upload the Arduino code to Aruduino. You can either do this from Jetson or linux Arduino or from Windows.

You have to install Arduino ide on your laptop (Linux or windows) / Jetson - 

Note: The ros_lib library generated on your Jetson and the Arduino environment through which you are uploading the code should be the same. Otherwise it can cause errors.

<details>
<summary>Using Linux Arduino on your laptop -</summary>
Install Arduino IDE on your Ubuntu 20.04 . After Installation a <sketchbook> directory will be formed where the Linux Arduino environment saves your sketches. Typically this is a directory called sketchbook or Arduino in your home directory. e.g cd ~/Arduino/libraries

You have to generate the ros_lib library and save it in the Linux Arduino environment as  the jetson and the Arduino IDE should have the same ros_lib library.

You should have pre-installed ros noetic and rosserial_arduino package on your laptop.

Generate the ros_lib folder -

```
cd <sketchbook>/libraries
rm -rf ros_lib  
rosrun rosserial_arduino make_libraries.py
```
</details>



