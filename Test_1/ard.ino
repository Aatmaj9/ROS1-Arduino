#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <Servo.h>
#include <Wire.h>
#include "MS5837.h"

MS5837 sensor;

ros::NodeHandle nh;
byte servoPin1 = 9;
byte servoPin2 = 6;

std_msgs::Float32 depthMessage;
std_msgs::Int32 thrustMessage;

float depthValue;

Servo servo1;
Servo servo2;

void thrustCallback(const std_msgs::Int32 &thrustInput) {
  int thrustValue = thrustInput.data;

  servo1.writeMicroseconds(thrustValue);
  servo2.writeMicroseconds(3000-thrustvalue);

}

ros::Publisher depthPub("depth_info", &depthMessage);
ros::Subscriber<std_msgs::Int32> thrustSub("thrust_info", &thrustCallback);


void setup() {
  Wire.begin();

  sensor.setModel(MS5837::MS5837_02BA);
  sensor.init();
  sensor.setFluidDensity(997);
  servo1.attach(servoPin1);
  servo1.writeMicroseconds(1500);
  servo2.attach(servoPin2);
  servo2.writeMicroseconds(1500);

  delay(2000);

  nh.getHardware()->setBaud(57600); 
  nh.initNode();
  nh.advertise(depthPub);
  nh.subscribe(thrustSub);
}

void loop() {
  sensor.read();
  depthValue = sensor.depth();
  depthMessage.data = depthValue;
  depthPub.publish(&depthMessage);
  nh.spinOnce();

  delay(100);
}