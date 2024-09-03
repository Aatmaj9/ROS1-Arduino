#include <ros.h>
#include <std_msgs/Float32.h>
#include <Servo.h>
#include <Wire.h>
#include "MS5837.h"

MS5837 sensor;

ros::NodeHandle nh1;
ros::NodeHandle nh2;
byte servoPin1 = 9;
byte servoPin2 = 6;

std_msgs::Float32 outputMessage1;
std_msgs::Float32 outputMessage2;

long double value1;
long double map_value1;
long double map_value2;

// Define the publishers with the respective node handles
ros::Publisher pub1("depth_info", &outputMessage1);
ros::Publisher pub2("depth_info", &outputMessage2);

void callBackFunction(const std_msgs::Float32 &inputMessage) {
  value1 = inputMessage.data;
  map_value1 = map(value1, 0, 90, 1200, 1800);
  map_value2 = map(value1, 0, 90, 1800, 1200);

  outputMessage1.data = sensor.depth();
  outputMessage2.data = sensor.depth();

  pub1.publish(&outputMessage1);
  pub2.publish(&outputMessage2);
}

ros::Subscriber<std_msgs::Float32> sub1("command", &callBackFunction);
ros::Subscriber<std_msgs::Float32> sub2("command", &callBackFunction);

Servo servo1;
Servo servo2;

void setup() {
  Serial.begin(57600);

  Wire.begin();

  sensor.setModel(MS5837::MS5837_02BA);
  sensor.init();
  sensor.setFluidDensity(997); // kg/m^3 (997 freshwater, 1029 for seawater)

  servo1.attach(servoPin1);
  servo1.writeMicroseconds(1500);
  servo2.attach(servoPin2);
  servo2.writeMicroseconds(1500);

  delay(7000);

  nh1.initNode();
  nh1.advertise(pub1);
  nh1.subscribe(sub1);

  nh2.initNode();
  nh2.advertise(pub2);
  nh2.subscribe(sub2);
}

void loop() {
  sensor.read();

  Serial.print("Depth: ");
  Serial.print(sensor.depth());
  Serial.println(" m");

  callBackFunction(outputMessage1);

  servo1.writeMicroseconds(map_value1);
  servo2.writeMicroseconds(map_value2);

  nh1.spinOnce();
  nh2.spinOnce();

  delay(1000);
}