#include <ros.h>
#include <std_msgs/Float32.h>
#include <Servo.h>

ros::NodeHandle nh1;
ros::NodeHandle nh2;
byte servoPin1 = 9;
byte servoPin2 = 6;
//std_msgs::Int32 outputMessage;
long double value1;
long double map_value1;
long double map_value2;

//ros::Publisher pub("info_back2", &outputMessage);

void callBackFunction(const std_msgs::Float32 &inputMessage){
  value1=inputMessage.data;
  map_value1 = map(value1, -90, 90, 1200, 1800);
  map_value2 = map(value1,-90,90,1800,1200);
  //pub.publish(&outputMessage);
}

ros::Subscriber<std_msgs::Float32> sub("command", &callBackFunction);

Servo servo1;
Servo servo2;

void setup() {
  Serial.begin(57600);
  servo1.attach(servoPin1);
  servo1.writeMicroseconds(1500);
  servo2.attach(servoPin2);
  servo2.writeMicroseconds(1500);

  delay(7000);

  nh1.initNode();
  //nh.advertise(pub);
  nh1.subscribe(sub);

  nh2.initNode();
  //nh.advertise(pub);
  nh2.subscribe(sub);

}

void loop() {
  servo1.writeMicroseconds(map_value1);
  servo2.writeMicroseconds(map_value2);
  nh1.spinOnce();
  nh2.spinOnce();
  delay(1);

}