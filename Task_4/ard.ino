#include <ros.h>
#include <std_msgs/Int32.h>
#include <Servo.h>

ros::NodeHandle nh;

byte servoPin1 = 9;
byte servoPin2 = 4;

std_msgs::Int32 outputMessage;

int x=1500;
int y=1500;

ros::Publisher pub("info_back", &outputMessage);

void callBackFunction(const std_msgs::Int32 &inputMessage){
  if(inputMessage.data == 1){
    x=1600;
    y=1400;
    outputMessage.data=2*inputMessage.data;
  }
  pub.publish(&outputMessage);
}

ros::Subscriber<std_msgs::Int32> sub("information", &callBackFunction);

Servo servo1;
Servo servo2;

void setup() {
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo1.writeMicroseconds(1500);
  servo2.writeMicroseconds(1500);

  delay(7000);

  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);

}

void loop() {
  servo1.writeMicroseconds(x);
  servo2.writeMicroseconds(y);
  nh.spinOnce();
  delay(1);

}