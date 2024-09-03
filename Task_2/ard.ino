#include <ros.h>
#include <std_msgs/Int32.h>
#include <Servo.h>

ros::NodeHandle nh;
byte servoPin = 9;
std_msgs::Int32 outputMessage;

int x=1500;
ros::Publisher pub("info_back", &outputMessage);

void callBackFunction(const std_msgs::Int32 &inputMessage){
  
  if(inputMessage.data == 1){
    x=1600;
    outputMessage.data=2*inputMessage.data;
  }
  else if(inputMessage.data == 2){
    x=1400;
    outputMessage.data=0.5*inputMessage.data;
  }
  else if(inputMessage.data == 3){
    x=1500;
    outputMessage.data=2*inputMessage.data;
  }
  else if(inputMessage.data == 4){
    x=1700;
    outputMessage.data=2*inputMessage.data;
  }
  pub.publish(&outputMessage);
}

ros::Subscriber<std_msgs::Int32> sub("information", &callBackFunction);

Servo servo;

void setup() {
  servo.attach(servoPin);
  servo.writeMicroseconds(1500);

  delay(7000);

  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);

}

void loop() {
  servo.writeMicroseconds(x);
  nh.spinOnce();
  delay(1);

}