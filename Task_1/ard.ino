#include <ros.h>
#include <std_msgs/Int32.h>


ros::NodeHandle nh;
byte servoPin = 9;
Servo servo;
std_msgs::Int32 outputMessage;

ros::Publisher pub("info_back", &outputMessage);

void callBackFunction(const std_msgs::Int32 &inputMessage){
  outputMessage.data=2*inputMessage.data;
  if(msg.inputMessage == 1){
    digitalWrite(servoPin, HIGH);
  }
  pub.publish(&outputMessage);
}

ros::Subscriber<std_msgs::Int32> sub("information", &callBackFunction);


void setup() {
  servo.attach(servoPin);
  servo.writeMicroseconds(1500);

  delay(7000);

  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);

}

void loop() {
  nh.spinOnce();
  delay(1);

}