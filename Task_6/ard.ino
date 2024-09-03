#include <ros.h>
#include <std_msgs/Int32.h>
#include <Servo.h>
#include <Wire.h>

ros::NodeHandle nh;
byte servoPin = 9;
std_msgs::Int32 outputMessage;


const int MPU_addr = 0x68;  
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

int x=1500;

ros::Publisher pub("imu_info", &outputMessage);

void callBackFunction(const std_msgs::Int32 &inputMessage) {
  if ((Tmp/340.00+36.53) < 28) {
    outputMessage.data = 1;
  } else {
    outputMessage.data = 0;
  }

  if(inputMessage.data == 1){
    x=1600;
  }
  if(inputMessage.data == 0){
    x=1500;
  }
  
  pub.publish(&outputMessage);
}

ros::Subscriber<std_msgs::Int32> sub("information", &callBackFunction);

Servo servo;

void setup() {

  servo.attach(servoPin);
  servo.writeMicroseconds(1500);

  delay(7000);
  
  Wire.begin(); 
  Wire.beginTransmission(MPU_addr); 
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true); 
  Serial.begin(57600);

  delay(7000);

  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);
  Serial.begin(57600);
}


void loop() { 
  servo.writeMicroseconds(x);
  nh.spinOnce();
  delay(1);
  Wire.beginTransmission(MPU_addr); 
  Wire.write(0x3B); 
  Wire.endTransmission(false); 
  Wire.requestFrom(MPU_addr,14,true);
  
  AcX = Wire.read() << 8 | Wire.read();     
  AcY = Wire.read() << 8 | Wire.read();  
  AcZ = Wire.read() << 8 | Wire.read();  
  Tmp = Wire.read() << 8 | Wire.read();  
  GyX = Wire.read() << 8 | Wire.read();  
  GyY = Wire.read() << 8 | Wire.read();  
  GyZ = Wire.read() << 8 | Wire.read();

  nh.spinOnce();

  Serial.print("AcX = "); Serial.print(AcX); 
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);
  delay(1000);
  
}
