#include <Arduino.h>


//ROS Setup
#include <ros.h>
#include <std_msgs/Float32.h>
ros::NodeHandle  nh;

//ADAFruit PWM Shield Setup
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

//Servo definition:
uint8_t base_servo = 0;
uint8_t servo1 = 1;
uint8_t servo2 = 2;
uint8_t servo3 = 5; 
uint8_t gripper_servo = 4;

float baseSmooth;
float basePrev;

float j1Smooth;
float j1Prev;

float j2Smooth;
float j2Prev;

float j3Smooth;
float j3Prev;

//Servo control loops:

void servo_cb3(const std_msgs::Float32& cmd_msg3) {
  j2Smooth = (cmd_msg3.data * 0.03) + (j2Prev * 0.97);
  j2Prev = j2Smooth;

  pwm.setPWM(servo2,0,j2Smooth);
}
void servo_cb4(const std_msgs::Float32& cmd_msg4) {
  j3Smooth = (cmd_msg4.data * 0.03) + (j3Prev * 0.97);
  j3Prev = j3Smooth;

  pwm.setPWM(servo3,0,j3Smooth);
}
void servo_cb5(const std_msgs::Float32& cmd_msg5) {
  pwm.setPWM(gripper_servo,0,cmd_msg5.data);
}

//ROS subscriptions

ros::Subscriber<std_msgs::Float32> sub3("servo2", servo_cb3);
ros::Subscriber<std_msgs::Float32> sub4("servo3", servo_cb4);
ros::Subscriber<std_msgs::Float32> sub5("gripper_servo", servo_cb5);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //ROS
  
  nh.initNode();

  nh.subscribe(sub3);
  nh.subscribe(sub4);
  nh.subscribe(sub5);
   

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);

}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  //pwm.setPWM(base_servo,0,300); //300 is centre
  //pwm.setPWM(servo1,0,300); // 300 is upright
  //pwm.setPWM(servo2,0,300); // 300 is 90 degrees
  //pwm.setPWM(servo3,0,300); // 100 is facing down when parallel with table
  //pwm.setPWM(gripper_servo,0,150);
  delay(1);
}
