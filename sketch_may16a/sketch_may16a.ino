
/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#include <Arduino.h>
#include <VarSpeedServo.h>
//#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

VarSpeedServo servo;
VarSpeedServo servo2;
VarSpeedServo servo3;
VarSpeedServo servo4;
VarSpeedServo servo5;

void servo_cb( const std_msgs::UInt16& cmd_msg){
  servo.write(cmd_msg.data, 30); //set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}
void servo_cb2( const std_msgs::UInt16& cmd_msg2){
  servo2.write(cmd_msg2.data, 40); //set servo angle, should be from 0-180 
}
void servo_cb3( const std_msgs::UInt16& cmd_msg3){
  servo3.write(cmd_msg3.data, 60); //set servo angle, should be from 0-180 
}
void servo_cb4( const std_msgs::UInt16& cmd_msg4){
  servo4.write(cmd_msg4.data, 60); //set servo angle, should be from 0-180 
}
void servo_cb5( const std_msgs::UInt16& cmd_msg5){
  servo5.write(cmd_msg5.data, 60); //set servo angle, should be from 0-180 
}


ros::Subscriber<std_msgs::UInt16> sub("base_servo", servo_cb);
ros::Subscriber<std_msgs::UInt16> sub2("servo1", servo_cb2);
ros::Subscriber<std_msgs::UInt16> sub3("servo2", servo_cb3);
ros::Subscriber<std_msgs::UInt16> sub4("servo3", servo_cb4);
ros::Subscriber<std_msgs::UInt16> sub5("gripper_servo", servo_cb5);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(sub4);
  nh.subscribe(sub5);
  
  servo.attach(2); //attach it to pin 9
  servo2.attach(3);
  servo3.attach(4);
  servo4.attach(5);
  servo5.attach(6);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
