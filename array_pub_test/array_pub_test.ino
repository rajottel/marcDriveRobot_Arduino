#include <ros.h>
#include <Arduino.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt16.h>

float pos1[4] = {266, 290, 290, 290};
float home_pos[4] = {200, 180, 180, 180};

ros::NodeHandle nh;

std_msgs::Float32MultiArray des_pose;
ros::Publisher pub("des_pose", &des_pose);


 void message_cb( const std_msgs::UInt16& cmd_msg){
   if (cmd_msg.data == 1){
      des_pose.data = pos1;
   }
   else{
    des_pose.data = home_pos;
   }
   return des_pose.data;
  }

ros::Subscriber<std_msgs::UInt16> sub("go_home", &message_cb);

void setup(){
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
}

void loop(){
  
 
  des_pose.data_length = 4; 
  pub.publish(&des_pose);
  nh.spinOnce();
  delay(1000);
}
