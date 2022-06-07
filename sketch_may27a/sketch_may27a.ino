#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  // put your setup code here, to run once:
pwm.begin();
pwm.setPWMFreq(50);
}

void loop() {
  // put your main code here, to run repeatedly:
pwm.setPWM(0,0,270); //70(Left) to (middle is 270, home position) to 530 (Right)
pwm.setPWM(1,0,370); //100(forward) to (middle(straight up) is 250, home position) to 440(backward)
pwm.setPWM(2,0,150); //100(forward) to (middle(straight up) is 300, home position) to 500(backward)
pwm.setPWM(3,0,140); //100(forward) to (middle(straight up) is 350) to 550(backwards) (140 is home position)
//pwm.setPWM(4,0,100); //servo gets too hot)
}
