/*
  WDIY ESP32-CAM Plugin Robot
  Motor Driver
  https://github.com/abourdim/wdiy_esp32_cam_robot
*/
#include "SetMotor.h"
void setup() {
  Serial.begin(115200);
  motor_init();  // set up the two PWM channels
}

void loop() {
  // put your main code here, to run repeatedly:
  Car_forward(200, 200);    //The robot moves forward
  delay(2000);              //delay 2s
  Car_backwards(200, 200);  //No reverse on this chassis -- stops, and says why
  delay(2000);
  Car_left(150, 150);  //The robot swings left around its left wheel
  delay(2000);
  Car_right(150, 150);  //The robot swings right around its right wheel
  delay(2000);
  Car_stop();  //The robot stops
  delay(2000);
}
