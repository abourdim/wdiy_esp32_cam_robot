#ifndef SETMOTOR_H
#define SETMOTOR_H
#include "Arduino.h"

// --- What this board actually has -------------------------------------------
// The keyestudio car this code came from drove an I2C motor controller at 0x30
// on GPIO14/13, which took a PWM byte and a direction byte per motor. There is
// no such chip here. On esp32_cam_plugin_robot each motor is a single low-side
// N-MOSFET (Q1/Q2, 2N7002) with its gate straight on a GPIO, the motor between
// the screw terminal and the battery rail (VCC), and a 1N4007 across it for
// flyback:
//
//        VCC ---+--------+
//               |        |
//              [M]      /|\  D1/D2 (1N4007, cathode to VCC)
//               |        |
//        J1/J2 -+--------+
//               |
//               D
//    GPIO --[  Q1/Q2 2N7002
//               S
//              GND
//
// One switch per motor means one degree of freedom per motor: how hard it is
// driven. There is no H-bridge, so THERE IS NO REVERSE. Everything downstream
// of this file inherits that -- see Car_backwards() below and the joystick
// mixing in app_server.h.
//
// R1/R2 (10k) hold both gates down. That is not just tidiness on GPIO12: it is
// MTDI, a strapping pin the ESP32 samples at reset to choose the flash voltage,
// and it must be low at boot or the module comes up expecting 1.8V flash and
// does not boot at all. The pulldown is what makes GPIO12 safe to use here.
#define M1_PIN 13  // J1 screw terminal -- gate of Q1
#define M2_PIN 12  // J2 screw terminal -- gate of Q2 (MTDI, see above)

// LEDC channels 4/5, deliberately not 0/1: the camera driver takes
// LEDC_CHANNEL_0 / LEDC_TIMER_0 for its XCLK (see the camera config in
// 4_CamRobot.ino) and quietly reprogramming its timer under it is a long
// afternoon. Channels 4 and 5 share timer 2, which nothing else wants.
#define M1_CH 4
#define M2_CH 5

// 20kHz: above hearing, so the motors don't whine. A 2N7002 switches a gate
// this small far faster than that, so there is no meaningful switching loss.
#define MOTOR_PWM_HZ   20000
#define MOTOR_PWM_BITS 8

void motor_init() {
  ledcSetup(M1_CH, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
  ledcSetup(M2_CH, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
  ledcAttachPin(M1_PIN, M1_CH);
  ledcAttachPin(M2_PIN, M2_CH);
  ledcWrite(M1_CH, 0);
  ledcWrite(M2_CH, 0);
}

// motor:    M1_CH or M2_CH
// pwmvalue: 0..255, 0 = coasting (the MOSFET is off and the motor freewheels
//           through its flyback diode -- there is no braking on this chassis)
void motor_write(uint8_t motor, uint8_t pwmvalue) {
  ledcWrite(motor, pwmvalue);
}

// M1 is the RIGHT motor and M2 the LEFT one. Nothing on the PCB says so -- the
// silkscreen just reads J1 and J2 -- but the whole steering convention above
// this file assumes it, so wire the terminals to match rather than trying to
// correct it in software later.
void Car_forward(uint8_t M1_speed, uint8_t M2_speed) {
  motor_write(M1_CH, M1_speed);
  motor_write(M2_CH, M2_speed);
}

// Kept so the sketch below reads like every other WDIY car, and so a lesson
// can point at it. It cannot do anything: reversing a motor needs an H-bridge
// and this board has a single low-side switch per side. Stopping is the
// closest honest answer, and saying so on serial beats silently doing nothing.
void Car_backwards(uint8_t M1_speed, uint8_t M2_speed) {
  (void)M1_speed;
  (void)M2_speed;
  Serial.println("Car_backwards: this chassis has no reverse (no H-bridge) -- stopping");
  motor_write(M1_CH, 0);
  motor_write(M2_CH, 0);
}

// Turns are a forward pivot: drive the outside wheel, let the inside one
// coast. On the keyestudio car these spun on the spot by running the wheels in
// opposite directions, which is not available here -- so the car swings round
// the stationary wheel instead and needs roughly twice the room to do it.
void Car_left(uint8_t M1_speed, uint8_t M2_speed) {
  (void)M2_speed;
  motor_write(M1_CH, M1_speed);  // right wheel drives
  motor_write(M2_CH, 0);
}

void Car_right(uint8_t M1_speed, uint8_t M2_speed) {
  (void)M1_speed;
  motor_write(M1_CH, 0);
  motor_write(M2_CH, M2_speed);  // left wheel drives
}

void Car_stop() {
  motor_write(M1_CH, 0);
  motor_write(M2_CH, 0);
}

#endif
