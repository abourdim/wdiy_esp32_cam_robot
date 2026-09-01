#ifndef SETMOTOR_H
#define SETMOTOR_H
#include "Arduino.h"

// --- What this board actually has -------------------------------------------
// The keyestudio car this code was ported from drove an I2C motor controller
// at 0x30 on GPIO14/13, which took a PWM byte and a direction byte per motor.
// There is no such chip on esp32_cam_plugin_robot. Each motor is a single
// low-side N-MOSFET (Q1/Q2, 2N7002) with its gate straight on a GPIO, the
// motor sitting between the screw terminal and the battery rail (VCC), and a
// 1N4007 (D1/D2) across it for flyback:
//
//        VCC ---+--------+
//               |        |
//              [M]      /|\  D1/D2 (cathode to VCC)
//               |        |
//        J1/J2 -+--------+
//               |
//               D
//    GPIO --[  Q1/Q2 2N7002
//               S
//              GND
//
// One switch per motor is one degree of freedom per motor: how hard it is
// driven, and nothing else. There is no H-bridge, so THIS ROBOT HAS NO
// REVERSE. That single fact shapes everything downstream -- Car_backwards()
// below, the D-pad's Back button, the joystick mixing, and both autonomous
// modes in app_server.h all had to give something up for it, and each says so
// where it happens.
//
// R1/R2 (10k) hold both gates down. On GPIO12 that is not merely tidy: GPIO12
// is MTDI, which the ESP32 samples at reset to choose its flash voltage. Left
// floating -- or worse, pulled up -- the module comes up expecting 1.8V flash
// and does not boot. The pulldown is the entire reason GPIO12 is usable as a
// motor output here, so do not remove R2 and do not drive this pin high from
// anything that runs before the app does.
#define M1_PIN 13  // J1 screw terminal -- gate of Q1
#define M2_PIN 12  // J2 screw terminal -- gate of Q2 (MTDI, see above)

// LEDC channels 4/5, deliberately not 0/1: the camera driver claims
// LEDC_CHANNEL_0 / LEDC_TIMER_0 for its XCLK (see the camera config in
// 4_CamRobot.ino), and channels share timers in pairs, so anything on 0/1
// would be fighting the sensor clock. Channels 4 and 5 sit on timer 2, which
// nothing else in this firmware wants.
#define M1_CH 4
#define M2_CH 5

// 20kHz: above hearing, so the motors don't whine at part throttle. A 2N7002's
// gate charge is tiny, so switching this fast costs nothing measurable.
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
// pwmvalue: 0..255. Zero is a coast, not a brake -- the MOSFET simply stops
//           conducting and the motor freewheels through its flyback diode.
//           There is no way to short the windings on this board, so the robot
//           always rolls a little past where you let go.
//
// No mutex here, unlike the version this was ported from. There, motor writes
// went over an I2C bus shared with a screen and had to take turns; here the
// motors are two dedicated PWM pins that no other task touches, and
// ledcWrite() is a single register write.
void motor_write(uint8_t motor, uint8_t pwmvalue) {
  ledcWrite(motor, pwmvalue);
}

// --- Drive mirror -----------------------------------------------------------
// What the robot was last ASKED to do, as PWM per side, recorded at the moment
// the command is applied. Nothing reads these now that the screen and its face
// are gone; they are kept because every drive path already sets them and they
// are the obvious hook for anything that later wants to know what the wheels
// were told to do.
volatile int lastDriveL = 0;
volatile int lastDriveR = 0;
// Steering INTENT, -1 left / 0 straight / +1 right, recorded separately rather
// than worked back out of the two wheel values. On a forward-only chassis a
// turn is "one wheel driven, one wheel not", and which wheel stopped is not
// something you want to be inferring from a pair of PWM values at the far end
// of the program. Each call site already knows which way it meant to turn.
volatile int lastDriveSteer = 0;
volatile unsigned long lastDriveCmdAt = 0;

static inline void motorMirror(int l, int r, int steer) {
  lastDriveL = l;
  lastDriveR = r;
  lastDriveSteer = steer;
  if (l != 0 || r != 0) lastDriveCmdAt = millis();
}

// M1 is the RIGHT motor and M2 the LEFT one. Nothing on the PCB says so -- the
// silkscreen reads J1 and J2 and stops there -- but every steering decision in
// app_server.h assumes it, inherited from the mixing convention of the car this
// came from. Wire the terminals to match. If the robot turns the wrong way, the
// fix is to swap the two plugs, not to edit this file.
void Car_forward() {
  motorMirror(255, 255, 0);
  motor_write(M1_CH, 255);
  motor_write(M2_CH, 255);
}

// Kept so this file still presents the same five verbs as every other WDIY
// car, and so a lesson has something to point at when explaining why. It
// cannot do what its name says: reversing needs an H-bridge and this board has
// a single low-side switch per side. Stopping is the closest honest answer,
// and saying so on serial beats silently doing nothing.
void Car_backwards() {
  Serial.println("Car_backwards: this chassis has no reverse (no H-bridge) -- stopping");
  motorMirror(0, 0, 0);
  motor_write(M1_CH, 0);
  motor_write(M2_CH, 0);
}

// Turns are a forward pivot around the undriven wheel, not a spin on the spot:
// with no reverse there is no way to run one side backwards. The robot swings
// through a much bigger arc than the keyestudio car did and needs the room in
// front of it to do so.
void Car_left() {
  motorMirror(255, 0, -1);
  motor_write(M1_CH, 255);  // right wheel drives
  motor_write(M2_CH, 0);
}

void Car_right() {
  motorMirror(0, 255, +1);
  motor_write(M1_CH, 0);
  motor_write(M2_CH, 255);  // left wheel drives
}

void Car_stop() {
  // Every path to a standstill goes through here -- the Stop button, the
  // joystick returning to centre, and loop()'s failsafe -- so this one line is
  // enough to keep the face's idea of "driving" honest.
  motorMirror(0, 0, 0);
  motor_write(M1_CH, 0);
  motor_write(M2_CH, 0);
}

#endif
