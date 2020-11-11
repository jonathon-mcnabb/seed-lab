#pragma once

#define ENCODER_OPTIMIZE_INTERRUPTS

// A & B pins for encoder 1
#define A_1_PIN 2
#define B_1_PIN 5

// A & B pins for encoder 2
#define A_2_PIN 3
#define B_2_PIN 6

#define ANGULAR_RESET_PIN 8

// Constants used in the calculations
// TODO: this should be 16*50, since only *one* pin is on an interrupt pin... :/
#define COUNTS_PER_ROTATION (64*50) // 64 if both A&B on interrupt pins, 16 else

// -------- Motor Defines ----------
#define MOTOR_1_PWM 9
#define MOTOR_1_DIRECTION 7

#define MOTOR_2_PWM 10
#define MOTOR_2_DIRECTION 8

#define ENABLE_PIN 4 // for turning off & on the motor
#define STATUS_FLAG 12

#define WHEEL_2_FORWARD HIGH
#define WHEEL_2_BACKWARD LOW

#define WHEEL_1_FORWARD LOW
#define WHEEL_1_BACKWARD HIGH

#define WHEEL_RADIUS 0.0762 // METERS, 3 inches
// outer width = 8 1/8 in
// inner width = 6 3/8 in
// average = 7.25 in -> 0.18415 Meters
// about 14.5 inches
#define DISTANCE_B_W_WHEELS (0.42)

#define PERIOD 10 // in MS
#define BATTERY_MAX_VOLTAGE 8.2

#define SET_RHO_DOT (PI * SET_RADIUS / SET_TIME)
#define SET_PHI_DOT (PI / SET_TIME) // = PI / t_0

// Set the values here.
// #define SET_PHI 0 // (PI*2) // = 180 degree turn
// #define SET_RHO 3.048 //(0.9144) // 4*0.3048 // in meters

// #define DEMO_2
// static double SET_PHI = (PI)*2.0;
// static double SET_RHO = 0;

static double SET_PHI = 0; //(PI*2.0);
static double SET_RHO = 3.048; //0.9144; // 0.3048 is movement in radians

#define EPSILON 0.5
#define TAU (6.28318530718)
#define NOMINAL_VOLTAGE 8
#define SUM_MAX 2
