/*!
 * @file Car_Control.h
 * @brief Car Control - Analysis of Control Systems
 * @n Header file for Car Control
 *
 * @author [Demian](A01368643@tec.mx)
 * @author [Max](A01552369@tec.mx)
 * @version  V1.0.0
 * @date  2023-10-14
 */

#include "Arduino.h"
#include "PID_Control.h"

#ifndef Car_Control_cpp
    #define Car_Control_cpp

// Pin for servo
#define Servo_PWM 5

// Pins for line follower
#define sensorIzqPin 10 
#define sensorDerPin 12
#define sensorCentralPin 11

// Speed calc. constants
#define radio 3.2 // cm
#define pulse_per_rev 543
#define rads_per_rev 6.2832

// Pins for H-Bridge
#define IN1 8 //Forward-Backward motor
#define IN2 9 //Forward-Backward motor
#define ENA 6 //pwm 
#define encoderPinA 2 
#define encoderPinB 3


extern HardwareSerial Serial;

class Car_Control {
public:

//Controller-related variables PRUEBA 1
// float Kp = 118.73;
// float Ki = 40.06;
// float Kd = 45.68;


float Kp = 198.73;
float Ki = 100;
float Kd = 10;

float sample_time = 0.1;
float u_max = 255;
float u_min = 70;

PID_Control pid;

//Data from sensor 0=White, 1=Black
int sensIzq; 
int sensDer; 
int sensCent;

//Servo control Variables
int sens = 0;
int dir = 0;
int servo_val = 90;
int servo_last = 90;

// Variables for tracking encoder state
volatile long encoderCount = 0;
volatile long lastEncoderCount = 0;
unsigned long lastUpdateTime = 0;
unsigned long lastUpdateSpeed = 0;
unsigned long currentTime = 0;

unsigned long currentTimeMot = 0;
unsigned long lastUpdateTimeMot = 0;
unsigned long currentTimePrint = 0;
unsigned long lastUpdateTimePrint = 0;
int index = 0;
float dtRand = 1000;
bool rand = false;

int changeSpeedTim = 100;
float dt = 50;
float print_dt = 200;

// Motorreductor pwm signal
int pwm = 0;
float pwm_r = 0;
float pwm_d = 0.0;
float setpoint = 0;
float temp = 0;

// Message transmission
char buf[13];
int bufR[3];

// Variables for calculating speed
double speed = 0.0;
double motor_speed = 0.0;

Car_Control();
int updateDirection();
int updateServo(int);
void updateMotorSpeed();
void readSpeed();
};

#endif
