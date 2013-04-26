#ifndef _balanduino_h_
#define _balanduino_h_

#include <stdint.h> // Needed for uint8_t, uint16_t etc.

/* Firmware Version Information */
const char* version = "0.9.0";

char stringBuf[30];
char convBuf[10];

bool sendData;
bool sendSettings;
bool sendInfo;
bool sendPIDValues;
bool sendPairConfirmation;

const uint16_t PWM_FREQUENCY = 20000; // The motor driver can handle a pwm frequency up to 20kHz
const uint16_t PWMVALUE = F_CPU/PWM_FREQUENCY/2; // The frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, we use no prescaling so the frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2

/* Used to make commands more readable */
uint8_t lastCommand; // This is used set a new targetPosition
enum Command {
  updatePS3,
  updateWii,
  updateXbox,
  stop,
  forward,
  backward,
  left,
  right,
  imu,
  joystick,
};

// These are used to read and write to the port registers - see http://www.arduino.cc/en/Reference/PortManipulation
// I do this to save processing power - see this page for more information: http://www.billporter.info/ready-set-oscillate-the-fastest-way-to-change-arduino-pins/
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#define pwmPortDirection DDRD

/* Left motor */
#define leftPort PORTC
#define leftPortDirection DDRC

#define leftA PINC6 // PC6 - M1A - pin 23
#define leftB PINC7 // PC7 - M1B - pin 24
#define leftPWM PIND5 // PD5 - PWM1A (OC1A) - pin 18

/* Right motor */
#define rightPort PORTB
#define rightPortDirection DDRB

#define rightA PINB0 // PB0 - M2A - pin 25
#define rightB PINB1 // PB1 - M2B - pin 26
#define rightPWM PIND4 // PD4 - PWM1B (OC1B) - pin 17

/* Pins connected to the motor drivers enable pins */
const uint8_t leftEnable = 21;
const uint8_t rightEnable = 22;

/* Encoders */
const uint8_t leftEncoder1 = 15;
const uint8_t leftEncoder2 = 30;
const uint8_t rightEncoder1 = 16;
const uint8_t rightEncoder2 = 31;

volatile int32_t leftCounter = 0;
volatile int32_t rightCounter = 0;

const uint8_t buzzer = 5; // Buzzer used for feedback, it can be disconected using the jumper
uint8_t batteryLevel = 100; // Battery Percentage

/* EEProm Address Definitions */
const uint8_t InitializationFlagsAddr = 0;
const uint8_t BackToSpotAddr = 3;
const uint8_t controlAngleLimitAddr = 4;
const uint8_t turningAngleLimitAddr = 5;
const uint8_t KpAddr = 6+sizeof(double)*0; // A double is 4-bytes long inside an avr, so we will reserve four bytes for each value 
const uint8_t KiAddr = 6+sizeof(double)*1;
const uint8_t KdAddr = 6+sizeof(double)*2;
const uint8_t targetAngleAddr = 6+sizeof(double)*3;

/* IMU Data */
int16_t accY;
int16_t accZ;
int16_t gyroX;

// Results
double accAngle;
double gyroRate;
double gyroAngle;
double pitch;

/* PID variables */
const double defaultKp = 10;
const double defaultKi = 2;
const double defaultKd = 3;
const double defaultTargetAngle = 180;

double Kp;
double Ki;
double Kd;
double targetAngle;
double lastRestAngle = defaultTargetAngle; // Used to limit the new restAngle if it's much larger than the previous one

double lastError; // Store last angle error
double integratedError; // Store integrated error

double error;
double pTerm, iTerm, dTerm;
double PIDValue, PIDLeft, PIDRight;

/* Used for timing */
uint32_t kalmanTimer; // Timer used for the Kalman filter
uint32_t pidTimer; // Timer used for the PID loop
uint32_t encoderTimer; // Timer used used to determine when to update the encoder values
uint32_t dataTimer; // This is used so it doesn't send data to often
uint32_t wiiTimer; // This is used to read the USB endpoint more often when it's connected to a Wiimote
uint32_t ledTimer; // Used to update the LEDs to indicate battery level on the PS3, Wii and Xbox controllers

/* Used to rumble controllers upon connection */
bool ps3Rumble; // These are used to check if a controller has connected
bool wiiRumble;
bool xboxRumble;
bool ps3RumbleEnable; // These are used to turn rumble off again except for the PS3 controller which is turned on
bool wiiRumbleEnabled;
bool xboxRumbleEnabled;
bool ps3RumbleDisable; // Used to turn rumble off again on the PS3 controller

/* Direction set by the controllers or SPP library */
bool steerForward;
bool steerBackward;
bool steerStop = true; // Stop by default
bool steerLeft;
bool steerRight;

bool stopped; // This is used to set a new target position after braking

bool layingDown = true; // Use to indicate if the robot is laying down

const uint8_t defaultBackToSpot = 1;
uint8_t BackToSpot = defaultBackToSpot;

const uint8_t defaultControlAngleLimit = 8;
uint8_t controlAngleLimit = defaultControlAngleLimit;

const uint8_t defaultTurningAngleLimit = 20;
uint8_t turningAngleLimit = defaultTurningAngleLimit;

double targetOffset = 0; // Offset for going forward and backward
double turningOffset = 0; // Offset for turning left and right

// Data send via SPP
double sppData1 = 0;
double sppData2 = 0;

bool commandSent = false; // This is used so multiple controller can be used at once

uint32_t SPPreceiveControlTimestamp;
const uint16_t SPPreceiveControlTimeout = 500; // After how long time should we prioritize the other controllers instead of the serial control

int32_t wheelPosition; // Wheel position based on encoder readings
int32_t lastWheelPosition; // Used to calculate the wheel velocity
int32_t wheelVelocity; // Wheel velocity based on encoder readings
int32_t targetPosition; // The encoder position the robot should be at

const uint16_t zoneA = 8000;
const uint16_t zoneB = 4000;
const uint16_t zoneC = 1000;
const double positionScaleA = 600; // One resolution is 928 pulses per encoder
const double positionScaleB = 800;
const double positionScaleC = 1000;
const double positionScaleD = 500;
const double velocityScaleMove = 70;
const double velocityScaleStop = 60;
const double velocityScaleTurning = 70;
#endif
