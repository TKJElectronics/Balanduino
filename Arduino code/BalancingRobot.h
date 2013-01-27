#ifndef _balancingrobot_h_
#define _balancingrobot_h_

#include <stdint.h> // Needed for uint8_t

//#define PROMINI

char stringBuf[30];

bool sendData;
bool sendPIDValues;
uint8_t dataCounter;

#define PWM_FREQUENCY 20000 // The motor driver can handle a pwm frequency up to 20kHz
#define PWMVALUE F_CPU/PWM_FREQUENCY/2 // Frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, we use no prescaling so frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2

/* Used for the PS3 Communication and motor functions */
int lastCommand; // This is used set a new targetPosition
enum Command {
  stop,
  forward,
  backward,
  left,
  right,
  imu,
  joystick,
};

/* These are used to read and write to the port registers - see http://www.arduino.cc/en/Reference/PortManipulation 
 I do this to save processing power - see this page for more information: http://www.billporter.info/ready-set-oscillate-the-fastest-way-to-change-arduino-pins/ */
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

/* Left motor */
#ifdef PROMINI
#define leftPort PORTC
#define leftPortDirection DDRC
#define leftA PINC2 // PC2 - pin A2 (2INA on the Pololu motor driver)
#define leftB PINC3 // PC3 - pin A3 (2INB on the Pololu motor driver)
#else
#define leftPort PORTD
#define leftPortDirection DDRD
#define leftA PIND0 // PD0 - pin 0 (2INA on the Pololu motor driver)
#define leftB PIND1 // PD1 - pin 1 (2INB on the Pololu motor driver)
#endif

#define leftPwmPortDirection DDRB
#define leftPWM PINB1 // PB1 - pin 9 (OC1A) - (2PWM on the Pololu motor driver)

/* Right motor */
#define rightPort PORTC
#define rightPortDirection DDRC
#define rightPwmPortDirection DDRB

#define rightA PINC4 // PC4 - pin A4 (1INA on the Pololu motor driver)
#define rightB PINC5 // PC5 - pin A5 (1INB on the Pololu motor driver)
#define rightPWM PINB2 // PB2 - pin 10 (OC1B) - (1PWM on the Pololu motor driver)

/* 
  Note that the right motor is connected as so to the Pololu motor driver:
  Black wire - output 1A
  Red wire - output 1B
  And the left motor is connected as so to the Pololu motor driver:
  Red wire - output 2A
  Black wire - output 2B
*/  

/* Encoders */
#define leftEncoder1 2 // Yellow wire
#define leftEncoder2 4 // White wire
#define rightEncoder1 3 // White wire
#define rightEncoder2 5 // Yellow wire

volatile long leftCounter = 0;
volatile long rightCounter = 0;

/* IMU */
#define gyroY A0
#ifdef PROMINI
#define accY A6
#define accZ A7
#else
#define accY A1
#define accZ A2
#endif

#define buzzer 6 // Connected to a BC547 transistor - there is a protection diode at the buzzer as well

// Zero voltage values for the sensors - [0] = gyroY, [1] = accY, [2] = accZ
double zeroValues[3] = { 0 };

// Results
double accYangle;
double gyroYrate;
double gyroAngle;
double pitch;

/* PID variables */
double Kp = 7;
double Ki = 2;
double Kd = 8;
double targetAngle = 180;

double lastError; // Store position error
double iTerm; // Store integral term

/* Used for timing */
unsigned long timer;

#define STD_LOOP_TIME 10000 // Fixed time loop of 10 milliseconds
unsigned long lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime;

bool steerForward;
bool steerBackward;
bool steerStop = true; // Stop by default
bool steerLeft;
bool steerRight;

bool stopped; // This is used to set new target position after breaking

bool layingDown = true; // Use to indicate if the robot is laying down

double targetOffset = 0; // Offset for going forward and backward
double turningOffset = 0; // Offset for turning left and right

double sppData1 = 0;
double sppData2 = 0;

uint8_t loopCounter = 0; // Used to update wheel velocity
long wheelPosition;
long lastWheelPosition;
long wheelVelocity;
long targetPosition;
int zoneA = 4000;
int zoneB = 2000;
double positionScaleA = 250; // One resolution is 464 pulses
double positionScaleB = 500; 
double positionScaleC = 1000;
double velocityScaleMove = 35;
double velocityScaleStop = 30;
double velocityScaleTurning = 35;
#endif
