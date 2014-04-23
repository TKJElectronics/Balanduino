/* Copyright (C) 2013-2014 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
*/

#ifndef _balanduino_h_
#define _balanduino_h_

#if ARDUINO < 156 // Make sure the newest version of the Arduino IDE is used
#error "Please update the Arduino IDE to version 1.5.6 at the following website: http://arduino.cc/en/Main/Software"
#endif

#include <stdint.h> // Needed for uint8_t, uint16_t etc.

/* Firmware Version Information */
const char *version = "1.1.0";
const uint8_t eepromVersion = 2; // EEPROM version - used to restore the EEPROM values if the configuration struct have changed

bool sendIMUValues, sendSettings, sendInfo, sendStatusReport, sendPIDValues, sendPairConfirmation, sendKalmanValues; // Used to send out different values via Bluetooth

const uint16_t PWM_FREQUENCY = 20000; // The motor driver can handle a PWM frequency up to 20kHz
const uint16_t PWMVALUE = F_CPU / PWM_FREQUENCY / 2; // The frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, prescaling is used so the frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2

/* Used to make commands more readable */
enum Command {
  updatePS3,
  updatePS4,
  updateWii,
  updateXbox,
  updateSpektrum,
  stop,
  forward,
  backward,
  left,
  right,
  imu,
  joystick,
};
Command lastCommand; // This is used set a new targetPosition

// These pins macros are defined in avrpins.h in the USB Host library. This allows to read and write directly to the port registers instead of using Arduino's slow digitalRead()/digitalWrite() functions
// The source is available here: https://github.com/felis/USB_Host_Shield_2.0/blob/master/avrpins.h
// I do this to save processing power - see this page for more information: http://www.billporter.info/ready-set-oscillate-the-fastest-way-to-change-arduino-pins/
// Also see the Arduino port manipulation guide: http://www.arduino.cc/en/Reference/PortManipulation

/* Left motor */
#define leftA P23
#define leftB P24
#define leftPWM P18

/* Right motor */
#define rightA P25
#define rightB P26
#define rightPWM P17

/* Pins connected to the motor drivers diagnostic pins */
#define leftDiag P21
#define rightDiag P22

/* Encoders */
#define leftEncoder1 P15
#define leftEncoder2 P30

#define rightEncoder1 P16
#define rightEncoder2 P31

#define leftEncoder1Pin 15 // Used for attachInterrupt
#define rightEncoder1Pin 16
#define leftEncoder2Pin 30 // Used for pin change interrupt
#define rightEncoder2Pin 31

#define PIN_CHANGE_INTERRUPT_VECTOR_LEFT PCINT0_vect // You should change these to match your pins, if you are in doubt, just comment them out to disable them
#define PIN_CHANGE_INTERRUPT_VECTOR_RIGHT PCINT0_vect

#define buzzer P5 // Buzzer used for feedback, it can be disconnected using the jumper

#define spektrumBindPin P0 // Pin used to bind with the Spektrum satellite receiver - you can use any pin while binding, but you should connect it to RX0 afterwards

#define MAKE_PIN(pin) MAKE_PIN2(pin) // Puts a P in front of the pin number, e.g. 1 becomes P1
#define MAKE_PIN2(pin) P ## pin

#define LED MAKE_PIN(LED_BUILTIN) // LED_BUILTIN is defined in pins_arduino.h in the hardware add-on

#define VBAT A5 // Not broken out - used for battery voltage measurement

/* Counters used to count the pulses from the encoders */
volatile int32_t leftCounter = 0;
volatile int32_t rightCounter = 0;

double batteryVoltage; // Measured battery level
uint8_t batteryCounter; // Counter used to check if it should check the battery level

// This struct will store all the configuration values
typedef struct {
  double P, I, D; // PID variables
  double targetAngle; // Resting angle of the robot
  uint8_t backToSpot; // Set whenever the robot should stay in the same spot
  uint8_t controlAngleLimit; // Set the maximum tilting angle of the robot
  uint8_t turningLimit; // Set the maximum turning value
  double Qangle, Qbias, Rmeasure; // Kalman filter values
  double accYzero, accZzero; // Accelerometer zero values
  double leftMotorScaler, rightMotorScaler;
  bool bindSpektrum;
} cfg_t;

extern cfg_t cfg;

/* EEPROM Address Definitions */
const uint8_t initFlagsAddr = 0; // Set the first byte to the EEPROM version
const uint8_t configAddr = 1; // Save the configuration starting from this location

double lastRestAngle; // Used to limit the new restAngle if it's much larger than the previous one

/* IMU Data */
int16_t accY, accZ, gyroX;
double gyroXzero;
uint8_t i2cBuffer[8]; // Buffer for I2C data

// Results
double accAngle, gyroRate, gyroAngle;
double pitch;

double lastError; // Store last angle error
double integratedError; // Store integrated error

double error;
double pTerm, iTerm, dTerm;
double PIDValue, PIDLeft, PIDRight;

/* Used for timing */
uint32_t kalmanTimer; // Timer used for the Kalman filter
uint32_t pidTimer; // Timer used for the PID loop
uint32_t imuTimer; // This is used to set a delay between sending IMU values
uint32_t encoderTimer; // Timer used used to determine when to update the encoder values
uint32_t reportTimer; // This is used to set a delay between sending report values
uint32_t ledTimer; // Used to update the LEDs to indicate battery level on the PS3, PS4, Wii and Xbox controllers
uint32_t blinkTimer; // Used to blink the built in LED, starts blinking faster upon an incoming Bluetooth request

/* Used to rumble controllers upon connection */
bool ps3RumbleEnable, wiiRumbleEnabled, ps4RumbleEnabled; // These are used to turn rumble off again on the Wiimote and PS4 controller and to turn on rumble on the PS3 controller
bool ps3RumbleDisable, xboxRumbleDisable; // Used to turn rumble off again on the PS3 and Xbox controller

bool steerStop = true; // Stop by default
bool stopped; // This is used to set a new target position after braking

bool layingDown = true; // Use to indicate if the robot is laying down

double targetOffset = 0; // Offset for going forward and backward
double turningOffset = 0; // Offset for turning left and right

char dataInput[30]; // Incoming data buffer
bool bluetoothData; // True if data received is from the Bluetooth connection
double sppData1, sppData2; // Data send via SPP connection

bool commandSent = false; // This is used so multiple controller can be used at once

uint32_t receiveControlTimer;
const uint16_t receiveControlTimeout = 500; // After how long time should it should prioritize the other controllers instead of the serial control

int32_t lastWheelPosition; // Used to calculate the wheel velocity
int32_t wheelVelocity; // Wheel velocity based on encoder readings
int32_t targetPosition; // The encoder position the robot should be at

// Variables used for Spektrum receiver
extern uint16_t rcValue[]; // Channel values
bool spekConnected; // True if spektrum receiver is connected
uint32_t spekConnectedTimer; // Timer used to check if the connection is dropped

#define RC_CHAN_THROTTLE 0
#define RC_CHAN_ROLL     1
#define RC_CHAN_PITCH    2
#define RC_CHAN_YAW      3
#define RC_CHAN_AUX1     4
#define RC_CHAN_AUX2     5
#define RC_CHAN_AUX3     6
#if (SPEKTRUM == 2048) // 8 channels
#define RC_CHAN_AUX4     7
#endif

// Encoder values
#if defined(PIN_CHANGE_INTERRUPT_VECTOR_LEFT) && defined(PIN_CHANGE_INTERRUPT_VECTOR_RIGHT)
const uint16_t zoneA = 8000 * 2;
const uint16_t zoneB = 4000 * 2;
const uint16_t zoneC = 1000 * 2;
const double positionScaleA = 600 * 2; // One resolution is 1856 pulses per encoder
const double positionScaleB = 800 * 2;
const double positionScaleC = 1000 * 2;
const double positionScaleD = 500 * 2;
const double velocityScaleMove = 70 * 2;
const double velocityScaleStop = 60 * 2;
const double velocityScaleTurning = 70 * 2;
#else
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

// Function prototypes
void readSPPData();
void readUsb();
void updateLEDs();
void onInitPS3();
void onInitPS4();
void onInitWii();
void onInitXbox();
void steer(Command command);
double scale(double input, double inputMin, double inputMax, double outputMin, double outputMax);

bool checkInitializationFlags();
void readEEPROMValues();
void updateConfig();
void restoreEEPROMValues();

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop);
uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop);
uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes);

void updatePID(double restAngle, double offset, double turning, double dt);
void moveMotor(Command motor, Command direction, double speedRaw);
void stopMotor(Command motor);
void setPWM(Command motor, uint16_t dutyCycle);
void stopAndReset();
void leftEncoder();
void rightEncoder();
int32_t readLeftEncoder();
int32_t readRightEncoder();
int32_t getWheelsPosition();

void bindSpektrum();
void readSpektrum(uint8_t input);

void checkSerialData();
void printMenu();
void calibrateMotor();
void testMotorSpeed(double *leftSpeed, double *rightSpeed, double leftScaler, double rightScaler);
void calibrateAcc();
void printValues();
void setValues(char *input);
bool calibrateGyro();
bool checkMinMax(int16_t *array, uint8_t length, int16_t maxDifference);

#endif
