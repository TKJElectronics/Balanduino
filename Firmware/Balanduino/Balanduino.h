/* Copyright (C) 2013-2015 Kristian Lauszus, TKJ Electronics. All rights reserved.

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

#if ARDUINO < 156 // Make sure that at least Arduino IDE version 1.5.6 is used
  #error "Please update the Arduino IDE to version 1.5.6 or newer at the following website: http://arduino.cc/en/Main/Software"
#endif

#include <stdint.h> // Needed for uint8_t, uint16_t etc.

/* Firmware Version Information */
static const char *version = "1.2.0";
static const uint8_t eepromVersion = 3; // EEPROM version - used to restore the EEPROM values if the configuration struct have changed

static bool sendIMUValues, sendSettings, sendInfo, sendStatusReport, sendPIDValues, sendPairConfirmation, sendKalmanValues; // Used to send out different values via Bluetooth

static const uint16_t PWM_FREQUENCY = 20000; // The motor driver can handle a PWM frequency up to 20kHz
static const uint16_t PWMVALUE = F_CPU / PWM_FREQUENCY / 2; // The frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, prescaling is used so the frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2

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
} lastCommand; // This is used set a new targetPosition

// These pins macros are defined in avrpins.h in the USB Host library. This allows to read and write directly to the port registers instead of using Arduino's slow digitalRead()/digitalWrite() functions
// The source is available here: https://github.com/felis/USB_Host_Shield_2.0/blob/master/avrpins.h
// I do this to save processing power - see this page for more information: http://www.billporter.info/ready-set-oscillate-the-fastest-way-to-change-arduino-pins/
// Also see the Arduino port manipulation guide: http://www.arduino.cc/en/Reference/PortManipulation

/* Left motor */
#define leftA P23
#define leftB P24
#define leftPWM P18

/* Right motor */
#if BALANDUINO_REVISION < 13
  #define rightA P25
  #define rightB P26
#else
  #define rightA P15
  #define rightB P16
#endif
#define rightPWM P17

/* Pins connected to the motor drivers diagnostic pins */
#define leftDiag P21
#define rightDiag P22

/* Encoders */
#if BALANDUINO_REVISION < 13
  #define leftEncoder1Pin 15 // Used for attachInterrupt
  #define leftEncoder2Pin 30 // Used for pin change interrupt
  #define rightEncoder1Pin 16 // Used for attachInterrupt
  #define rightEncoder2Pin 31 // Used for pin change interrupt
#else
  #define leftEncoder1Pin 25 // Used for pin change interrupt
  #define leftEncoder2Pin 26 // Used for pin change interrupt
  #define rightEncoder1Pin 30 // Used for pin change interrupt
  #define rightEncoder2Pin 31 // Used for pin change interrupt
#endif

#define MAKE_PIN(pin) _MAKE_PIN(pin) // Puts a P in front of the pin number, e.g. 1 becomes P1
#define _MAKE_PIN(pin) P ## pin

#define leftEncoder1 MAKE_PIN(leftEncoder1Pin)
#define leftEncoder2 MAKE_PIN(leftEncoder2Pin)

#define rightEncoder1 MAKE_PIN(rightEncoder1Pin)
#define rightEncoder2 MAKE_PIN(rightEncoder2Pin)

// You should change these to match your pins
#if BALANDUINO_REVISION < 13
  #define PIN_CHANGE_INTERRUPT_VECTOR_LEFT PCINT0_vect
  #define PIN_CHANGE_INTERRUPT_VECTOR_RIGHT PCINT0_vect
#else
  #define PIN_CHANGE_INTERRUPT_VECTOR_LEFT PCINT1_vect
  #define PIN_CHANGE_INTERRUPT_VECTOR_RIGHT PCINT0_vect
#endif

// Buzzer used for feedback, it can be disconnected using the jumper
#if BALANDUINO_REVISION < 13
  #define buzzer P5
#else
  #define buzzer P11 /* A4 */
#endif

#define spektrumBindPin P0 // Pin used to bind with the Spektrum satellite receiver - you can use any pin while binding, but you should connect it to RX0 afterwards

#define LED MAKE_PIN(LED_BUILTIN) // LED_BUILTIN is defined in pins_arduino.h in the hardware add-on

#define VBAT A5 // Not broken out - used for battery voltage measurement

/* Counters used to count the pulses from the encoders */
static volatile int32_t leftCounter = 0;
static volatile int32_t rightCounter = 0;

static float batteryVoltage; // Measured battery level
static uint8_t batteryCounter; // Counter used to check if it should check the battery level

// This struct will store all the configuration values
typedef struct {
  float P, I, D; // PID variables
  float targetAngle; // Resting angle of the robot
  uint8_t backToSpot; // Set whenever the robot should stay in the same spot
  uint8_t controlAngleLimit; // Set the maximum tilting angle of the robot
  uint8_t turningLimit; // Set the maximum turning value
  float Qangle, Qbias, Rmeasure; // Kalman filter values
  float accYzero, accZzero; // Accelerometer zero values
  float leftMotorScaler, rightMotorScaler;
  bool bindSpektrum;
} cfg_t;

extern cfg_t cfg;

/* EEPROM Address Definitions */
static const uint8_t initFlagsAddr = 0; // Set the first byte to the EEPROM version
static const uint8_t configAddr = 1; // Save the configuration starting from this location

static float lastRestAngle; // Used to limit the new restAngle if it's much larger than the previous one

/* IMU Data */
static float gyroXzero;
static uint8_t i2cBuffer[8]; // Buffer for I2C data

// Results
static float accAngle, gyroAngle; // Result from raw accelerometer and gyroscope readings
static float pitch; // Result from Kalman filter

static float lastError; // Store last angle error
static float iTerm; // Store iTerm

/* Used for timing */
static uint32_t kalmanTimer; // Timer used for the Kalman filter
static uint32_t pidTimer; // Timer used for the PID loop
static uint32_t imuTimer; // This is used to set a delay between sending IMU values
static uint32_t encoderTimer; // Timer used used to determine when to update the encoder values
static uint32_t reportTimer; // This is used to set a delay between sending report values
static uint32_t ledTimer; // Used to update the LEDs to indicate battery level on the PS3, PS4, Wii and Xbox controllers
static uint32_t blinkTimer; // Used to blink the built in LED, starts blinking faster upon an incoming Bluetooth request

/* Used to rumble controllers upon connection */
static bool ps3RumbleEnable, wiiRumbleEnabled, ps4RumbleEnabled; // These are used to turn rumble off again on the Wiimote and PS4 controller and to turn on rumble on the PS3 controller
static bool ps3RumbleDisable, xboxRumbleDisable; // Used to turn rumble off again on the PS3 and Xbox controller

static bool steerStop = true; // Stop by default
static bool stopped; // This is used to set a new target position after braking

static bool layingDown = true; // Use to indicate if the robot is laying down

static float targetOffset = 0.0f; // Offset for going forward and backward
static float turningOffset = 0.0f; // Offset for turning left and right

static char dataInput[30]; // Incoming data buffer
static bool bluetoothData; // True if data received is from the Bluetooth connection
static float sppData1, sppData2; // Data send via SPP connection

static bool commandSent = false; // This is used so multiple controller can be used at once

static uint32_t receiveControlTimer;
static const uint16_t receiveControlTimeout = 500; // After how long time should it should prioritize the other controllers instead of the serial control

static int32_t lastWheelPosition; // Used to calculate the wheel velocity
static int32_t wheelVelocity; // Wheel velocity based on encoder readings
static int32_t targetPosition; // The encoder position the robot should be at

// Variables used for Spektrum receiver
extern uint16_t rcValue[]; // Channel values
static bool spekConnected; // True if spektrum receiver is connected
static uint32_t spekConnectedTimer; // Timer used to check if the connection is dropped

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
  static const uint16_t zoneA = 8000 * 2;
  static const uint16_t zoneB = 4000 * 2;
  static const uint16_t zoneC = 1000 * 2;
  static const float positionScaleA = 600.0f * 2.0f; // One resolution is 1856 pulses per encoder
  static const float positionScaleB = 800.0f * 2.0f;
  static const float positionScaleC = 1000.0f * 2.0f;
  static const float positionScaleD = 500.0f * 2.0f;
  static const float velocityScaleMove = 70.0f * 2.0f;
  static const float velocityScaleStop = 60.0f * 2.0f;
  static const float velocityScaleTurning = 70.0f * 2.0f;
#else
  static const uint16_t zoneA = 8000;
  static const uint16_t zoneB = 4000;
  static const uint16_t zoneC = 1000;
  static const float positionScaleA = 600.0f; // One resolution is 928 pulses per encoder
  static const float positionScaleB = 800.0f;
  static const float positionScaleC = 1000.0f;
  static const float positionScaleD = 500.0f;
  static const float velocityScaleMove = 70.0f;
  static const float velocityScaleStop = 60.0f;
  static const float velocityScaleTurning = 70.0f;
#endif

// Function prototypes
// We use inline to eliminates the size and speed overhead of calling and returning from a function that is only used once.
static inline void readSPPData();
static inline void readUsb();
static void updateLEDs();
static void onInitPS3();
static void onInitPS4();
static void onInitWii();
static void onInitXbox();
static void steer(Command command);
static float scale(float input, float inputMin, float inputMax, float outputMin, float outputMax);

static inline bool checkInitializationFlags();
static inline void readEEPROMValues();
static void updateConfig();
static void restoreEEPROMValues();

static uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop);
static uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop);
static uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes);

static inline void updatePID(float restAngle, float offset, float turning, float dt);
static void moveMotor(Command motor, Command direction, float speedRaw);
static void stopMotor(Command motor);
static inline void setPWM(Command motor, uint16_t dutyCycle);
static void stopAndReset();
// On newer versions of the PCB these two functions are only used in one place, so they will be inlined by the compiler.
static inline void leftEncoder();
static inline void rightEncoder();
static int32_t readLeftEncoder();
static int32_t readRightEncoder();
int32_t getWheelsPosition();

static inline void bindSpektrum();
static void readSpektrum(uint8_t input);

static inline void checkSerialData();
static void printMenu();
static inline void calibrateMotor();
static void testMotorSpeed(float *leftSpeed, float *rightSpeed, float leftScaler, float rightScaler);
static inline void calibrateAcc();
static inline void printValues();
static void setValues(char *input);
static inline bool calibrateGyro();
static bool checkMinMax(int16_t *array, uint8_t length, int16_t maxDifference);

#endif
