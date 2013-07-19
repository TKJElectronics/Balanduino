#ifndef _balanduino_h_
#define _balanduino_h_

#include <stdint.h> // Needed for uint8_t, uint16_t etc.
#include <string.h> // Needed for strlen

/* Firmware Version Information */
const char *version = "0.9.0";
const char *eepromVersion = "001"; // EEPROM version - used to restore the EEPROM values if the configuration values have changed

bool sendData;
bool sendSettings;
bool sendInfo;
bool sendPIDValues;
bool sendPairConfirmation;
bool sendKalmanValues;

const uint16_t PWM_FREQUENCY = 20000; // The motor driver can handle a pwm frequency up to 20kHz
const uint16_t PWMVALUE = F_CPU/PWM_FREQUENCY/2; // The frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, prescaling is used so the frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2

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

const uint8_t buzzer = 5; // Buzzer used for feedback, it can be disconnected using the jumper
double batteryVoltage; // Measured battery level

bool ledState; // Last state of the built in LED

// This struct will store all the configuration values
typedef struct {
  // PID variables
  double P;
  double I;
  double D;
  
  double targetAngle; // Resting angle of the robot
  uint8_t backToSpot; // Set whenever the robot should stay in the same spot
  uint8_t controlAngleLimit; // Set the maximum tilting angle of the robot
  uint8_t turningLimit; // Set the maximum turning value

  double Qangle;
  double Qbias;
  double Rmeasure;
} cfg_t;

extern cfg_t cfg;

/* EEPROM Address Definitions */
const uint8_t initFlagsAddr = 0; // Set the first three bytes to the EEPROM version
const uint8_t configAddr = strlen(eepromVersion); // Save the configuration starting from this location

double lastRestAngle; // Used to limit the new restAngle if it's much larger than the previous one

/* IMU Data */
int16_t accY;
int16_t accZ;
int16_t gyroX;

uint8_t i2cBuffer[8]; // Buffer for I2C data

// Results
double accAngle;
double gyroRate;
double gyroAngle;
double pitch;

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
uint32_t ledTimer; // Used to update the LEDs to indicate battery level on the PS3, Wii and Xbox controllers
uint32_t blinkTimer; // Used to blink the built in LED, starts blinking faster upon an incoming Bluetooth request

/* Used to rumble controllers upon connection */
bool ps3Initialized, wiiInitialized, xboxInitialized; // These are used to check if a controller has been initialized
bool ps3RumbleEnable, wiiRumbleEnabled, xboxRumbleEnabled; // These are used to turn rumble off again except for the PS3 controller which is turned on
bool ps3RumbleDisable; // Used to turn rumble off again on the PS3 controller

/* Direction set by the controllers or SPP library */
bool steerForward;
bool steerBackward;
bool steerStop = true; // Stop by default
bool steerLeft;
bool steerRight;

bool stopped; // This is used to set a new target position after braking

bool layingDown = true; // Use to indicate if the robot is laying down

double targetOffset = 0; // Offset for going forward and backward
double turningOffset = 0; // Offset for turning left and right

// Data send via SPP
double sppData1 = 0;
double sppData2 = 0;

bool commandSent = false; // This is used so multiple controller can be used at once

uint32_t receiveControlTimer;
const uint16_t receiveControlTimeout = 500; // After how long time should it should prioritize the other controllers instead of the serial control

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

// Function prototypes
void sendBluetoothData();
void readSPPData();
void readUsb();
void updateLEDs();
void onInit();
void steer(Command command);
double scale(double input, double inputMin, double inputMax, double outputMin, double outputMax);

void checkInitializationFlags();
void readEEPROMValues();
void updateConfig();
void restoreEEPROMValues();

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop);
uint8_t i2cWrite(uint8_t registerAddress, uint8_t* data, uint8_t length, bool sendStop);
uint8_t i2cRead(uint8_t registerAddress, uint8_t* data, uint8_t nbytes);

void moveMotor(Command motor, Command direction, double speedRaw);
void stopMotor(Command motor);
void setPWM(uint8_t pin, uint16_t dutyCycle);
void stopAndReset();
void leftEncoder();
void rightEncoder();
int32_t readLeftEncoder();
int32_t readRightEncoder();

void PID(double restAngle, double offset, double turning, double dt);

#endif
