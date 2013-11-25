#ifndef _balanduino_h_
#define _balanduino_h_

#if ARDUINO < 154 // Make sure the newest version of the Arduino IDE is used
#error "Please update the Arduino IDE to the newest version: http://arduino.cc/en/Main/Software"
#endif

#include <stdint.h> // Needed for uint8_t, uint16_t etc.

/* Firmware Version Information */
const char *version = "1.0.0";
const uint8_t eepromVersion = 1; // EEPROM version - used to restore the EEPROM values if the configuration struct have changed

bool sendIMUValues, sendSettings, sendInfo, sendStatusReport, sendMainPIDValues, sendEncoderPIDValues, sendPairConfirmation, sendKalmanValues; // Used to send out different values via Bluetooth

const uint16_t PWM_FREQUENCY = 20000; // The motor driver can handle a PWM frequency up to 20kHz
const uint16_t PWMVALUE = F_CPU / PWM_FREQUENCY / 2; // The frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, prescaling is used so the frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2

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
const uint8_t leftEncoder1 = 15; // PD2
const uint8_t leftEncoder2 = 30; // PA6
const uint8_t rightEncoder1 = 16; // PD3
const uint8_t rightEncoder2 = 31; // PA7

#define leftEncoder1Port PIND
#define leftEncoder1Mask (1 << PIND2)
#define leftEncoder2Port PINA
#define leftEncoder2Mask (1 << PINA6)

#define rightEncoder1Port PIND
#define rightEncoder1Mask (1 << PIND3)
#define rightEncoder2Port PINA
#define rightEncoder2Mask (1 << PINA7)

volatile int32_t leftCounter = 0;
volatile int32_t rightCounter = 0;

const uint8_t buzzer = 5; // Buzzer used for feedback, it can be disconnected using the jumper

double batteryVoltage; // Measured battery level
uint8_t batteryCounter; // Counter used to check if it should check the battery level

bool ledState; // Last state of the built in LED

// PID Values struct
typedef struct {
  double Kp;
  double Ki;
  double Kd;
} PIDValues;

// This struct will store all the configuration values stored in the EEPROM
typedef struct {
  PIDValues mainPID;
  PIDValues encoderPID;

  double targetAngle; // Resting angle of the robot
  uint8_t backToSpot; // Set whenever the robot should stay in the same spot
  uint8_t controlAngleLimit; // Set the maximum tilting angle of the robot
  uint8_t turningLimit; // Set the maximum turning value
  double Qangle, Qbias, Rmeasure; // Kalman filter values
  double accYzero, accZzero; // Accelerometer zero values
  double leftMotorScaler, rightMotorScaler;
} cfg_t;

extern cfg_t cfg;

/* EEPROM Address Definitions */
const uint8_t initFlagsAddr = 0; // Set the first three bytes to the EEPROM version
const uint8_t configAddr = 1; // Save the configuration starting from this location

double lastRestAngle; // Used to limit the new restAngle if it's much larger than the previous one

/* IMU Data */
int16_t accY, accZ, gyroX;
double gyroXzero;
uint8_t i2cBuffer[8]; // Buffer for I2C data

// Results
double accAngle, gyroRate, gyroAngle;
double pitch;

double PIDValue, PIDLeft, PIDRight;
double restAngle;

/* Used for timing */
uint32_t kalmanTimer; // Timer used for the Kalman filter
uint32_t pidTimer; // Timer used for the PID loop
uint32_t encoderTimer; // Timer used used to determine when to update the encoder values
uint32_t imuTimer; // This is used to set a delay between sending IMU values
uint32_t reportTimer; // This is used to set a delay between sending report values
uint32_t ledTimer; // Used to update the LEDs to indicate battery level on the PS3, Wii and Xbox controllers
uint32_t blinkTimer; // Used to blink the built in LED, starts blinking faster upon an incoming Bluetooth request

/* Used to rumble controllers upon connection */
bool ps3Initialized, wiiInitialized, xboxInitialized; // These are used to check if a controller has been initialized
bool ps3RumbleEnable, wiiRumbleEnabled; // These are used to turn rumble off again on the Wiimote and to turn on rumble on the PS3 controller
bool ps3RumbleDisable, xboxRumbleDisable; // Used to turn rumble off again on the PS3 and Xbox controller

/* Direction set by the controllers or the SPP library */
bool steerForward, steerBackward, steerLeft, steerRight;
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
double targetPosition; // The encoder position the robot should be at
double wheelPosition;

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
void readSPPData();
void readUsb();
void updateLEDs();
void onInit();
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
void setPWM(uint8_t pin, uint16_t dutyCycle);
void stopAndReset();
void leftEncoder();
void rightEncoder();
int32_t readLeftEncoder();
int32_t readRightEncoder();
int32_t getWheelsPosition();

void checkSerialData();
void printMenu();
void calibrateMotor();
void testMotorSpeed(double *leftSpeed, double *rightSpeed, double leftScaler, double rightScaler);
void calibrateAcc();
void printValues();
void setValues(char *input);
void calibrateGyro();
bool checkMinMax(int16_t *array, uint8_t length, int16_t maxDifference);

#endif
