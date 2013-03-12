/*
 * The code is released under the GNU General Public License.
 * Developed by Kristian Lauszus, TKJ Electronics 2013
 * This is the algorithm for the Balanduino balancing robot.
 * It can be controlled by either an Android app or a Processing application via bluetooth.
 * The Android app can be found at the following link: https://github.com/TKJElectronics/BalanduinoAndroidApp
 * The Processing application can be found here: https://github.com/TKJElectronics/BalanduinoProcessingApp
 * It can also be controlled by a PS3, Wii or a Xbox controller
 * For details, see http://blog.tkjelectronics.dk/2012/03/the-balancing-robot/
 */

#include "Balanduino.h"
#include "EEPROMAnything.h"
#include <Wire.h>
#include <adk.h>
 
// These are all open source libraries written by Kristian Lauszus, TKJ Electronics
// The USB libraries are located at the following link: https://github.com/felis/USB_Host_Shield_2.0
#include <Kalman.h> // Kalman filter library - see: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/
#include <XBOXRECV.h>
#include <SPP.h>
#include <PS3BT.h>
#include <Wii.h>

// Create the Kalman library instance
Kalman kalman; // See https://github.com/TKJElectronics/KalmanFilter for source code

// This will take care of all USB communication
USB Usb;
// This is the main Bluetooth library, it will take care of all the usb and hci communication with the Bluetooth dongle
BTD Btd(&Usb); // Uncomment DEBUG in "BTD.cpp" to save space
// You have to connect a Xbox wireless receiver to the Arduino to control it with a wireless Xbox controller
XBOXRECV Xbox(&Usb); // Uncomment DEBUG in "XBOXRECV.cpp" to save space
// Implementation for the Android Open Accessory Protocol. Simply connect your phone to get redirected to the Play Store
ADK adk(&Usb,"TKJ Electronics", // Manufacturer Name
             "Balanduino", // Model Name
             "Android App for Balanduino", // Description - user visible string
             "0.4", // Version of the Android app
             "https://play.google.com/store/apps/details?id=com.tkjelectronics.balanduino", // URL - web page to visit if no installed apps support the accessory
             "1234"); // Serial Number - this is not used

// The SPP (Serial Port Protocol) emulates a virtual Serial port, which is supported by most computers and mobile phones
SPP SerialBT(&Btd,"Balanduino","0000"); // Also uncomment DEBUG in "SPP.cpp"
// This is the PS3 library. It supports all the three original controller: the Dualshock 3, Navigation and Move controller
PS3BT PS3(&Btd,0x00,0x15,0x83,0x3D,0x0A,0x57); // Also remember to uncomment DEBUG in "PS3BT.cpp" to save space
// The Wii library can communicate with Wiimotes and the Nunchuck and Motion Plus extension and finally the Wii U Pro Controller
WII Wii(&Btd); // Also uncomment DEBUG in "Wii.cpp"
// You have to pair with your Wiimote first by creating the instance like this and the press 1+2 on the Wiimote or press sync if you are using a Wii U Pro Controller - you only have to do this once
//WII Wii(&Btd,PAIR);
// Or you can simply send "CW;" to the robot to start the pairing sequence
// This can also be done using the Android application

void setup() {
  /* Initialize UART */
  Serial.begin(57600);
  
  /* Read the last PID values and target angle */
  readEEPROMValues();
  checkInitializationFlags();
  
  /* Setup encoders */
  pinMode(leftEncoder1,INPUT);
  pinMode(leftEncoder2,INPUT);
  pinMode(rightEncoder1,INPUT);
  pinMode(rightEncoder2,INPUT);
  attachInterrupt(0,leftEncoder,CHANGE);
  attachInterrupt(1,rightEncoder,CHANGE);

  /* Enable the motor drivers */
  pinMode(leftEnable, OUTPUT);
  pinMode(rightEnable, OUTPUT);
  digitalWrite(leftEnable, HIGH);
  digitalWrite(rightEnable, HIGH);

  /* Setup motor pins to output */
  sbi(pwmPortDirection,leftPWM);
  sbi(leftPortDirection,leftA);
  sbi(leftPortDirection,leftB);
  sbi(pwmPortDirection,rightPWM);
  sbi(rightPortDirection,rightA);
  sbi(rightPortDirection,rightB);

  /* Set PWM frequency to 20kHz - see the datasheet http://www.atmel.com/Images/doc8272.pdf page 128-135 */
  // Set up PWM, Phase and Frequency Correct on pin 18 (OC1A) & pin 17 (OC1B) with ICR1 as TOP using Timer1
  TCCR1B = _BV(WGM13) | _BV(CS10); // Set PWM Phase and Frequency Correct with ICR1 as TOP and no prescaling
  ICR1H = (PWMVALUE >> 8); // ICR1 is the TOP value - this is set so the frequency is equal to 20kHz
  ICR1L = (PWMVALUE & 0xFF);

  /* Enable PWM on pin 18 (OC1A) & pin 17 (OC1B) */
  // Clear OC1A/OC1B on compare match when up-counting
  // Set OC1A/OC1B on compare match when downcounting
  TCCR1A = _BV(COM1A1) | _BV(COM1B1);
  setPWM(leftPWM,0); // Turn off pwm on both pins
  setPWM(rightPWM,0);
  
  if(Usb.Init() == -1) { // Check if USB Host is working
    Serial.print(F("OSC did not start"));
    while(1); // Halt
  }

  /* Setup IMU Inputs */
  Wire.begin();
  while(i2cWrite(0x1B,0x00,false)); // Set Gyro Full Scale Range to ±250deg/s
  while(i2cWrite(0x1C,0x00,false)); // Set Accelerometer Full Scale Range to ±2g
  while(i2cWrite(0x19,0x13,false)); // Set the sample rate to 400Hz
  while(i2cWrite(0x1A,0x00,false)); // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  while(i2cWrite(0x6B,0x01,true)); // PLL with X axis gyroscope reference and disable sleep mode

  uint8_t buf;
  while(i2cRead(0x75,&buf,1));
  if(buf != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while(1); // Halt
  }

  delay(100); // Wait for the sensor to get ready
  
  /* Set kalman and gyro starting angle */
  uint8_t data[4];
  while(i2cRead(0x3D,data,4));
  accY = ((data[0] << 8) | data[1]);
  accZ = ((data[2] << 8) | data[3]);
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accAngle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
  
  kalman.setAngle(accAngle); // Set starting angle
  gyroAngle = accAngle;
  
  /* Setup pin for buzzer and beep to indicate that it is now ready */
  pinMode(buzzer,OUTPUT);
  
  cbi(TCCR0B,CS00); // Set precaler to 8
  analogWrite(buzzer,128);
  delay(800); // This is really 100ms
  analogWrite(buzzer,0);
  sbi(TCCR0B,CS00); // Set precaler back to 64

  /* Setup timing */  
  kalmanTimer = micros();
  pidTimer = kalmanTimer;
  encoderTimer = kalmanTimer;
  dataTimer = kalmanTimer;
  wiiTimer = kalmanTimer;
}

void loop() {
  /* Calculate pitch */
  uint8_t data[8];
  while(i2cRead(0x3D,data,8));
  accY = ((data[0] << 8) | data[1]);
  accZ = ((data[2] << 8) | data[3]);
  gyroX = ((data[6] << 8) | data[7]);
  
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accAngle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
  gyroRate = (double)gyroX/131.0; // Convert to deg/s
  gyroAngle += gyroRate*((double)(micros()-kalmanTimer)/1000000.0); // Gyro angle is only used for debugging

  pitch = kalman.getAngle(accAngle, gyroRate, (double)(micros()-kalmanTimer)/1000000.0); // Calculate the angle using a Kalman filter
  kalmanTimer = micros();
  //Serial.print(accAngle);Serial.print('\t');Serial.print(gyroAngle);Serial.print('\t');Serial.println(pitch);

  /* Drive motors */
  // If the robot is laying down, it has to be put in a vertical position before it starts balancing
  // If it's already balancing it has to be ±45 degrees before it stops trying to balance
  if((layingDown && (pitch < 170 || pitch > 190)) || (!layingDown && (pitch < 135 || pitch > 225))) {
    layingDown = true; // The robot is in a unsolvable position, so turn off both motors and wait until it's vertical again
    stopAndReset();
  }
  else {
    layingDown = false; // It's no longer laying down
    PID(targetAngle,targetOffset,turningOffset,(double)(micros()-pidTimer)/1000000.0);
  }
  pidTimer = micros();

  /* Update encoders */
  if (micros() - encoderTimer >= 100000) { // Update encoder values every 100ms
    encoderTimer = micros();
    wheelPosition = readLeftEncoder() + readRightEncoder();
    wheelVelocity = wheelPosition - lastWheelPosition;
    lastWheelPosition = wheelPosition;
    //Serial.print(wheelPosition);Serial.print('\t');Serial.print(targetPosition);Serial.print('\t');Serial.println(wheelVelocity);
    if(abs(wheelVelocity) <= 40 && !stopped) { // Set new targetPosition if braking
      targetPosition = wheelPosition;
      stopped = true;
    }
  }

  /* Read the Bluetooth dongle and send PID and IMU values */
  readBTD();
  sendBluetoothData();
  if(Wii.wiimoteConnected) { // We have to read much more often from the Wiimote to prevent lag
    while((micros() - wiiTimer) < 10000)
      Usb.Task();   
  }
  wiiTimer = micros(); 
}
