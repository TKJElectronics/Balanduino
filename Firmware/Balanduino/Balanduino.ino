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

 This is the algorithm for the Balanduino balancing robot.
 It can be controlled by either an Android app or a computer application via Bluetooth.
 The Android app can be found at the following link: https://github.com/TKJElectronics/BalanduinoAndroidApp
 The Processing application can be found here: https://github.com/TKJElectronics/BalanduinoProcessingApp
 A dedicated Windows application can be found here: https://github.com/TKJElectronics/BalanduinoWindowsApp
 It can also be controlled by a PS3, PS4, Wii or a Xbox controller.
 Furthermore it supports the Spektrum serial protocol used for RC receivers.
 For details, see: http://balanduino.net/
*/

/* Use this to enable and disable the different options */
#define ENABLE_TOOLS
#define ENABLE_SPP
#define ENABLE_PS3
#define ENABLE_PS4
#define ENABLE_WII
#define ENABLE_XBOX
#define ENABLE_ADK
#define ENABLE_SPEKTRUM

#include "Balanduino.h"
#include <Arduino.h> // Standard Arduino header
#include <Wire.h> // Official Arduino Wire library

#ifdef ENABLE_ADK
#include <adk.h>
#endif

// These are all open source libraries written by Kristian Lauszus, TKJ Electronics
// The USB libraries are located at the following link: https://github.com/felis/USB_Host_Shield_2.0
#include <Kalman.h> // Kalman filter library - see: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/

#ifdef ENABLE_XBOX
#include <XBOXRECV.h>
#endif
#ifdef ENABLE_SPP
#include <SPP.h>
#endif
#ifdef ENABLE_PS3
#include <PS3BT.h>
#endif
#ifdef ENABLE_PS4
#include <PS4BT.h>
#endif
#ifdef ENABLE_WII
#include <Wii.h>
#endif

// Create the Kalman library instance
Kalman kalman; // See https://github.com/TKJElectronics/KalmanFilter for source code

#if defined(ENABLE_SPP) || defined(ENABLE_PS3) || defined(ENABLE_PS4) || defined(ENABLE_WII) || defined(ENABLE_XBOX) || defined(ENABLE_ADK)
#define ENABLE_USB
USB Usb; // This will take care of all USB communication
#else
#define _usb_h_ // Workaround include trap in the USB Host library
#include <avrpins.h> // Include this from the USB Host library
#endif

#ifdef ENABLE_ADK
// Implementation for the Android Open Accessory Protocol. Simply connect your phone to get redirected to the Play Store
ADK adk(&Usb, "TKJ Electronics", // Manufacturer Name
              "Balanduino", // Model Name
              "Android App for Balanduino", // Description - user visible string
              "0.6.1", // Version of the Android app
              "https://play.google.com/store/apps/details?id=com.tkjelectronics.balanduino", // URL - web page to visit if no installed apps support the accessory
              "1234"); // Serial Number - this is not used
#endif

#ifdef ENABLE_XBOX
XBOXRECV Xbox(&Usb); // You have to connect a Xbox wireless receiver to the Arduino to control it with a wireless Xbox controller
#endif

#if defined(ENABLE_SPP) || defined(ENABLE_PS3) || defined(ENABLE_PS4) || defined(ENABLE_WII)
#define ENABLE_BTD
#include <usbhub.h> // Some dongles can have a hub inside
USBHub Hub(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // This is the main Bluetooth library, it will take care of all the USB and HCI communication with the Bluetooth dongle
#endif

#ifdef ENABLE_SPP
SPP SerialBT(&Btd, "Balanduino", "0000"); // The SPP (Serial Port Protocol) emulates a virtual Serial port, which is supported by most computers and mobile phones
#endif

#ifdef ENABLE_PS3
PS3BT PS3(&Btd); // The PS3 library supports all three official controllers: the Dualshock 3, Navigation and Move controller
#endif

#ifdef ENABLE_PS4
//PS4BT PS4(&Btd, PAIR); // You should create the instance like this if you want to pair with a PS4 controller, then hold PS and Share on the PS4 controller
// Or you can simply send "CPP;" to the robot to start the pairing sequence
// This can also be done using the Android or via the serial port
PS4BT PS4(&Btd); // The PS4BT library supports the PS4 controller via Bluetooth
#endif

#ifdef ENABLE_WII
WII Wii(&Btd); // The Wii library can communicate with Wiimotes and the Nunchuck and Motion Plus extension and finally the Wii U Pro Controller
//WII Wii(&Btd,PAIR); // You will have to pair with your Wiimote first by creating the instance like this and the press 1+2 on the Wiimote or press sync if you are using a Wii U Pro Controller
// Or you can simply send "CPW;" to the robot to start the pairing sequence
// This can also be done using the Android or via the serial port
#endif

void setup() {
  /* Setup buzzer pin */
  buzzer::SetDirWrite();

  /* Read the PID values, target angle and other saved values in the EEPROM */
  if (!checkInitializationFlags()) {
    readEEPROMValues(); // Only read the EEPROM values if they have not been restored
#ifdef ENABLE_SPEKTRUM
    if (cfg.bindSpektrum) // If flag is set, then bind with Spektrum satellite receiver
      bindSpektrum();
#endif
  } else { // Indicate that the EEPROM values have been reset by turning on the buzzer
    buzzer::Set();
    delay(1000);
    buzzer::Clear();
    delay(100); // Wait a little after the pin is cleared
  }

  /* Initialize UART */
  Serial.begin(115200);

  /* Setup encoders */
  leftEncoder1::SetDirRead();
  leftEncoder2::SetDirRead();
  rightEncoder1::SetDirRead();
  rightEncoder2::SetDirRead();
  leftEncoder1::Set(); // Enable pull-ups
  leftEncoder2::Set();
  rightEncoder1::Set();
  rightEncoder2::Set();
  attachInterrupt(digitalPinToInterrupt(leftEncoder1Pin), leftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoder1Pin), rightEncoder, CHANGE);

#if defined(PIN_CHANGE_INTERRUPT_VECTOR_LEFT) && defined(PIN_CHANGE_INTERRUPT_VECTOR_RIGHT)
  /* Enable encoder pins interrupt sources */
  *digitalPinToPCMSK(leftEncoder2Pin) |= (1 << digitalPinToPCMSKbit(leftEncoder2Pin));
  *digitalPinToPCMSK(rightEncoder2Pin) |= (1 << digitalPinToPCMSKbit(rightEncoder2Pin));

  /* Enable pin change interrupts */
  *digitalPinToPCICR(leftEncoder2Pin) |= (1 << digitalPinToPCICRbit(leftEncoder2Pin));
  *digitalPinToPCICR(rightEncoder2Pin) |= (1 << digitalPinToPCICRbit(rightEncoder2Pin));
#endif

  /* Set the motordriver diagnostic pins to inputs */
  leftDiag::SetDirRead();
  rightDiag::SetDirRead();

  /* Setup motor pins to output */
  leftPWM::SetDirWrite();
  leftA::SetDirWrite();
  leftB::SetDirWrite();
  rightPWM::SetDirWrite();
  rightA::SetDirWrite();
  rightB::SetDirWrite();

  /* Set PWM frequency to 20kHz - see the datasheet http://www.atmel.com/Images/Atmel-8272-8-bit-AVR-microcontroller-ATmega164A_PA-324A_PA-644A_PA-1284_P_datasheet.pdf page 129-139 */
  // Set up PWM, Phase and Frequency Correct on pin 18 (OC1A) & pin 17 (OC1B) with ICR1 as TOP using Timer1
  TCCR1B = (1 << WGM13) | (1 << CS10); // Set PWM Phase and Frequency Correct with ICR1 as TOP and no prescaling
  ICR1 = PWMVALUE; // ICR1 is the TOP value - this is set so the frequency is equal to 20kHz

  /* Enable PWM on pin 18 (OC1A) & pin 17 (OC1B) */
  // Clear OC1A/OC1B on compare match when up-counting
  // Set OC1A/OC1B on compare match when down-counting
  TCCR1A = (1 << COM1A1) | (1 << COM1B1);

#ifdef ENABLE_USB
  if (Usb.Init() == -1) { // Check if USB Host is working
    Serial.print(F("OSC did not start"));
    buzzer::Set();
    while (1); // Halt
  }
#endif

  /* Attach onInit function */
  // This is used to set the LEDs according to the voltage level and vibrate the controller to indicate the new connection
#ifdef ENABLE_PS3
  PS3.attachOnInit(onInitPS3);
#endif
#ifdef ENABLE_PS4
  PS4.attachOnInit(onInitPS4);
#endif
#ifdef ENABLE_WII
  Wii.attachOnInit(onInitWii);
#endif
#ifdef ENABLE_XBOX
  Xbox.attachOnInit(onInitXbox);
#endif

  /* Setup IMU */
  Wire.begin();
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

  while (i2cRead(0x75, i2cBuffer, 1));
  if (i2cBuffer[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    buzzer::Set();
    while (1); // Halt
  }

  while (i2cWrite(0x6B, 0x80, true)); // Reset device, this resets all internal registers to their default values
  do {
    while (i2cRead(0x6B, i2cBuffer, 1));
  } while (i2cBuffer[0] & 0x80); // Wait for the bit to clear
  delay(5);
  while (i2cWrite(0x6B, 0x09, true)); // PLL with X axis gyroscope reference, disable temperature sensor and disable sleep mode
#if 1
  i2cBuffer[0] = 1; // Set the sample rate to 500Hz - 1kHz/(1+1) = 500Hz
  i2cBuffer[1] = 0x03; // Disable FSYNC and set 44 Hz Acc filtering, 42 Hz Gyro filtering, 1 KHz sampling
#else
  i2cBuffer[0] = 15; // Set the sample rate to 500Hz - 8kHz/(15+1) = 500Hz
  i2cBuffer[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
#endif
  i2cBuffer[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cBuffer[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cBuffer, 4, true)); // Write to all four registers at once

  delay(100); // Wait for the sensor to get ready

  /* Set Kalman and gyro starting angle */
  while (i2cRead(0x3D, i2cBuffer, 4));
  accY = ((i2cBuffer[0] << 8) | i2cBuffer[1]);
  accZ = ((i2cBuffer[2] << 8) | i2cBuffer[3]);
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accAngle = (atan2((double)accY - cfg.accYzero, (double)accZ - cfg.accZzero) + PI) * RAD_TO_DEG;

  kalman.setAngle(accAngle); // Set starting angle
  pitch = accAngle;
  gyroAngle = accAngle;

  /* Calibrate gyro zero value */
  while (calibrateGyro()); // Run again if the robot is moved while calibrating

  LED::SetDirWrite(); // Set LED pin to output
  stopAndReset(); // Turn off motors and reset different values

#ifdef ENABLE_TOOLS
  printMenu();
#endif

  /* Beep to indicate that it is now ready */
  buzzer::Set();
  delay(100);
  buzzer::Clear();

  /* Setup timing */
  kalmanTimer = micros();
  pidTimer = kalmanTimer;
  imuTimer = millis();
  encoderTimer = imuTimer;
  reportTimer = imuTimer;
  ledTimer = imuTimer;
  blinkTimer = imuTimer;
}

void loop() {
  if (!leftDiag::IsSet() || !rightDiag::IsSet()) { // Motor driver will pull these low on error
    buzzer::Set();
    stopMotor(left);
    stopMotor(right);
    while (1);
  }

#if defined(ENABLE_WII) || defined(ENABLE_PS4) // We have to read much more often from the Wiimote and PS4 controller to decrease latency
  bool readUSB = false;
#ifdef ENABLE_WII
  if (Wii.wiimoteConnected)
    readUSB = true;
#endif
#ifdef ENABLE_PS4
  if (PS4.connected())
    readUSB = true;
#endif
  if (readUSB)
    Usb.Task();
#endif

  /* Calculate pitch */
  while (i2cRead(0x3D, i2cBuffer, 8));
  accY = ((i2cBuffer[0] << 8) | i2cBuffer[1]);
  accZ = ((i2cBuffer[2] << 8) | i2cBuffer[3]);
  gyroX = ((i2cBuffer[6] << 8) | i2cBuffer[7]);

  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accAngle = (atan2((double)accY - cfg.accYzero, (double)accZ - cfg.accZzero) + PI) * RAD_TO_DEG;

  uint32_t timer = micros();
  // This fixes the 0-360 transition problem when the accelerometer angle jumps between 0 and 360 degrees
  if ((accAngle < 90 && pitch > 270) || (accAngle > 270 && pitch < 90)) {
    kalman.setAngle(accAngle);
    pitch = accAngle;
    gyroAngle = accAngle;
  } else {
    gyroRate = ((double)gyroX - gyroXzero) / 131.0; // Convert to deg/s
    double dt = (double)(timer - kalmanTimer) / 1000000.0;
    gyroAngle += gyroRate * dt; // Gyro angle is only used for debugging
    if (gyroAngle < 0 || gyroAngle > 360)
      gyroAngle = pitch; // Reset the gyro angle when it has drifted too much
    pitch = kalman.getAngle(accAngle, gyroRate, dt); // Calculate the angle using a Kalman filter
  }
  kalmanTimer = timer;
  //Serial.print(accAngle);Serial.print('\t');Serial.print(gyroAngle);Serial.print('\t');Serial.println(pitch);

#if defined(ENABLE_WII) || defined(ENABLE_PS4) // We have to read much more often from the Wiimote and PS4 controller to decrease latency
  if (readUSB)
    Usb.Task();
#endif

  /* Drive motors */
  timer = micros();
  // If the robot is laying down, it has to be put in a vertical position before it starts balancing
  // If it's already balancing it has to be ±45 degrees before it stops trying to balance
  if ((layingDown && (pitch < cfg.targetAngle - 10 || pitch > cfg.targetAngle + 10)) || (!layingDown && (pitch < cfg.targetAngle - 45 || pitch > cfg.targetAngle + 45))) {
    layingDown = true; // The robot is in a unsolvable position, so turn off both motors and wait until it's vertical again
    stopAndReset();
  } else {
    layingDown = false; // It's no longer laying down
    updatePID(cfg.targetAngle, targetOffset, turningOffset, (double)(timer - pidTimer) / 1000000.0);
  }
  pidTimer = timer;

  /* Update encoders */
  timer = millis();
  if (timer - encoderTimer >= 100) { // Update encoder values every 100ms
    encoderTimer = timer;
    int32_t wheelPosition = getWheelsPosition();
    wheelVelocity = wheelPosition - lastWheelPosition;
    lastWheelPosition = wheelPosition;
    //Serial.print(wheelPosition);Serial.print('\t');Serial.print(targetPosition);Serial.print('\t');Serial.println(wheelVelocity);
    if (abs(wheelVelocity) <= 40 && !stopped) { // Set new targetPosition if braking
      targetPosition = wheelPosition;
      stopped = true;
    }

    batteryCounter++;
    if (batteryCounter >= 10) { // Measure battery every 1s
      batteryCounter = 0;
      batteryVoltage = (double)analogRead(VBAT) / 63.050847458; // VBAT is connected to analog input 5 which is not broken out. This is then connected to a 47k-12k voltage divider - 1023.0/(3.3/(12.0/(12.0+47.0))) = 63.050847458
      if (batteryVoltage < 10.2 && batteryVoltage > 5) // Equal to 3.4V per cell - don't turn on if it's below 5V, this means that no battery is connected
        buzzer::Set();
      else
        buzzer::Clear();
    }
  }

  /* Read the Bluetooth dongle and send PID and IMU values */
#if defined(ENABLE_TOOLS) || defined(ENABLE_SPEKTRUM)
  checkSerialData();
#endif
#if defined(ENABLE_USB) || defined(ENABLE_SPEKTRUM)
  readUsb();
#endif
#if defined(ENABLE_TOOLS) || defined(ENABLE_SPP)
  printValues();
#endif

#ifdef ENABLE_BTD
  if (Btd.isReady()) {
    timer = millis();
    if ((Btd.watingForConnection && timer - blinkTimer > 1000) || (!Btd.watingForConnection && timer - blinkTimer > 100)) {
      blinkTimer = timer;
      LED::Toggle(); // Used to blink the built in LED, starts blinking faster upon an incoming Bluetooth request
    }
  } else
    LED::Clear(); // This will turn it off
#endif
}
