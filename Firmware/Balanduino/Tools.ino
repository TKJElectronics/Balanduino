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

#if defined(ENABLE_TOOLS) || defined(ENABLE_SPEKTRUM)

void checkSerialData() {
  if (Serial.available()) {
    int input = Serial.read();
#ifdef ENABLE_TOOLS
    if (input == 'm')
      printMenu();
    else {
#endif
#ifdef ENABLE_SPEKTRUM
      readSpektrum(static_cast<uint8_t> (input)); // Intentional cast
#endif
#ifdef ENABLE_TOOLS
      dataInput[0] = static_cast<char> (input); // Intentional cast
      delay(2); // Wait for rest of data
      uint8_t i = 1;
#endif
      while (1) {
        input = Serial.read();
        if (input == -1) // Error while reading the string
          return;
#ifdef ENABLE_SPEKTRUM
        readSpektrum(static_cast<uint8_t> (input)); // Intentional cast
#endif
#ifdef ENABLE_TOOLS
        dataInput[i] = static_cast<char> (input); // Intentional cast
        if (dataInput[i] == ';') // Keep reading until it reads a semicolon
          break;
        if (++i >= sizeof(dataInput) / sizeof(dataInput[0])) // String is too long
          return;
#endif
      }
#ifdef ENABLE_TOOLS
      bluetoothData = false;
      setValues(dataInput);
    }
#endif
  }
}

#endif // defined(ENABLE_TOOLS) || defined(ENABLE_SPEKTRUM)

#ifdef ENABLE_TOOLS

void printMenu() {
  Serial.println(F("\r\n========================================== Menu ==========================================\r\n"));

  Serial.println(F("m\t\t\t\tSend to show this menu\r\n"));

  Serial.println(F("A;\t\t\t\tSend to abort. Send 'C' again to continue\r\n"));

  Serial.println(F("AC;\t\t\t\tSend to calibrate the accelerometer"));
  Serial.println(F("MC;\t\t\t\tSend to calibrate the motors\r\n"));

  Serial.println(F("GP;\t\t\t\tGet PID values"));
  Serial.println(F("GK;\t\t\t\tGet Kalman filter values"));
  Serial.println(F("GS;\t\t\t\tGet settings values"));
  Serial.println(F("GI;\t\t\t\tGet info values\r\n"));

  Serial.println(F("SP,Kp;\t\t\t\tUsed to set the Kp value"));
  Serial.println(F("SI,Ki;\t\t\t\tUsed to set the Ki value"));
  Serial.println(F("SD,Kd;\t\t\t\tUsed to set the Kd value"));
  Serial.println(F("ST,targetAngle;\t\t\tUsed to set the target angle"));
  Serial.println(F("SK,Qangle,Qbias,Rmeasure;\tUsed to set the Kalman filter values"));
  Serial.println(F("SA,angle;\t\t\tUsed to set the maximum controlling angle"));
  Serial.println(F("SU,value;\t\t\tUsed to set the maximum turning value"));
  Serial.println(F("SB,value;\t\t\tUsed to set the back to spot value (true = 1, false = 0)\r\n"));

  Serial.println(F("IB;\t\t\t\tStart sending IMU values"));
  Serial.println(F("IS;\t\t\t\tStop sending IMU values"));
  Serial.println(F("RB;\t\t\t\tStart sending report values"));
  Serial.println(F("RS;\t\t\t\tStop sending report values\r\n"));

  Serial.println(F("CS;\t\t\t\tSend stop command"));
  Serial.println(F("CJ,x,y;\t\t\t\tSteer robot using x,y-coordinates"));
  Serial.println(F("CM,pitch,roll;\t\t\tSteer robot using pitch and roll"));
#ifdef ENABLE_WII
  Serial.println(F("CPW;\t\t\t\tStart paring sequence with Wiimote"));
#endif
#ifdef ENABLE_PS4
  Serial.println(F("CPP;\t\t\t\tStart paring sequence with PS4 controller"));
#endif
  Serial.println(F("CR;\t\t\t\tRestore default EEPROM values\r\n"));

#ifdef ENABLE_SPEKTRUM
  Serial.println(F("BS;\t\t\t\tBind with Spektrum satellite receiver"));
#endif
  Serial.println(F("\r\n==========================================================================================\r\n"));
}

void calibrateAcc() {
  Serial.println(F("Please put the robot perfectly horizontal on its side and then send any character to start the calibration routine"));
  while (Serial.read() == -1);

  int16_t accYbuffer[25], accZbuffer[25];
  for (uint8_t i = 0; i < 25; i++) {
    while (i2cRead(0x3D, i2cBuffer, 4));
    accYbuffer[i] = ((i2cBuffer[0] << 8) | i2cBuffer[1]);
    accZbuffer[i] = ((i2cBuffer[2] << 8) | i2cBuffer[3]);
    delay(10);
  }
  if (!checkMinMax(accYbuffer, 25, 1000) || !checkMinMax(accZbuffer, 25, 1000)) {
    Serial.print(F("Accelerometer calibration error"));
    buzzer::Set();
    while (1); // Halt
  }
  for (uint8_t i = 0; i < 25; i++) {
    cfg.accYzero += accYbuffer[i];
    cfg.accZzero += accZbuffer[i];
  }
  cfg.accYzero /= 25;
  cfg.accZzero /= 25;

  if (cfg.accYzero < 0) // Check which side is laying down
    cfg.accYzero += 16384.0; // 16384.0 is equal to 1g while the full scale range is ±2g
  else
    cfg.accYzero -= 16384.0;

  Serial.print(F("New zero values (g's): ")); // Print the new values in g's
  Serial.print(cfg.accYzero / 16384.0);
  Serial.print(F(","));
  Serial.println(cfg.accZzero / 16384.0);

  updateConfig(); // Store the new values in the EEPROM
  Serial.println(F("Calibration of the accelerometer is done"));
}

void calibrateMotor() {
  Serial.println(F("Put the robot so the wheels can move freely and then send any character to start the motor calibration routine"));
  while (Serial.read() == -1);

  Serial.println(F("Estimating minimum starting value. When the first two values do not change anymore, then send any character to continue\r\n"));
  delay(2000);
  double leftSpeed = 10, rightSpeed = 10;
  testMotorSpeed(&leftSpeed, &rightSpeed, 1, 1);

  Serial.print(F("\r\nThe speed values are (L/R): "));
  Serial.print(leftSpeed);
  Serial.print(F(","));
  Serial.println(rightSpeed);

  if (leftSpeed > rightSpeed) { // This means that the left motor needed a higher PWM signal before it rotated at the same speed
    cfg.leftMotorScaler = 1;
    cfg.rightMotorScaler = rightSpeed / leftSpeed; // Therefore we will scale the right motor a bit down, so they match
  } else { // And the same goes for the right motor
    cfg.leftMotorScaler = leftSpeed / rightSpeed;
    cfg.rightMotorScaler = 1;
  }

  Serial.print(F("The motor scalars are now (L/R): "));
  Serial.print(cfg.leftMotorScaler);
  Serial.print(F(","));
  Serial.println(cfg.rightMotorScaler);

  Serial.println(F("Now the motors will spin up again. Now the speed values should be almost equal. Send any character to exit\r\n"));
  delay(2000);
  leftSpeed = rightSpeed = 10; // Reset speed values
  testMotorSpeed(&leftSpeed, &rightSpeed, cfg.leftMotorScaler, cfg.rightMotorScaler);

  double maxSpeed = max(leftSpeed, rightSpeed);
  double minSpeed = min(leftSpeed, rightSpeed);

  Serial.print(F("The difference is now: "));
  Serial.print((maxSpeed - minSpeed) / maxSpeed * 100);
  Serial.println("%");

  updateConfig(); // Store the new values in the EEPROM
  Serial.println(F("Calibration of the motors is done"));
}

void testMotorSpeed(double *leftSpeed, double *rightSpeed, double leftScaler, double rightScaler) {
  int32_t lastLeftPosition = readLeftEncoder(), lastRightPosition = readRightEncoder();

  Serial.println(F("Velocity (L), Velocity (R), Speed value (L), Speed value (R)"));
  while (Serial.read() == -1) {
    moveMotor(left, forward, (*leftSpeed)*leftScaler);
    moveMotor(right, forward, (*rightSpeed)*rightScaler);

    int32_t leftPosition = readLeftEncoder();
    int32_t leftVelocity = leftPosition - lastLeftPosition;
    lastLeftPosition = leftPosition;

    int32_t rightPosition = readRightEncoder();
    int32_t rightVelocity = rightPosition - lastRightPosition;
    lastRightPosition = rightPosition;

    Serial.print(leftVelocity);
    Serial.print(F(","));
    Serial.print(rightVelocity);
    Serial.print(F(","));
    Serial.print(*leftSpeed);
    Serial.print(F(","));
    Serial.println(*rightSpeed);

    if (abs(leftVelocity) < 200)
      (*leftSpeed) += 0.1;
    else if (abs(leftVelocity) > 203)
      (*leftSpeed) -= 0.1;

    if (abs(rightVelocity) < 200)
      (*rightSpeed) += 0.1;
    else if (abs(rightVelocity) > 203)
      (*rightSpeed) -= 0.1;

    delay(100);
  }
  for (double i = *leftSpeed; i > 0; i--) { // Stop motors gently
    moveMotor(left, forward, i);
    moveMotor(right, forward, i);
    delay(50);
  }
  stopMotor(left);
  stopMotor(right);
}

#endif // ENABLE_TOOLS

#if defined(ENABLE_TOOLS) || defined(ENABLE_SPP)
void printValues() {
#ifdef ENABLE_SPP
  Print *out; // This allows the robot to use either the hardware UART or the Bluetooth SPP connection dynamically
  if (SerialBT.connected && bluetoothData)
    out = dynamic_cast<Print *> (&SerialBT); // Print using the Bluetooth SPP interface
  else
    out = dynamic_cast<Print *> (&Serial); // Print using the standard UART port
#else
  HardwareSerial *out = &Serial; // Print using the standard UART port
#endif

  if (sendPairConfirmation) {
    sendPairConfirmation = false;

    out->println(F("PC"));
  } else if (sendPIDValues) {
    sendPIDValues = false;

    out->print(F("P,"));
    out->print(cfg.P);
    out->print(F(","));
    out->print(cfg.I);
    out->print(F(","));
    out->print(cfg.D);
    out->print(F(","));
    out->println(cfg.targetAngle);
  } else if (sendSettings) {
    sendSettings = false;

    out->print(F("S,"));
    out->print(cfg.backToSpot);
    out->print(F(","));
    out->print(cfg.controlAngleLimit);
    out->print(F(","));
    out->println(cfg.turningLimit);
  } else if (sendInfo) {
    sendInfo = false;

    out->print(F("I,"));
    out->print(version);
    out->print(F(","));
    out->print(eepromVersion);

#if defined(__AVR_ATmega644__)
    out->println(F(",ATmega644"));
#elif defined(__AVR_ATmega1284P__)
    out->println(F(",ATmega1284P"));
#else
    out->println(F(",Unknown"));
#endif
  } else if (sendKalmanValues) {
    sendKalmanValues = false;

    out->print(F("K,"));
    out->print(kalman.getQangle(), 4);
    out->print(F(","));
    out->print(kalman.getQbias(), 4);
    out->print(F(","));
    out->println(kalman.getRmeasure(), 4);
  } else if (sendIMUValues && millis() - imuTimer > 50) { // Only send data every 50ms
    imuTimer = millis();

    out->print(F("V,"));
    out->print(accAngle);
    out->print(F(","));
    out->print(gyroAngle);
    out->print(F(","));
    out->println(pitch);
  } else if (sendStatusReport && millis() - reportTimer > 500) { // Send data every 500ms
    reportTimer = millis();

    out->print(F("R,"));
    out->print(batteryVoltage);
    out->print(F(","));
    out->println((double)reportTimer / 60000.0);
  }
}

void setValues(char *input) {
  if (input[0] == 'A' && input[1] == ';') { // Abort
    stopAndReset();
#ifdef ENABLE_SPP
    while (Serial.read() != 'C' && SerialBT.read() != 'C') // Wait until continue is sent
      Usb.Task();
#else
    while (Serial.read() != 'C');
#endif
  }

#ifdef ENABLE_SPEKTRUM
  else if (input[0] == 'B' && input[1] == 'S') { // Bind with Spektrum satellite receiver on next power up
    Serial.println(F("Now turn the Balanduino and the satellite receiver off by removing the power. The next time it turns on it will start the binding process with the satellite receiver"));
    cfg.bindSpektrum = true; // After this you should turn off the robot and then turn it on again
    updateConfig();
  }
#endif

  else if (input[0] == 'A' && input[1] == 'C') // Accelerometer calibration
    calibrateAcc();
  else if (input[0] == 'M' && input[1] == 'C') // Motor calibration
    calibrateMotor();

  /* For sending PID and IMU values */
  else if (input[0] == 'G') { // The different application sends when it needs the PID, settings or info
    if (input[1] == 'P') // Get PID Values
      sendPIDValues = true;
    else if (input[1] == 'S') // Get settings
      sendSettings = true;
    else if (input[1] == 'I') // Get info
      sendInfo = true;
    else if (input[1] == 'K') // Get Kalman filter values
      sendKalmanValues = true;
  }

  else if (input[0] == 'S') { // Set different values
    /* Set PID and target angle */
    if (input[1] == 'P') {
      strtok(input, ","); // Ignore 'P'
      cfg.P = atof(strtok(NULL, ";"));
    } else if (input[1] == 'I') {
      strtok(input, ","); // Ignore 'I'
      cfg.I = atof(strtok(NULL, ";"));
    } else if (input[1] == 'D') {
      strtok(input, ","); // Ignore 'D'
      cfg.D = atof(strtok(NULL, ";"));
    } else if (input[1] == 'T') { // Target Angle
      strtok(input, ","); // Ignore 'T'
      cfg.targetAngle = atof(strtok(NULL, ";"));
    }
    else if (input[1] == 'K') { // Kalman values
      strtok(input, ","); // Ignore 'K'
      cfg.Qangle = atof(strtok(NULL, ","));
      cfg.Qbias = atof(strtok(NULL, ","));
      cfg.Rmeasure = atof(strtok(NULL, ";"));
    }
    else if (input[1] == 'A') { // Controlling max angle
      strtok(input, ","); // Ignore 'A'
      cfg.controlAngleLimit = atoi(strtok(NULL, ";"));
    } else if (input[1] == 'U') { // Turning max value
      strtok(input, ","); // Ignore 'U'
      cfg.turningLimit = atoi(strtok(NULL, ";"));
    }
    else if (input[1] == 'B') { // Set Back To Spot
      if (input[3] == '1')
        cfg.backToSpot = 1;
      else
        cfg.backToSpot = 0;
    }

    updateConfig();
  }

  else if (input[0] == 'I') { // IMU transmitting states
    if (input[1] == 'B') // Begin sending IMU values
      sendIMUValues = true; // Start sending output to application
    else if (input[1] == 'S') // Stop sending IMU values
      sendIMUValues = false; // Stop sending output to application
  }

  else if (input[0] == 'R') { // Report states
    if (input[1] == 'B') // Begin sending report values
      sendStatusReport = true; // Start sending output to application
    else if (input[1] == 'S') // Stop sending report values
      sendStatusReport = false; // Stop sending output to application
  }

  else if (input[0] == 'C') { // Commands
    if (input[1] == 'S') // Stop
      steer(stop);
    else if (input[1] == 'J') { // Joystick
      receiveControlTimer = millis();
      strtok(input, ","); // Ignore 'J'
      sppData1 = atof(strtok(NULL, ",")); // x-axis
      sppData2 = atof(strtok(NULL, ";")); // y-axis
      steer(joystick);
    }
    else if (input[1] == 'M') { // IMU
      receiveControlTimer = millis();
      strtok(input, ","); // Ignore 'M'
      sppData1 = atof(strtok(NULL, ",")); // Pitch
      sppData2 = atof(strtok(NULL, ";")); // Roll
      steer(imu);
    }
#if defined(ENABLE_WII) || defined(ENABLE_PS4)
    else if (input[1] == 'P') { // Pair
#ifdef ENABLE_WII
      if (input[2] == 'W') { // Pair with a new Wiimote or Wii U Pro Controller
        Wii.pair();
        sendPairConfirmation = true;
      }
#endif
#ifdef ENABLE_PS4
      if (input[2] == 'P') { // Pair with PS4 controller
        PS4.pair();
        sendPairConfirmation = true;
      }
#endif
    }
#endif // defined(ENABLE_WII) || defined(ENABLE_PS4)
    else if (input[1] == 'R') {
      restoreEEPROMValues(); // Restore the default EEPROM values
      sendPIDValues = true;
      sendKalmanValues = true;
      sendSettings = true;
    }
  }
}
#endif // defined(ENABLE_TOOLS) || defined(ENABLE_SPP)

bool calibrateGyro() {
  int16_t gyroXbuffer[25];
  for (uint8_t i = 0; i < 25; i++) {
    while (i2cRead(0x43, i2cBuffer, 2));
    gyroXbuffer[i] = ((i2cBuffer[0] << 8) | i2cBuffer[1]);
    delay(10);
  }
  if (!checkMinMax(gyroXbuffer, 25, 2000)) {
    Serial.println(F("Gyro calibration error"));
    buzzer::Set();
    return 1;
  }
  for (uint8_t i = 0; i < 25; i++)
    gyroXzero += gyroXbuffer[i];
  gyroXzero /= 25;
  return 0;
}

bool checkMinMax(int16_t *array, uint8_t length, int16_t maxDifference) { // Used to check that the robot is laying still while calibrating
  int16_t min = array[0], max = array[0];
  for (uint8_t i = 1; i < length; i++) {
    if (array[i] < min)
      min = array[i];
    else if (array[i] > max)
      max = array[i];
  }
  return max - min < maxDifference;
}
