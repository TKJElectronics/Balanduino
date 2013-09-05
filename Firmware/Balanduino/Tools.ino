#ifdef ENABLE_TOOLS
void checkSerialData() {
  if (Serial.available()) {
    char input = Serial.read();
    if (input == 'm') {
      printMenu();
      while (!Serial.available()); // Wait for data
      input = Serial.read();
      if (input == 'c') { // Calibration
        Serial.println(F("Send 'a' to calibrate the accelerometer\r\nSend 'm' to calibrate the motor"));
        while (!Serial.available()); // Wait for data
        input = Serial.read();
        if (input == 'a') // Accelerometer calibration
          calibrateAcc();
        //else if (input == 'm') // Motor calibration
        return; // Skip the rest of this function
      }
    }

    dataInput[0] = input;
    uint8_t i = 1;
    delay(1); // Wait for rest of data
    while (1) {
      dataInput[i] = Serial.read();
      if (dataInput[i] == -1) // Error while reading the string
        return;
      if (dataInput[i] == ';') // Keep reading until it reads a semicolon
        break;
      if (++i >= sizeof(dataInput)/sizeof(dataInput[0])) // String is too long
        return;
    }
    bluetoothData = false;
    setValues(dataInput);
  }
}

void printMenu() {
  Serial.println(F("\r\n========================================== Menu ==========================================\r\n"));
  Serial.println(F("c\t\t\t\tEnter calibration menu\r\n"));

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
  Serial.println(F("SU,value;\t\t\tUsed to set the maximum turning vale"));
  Serial.println(F("SB,value;\t\t\tUsed to set the back to spot value (true = 1, false = 0)\r\n"));

  Serial.println(F("IB;\t\t\t\tStart sending IMU values"));
  Serial.println(F("IS;\t\t\t\tStop sending IMU values\r\n"));

  Serial.println(F("CS;\t\t\t\tSend stop command"));
  Serial.println(F("CJ,x,y;\t\t\t\tSteer robot using x,y-coordinates"));
  Serial.println(F("CM,pitch,roll;\t\t\tSteer robot using pitch and roll"));
  Serial.println(F("CW;\t\t\t\tStart paring sequence with Wiimote"));
  Serial.println(F("CR;\t\t\t\tRestore default EEPROM values"));
  Serial.println(F("\r\n==========================================================================================\r\n"));
}

void calibrateAcc() {
  Serial.println(F("Please put the robot perfectly horizontal and then send 'a' again to start the calibration routine"));
  while (Serial.read() != 'a');

  int16_t accYbuffer[25], accZbuffer[25];
  for (uint8_t i = 0; i < 25; i++) {
    while (i2cRead(0x3D, i2cBuffer, 4));
    accYbuffer[i] = ((i2cBuffer[0] << 8) | i2cBuffer[1]);
    accZbuffer[i] = ((i2cBuffer[2] << 8) | i2cBuffer[3]);
    delay(10);
  }
  if (!checkMinMax(accYbuffer, 25, 1000) || !checkMinMax(accZbuffer, 25, 1000)) {
    Serial.print(F("Accelerometer calibration error"));
    digitalWrite(buzzer, HIGH);
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

  updateConfig(); // Store the new values in the EEPROM
  Serial.println(F("Calibration of the accelerometer is done"));
}

#endif // ENABLE_TOOLS

#if defined(ENABLE_TOOLS) || defined(ENABLE_SPP)
void printValues() {
  Print *pipe; // This allows the robot to use either the hardware UART or the Bluetooth SPP connection dynamically

#ifdef ENABLE_SPP
  if (SerialBT.connected && bluetoothData)
    pipe = &SerialBT;
  else
    pipe = &Serial;
#else
  pipe = &Serial;
#endif

  if (sendPairConfirmation) {
    sendPairConfirmation = false;

    pipe->println(F("WC"));
  } else if (sendPIDValues) {
    sendPIDValues = false;

    pipe->print(F("P,"));
    pipe->print(cfg.P);
    pipe->print(F(","));
    pipe->print(cfg.I);
    pipe->print(F(","));
    pipe->print(cfg.D);
    pipe->print(F(","));
    pipe->println(cfg.targetAngle);
  } else if (sendSettings) {
    sendSettings = false;

    pipe->print(F("S,"));
    pipe->print(cfg.backToSpot);
    pipe->print(F(","));
    pipe->print(cfg.controlAngleLimit);
    pipe->print(F(","));
    pipe->println(cfg.turningLimit);
  } else if (sendInfo) {
    sendInfo = false;

    pipe->print(F("I,"));
    pipe->print(version);

    #if defined(__AVR_ATmega644__)
      pipe->print(F(",ATmega644,"));
    #elif defined(__AVR_ATmega1284P__)
      pipe->print(F(",ATmega1284P,"));
    #else
      pipe->print(F(",Unknown,"));
    #endif

    pipe->print(batteryVoltage);
    pipe->print(F(","));
    pipe->println((double)millis()/60000.0);
  } else if (sendKalmanValues) {
    sendKalmanValues = false;

    pipe->print(F("K,"));
    pipe->print(kalman.getQangle(), 4);
    pipe->print(F(","));
    pipe->print(kalman.getQbias(), 4);
    pipe->print(F(","));
    pipe->println(kalman.getRmeasure(), 4);
  } else if (sendData && (millis() - dataTimer > 50)) { // Only send data every 50ms
    dataTimer = millis();

    pipe->print(F("V,"));
    pipe->print(accAngle);
    pipe->print(F(","));
    pipe->print(gyroAngle);
    pipe->print(F(","));
    pipe->println(pitch);
  }
}

void setValues(char *input) {
  if (input[0] == 'A') { // Abort
    stopAndReset();
#ifdef ENABLE_SPP
    while (Serial.read() != 'C' && SerialBT.read() != 'C') // Wait until continue is sent
      Usb.Task();
#else
    while (Serial.read() != 'C');
#endif
  }

  /* For sending PID and IMU values */
  else if (input[0] == 'G') { // The Processing/Android application sends when it needs the PID, settings or info
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
    else if (input[1] == 'B') // Set Back To Spot
      cfg.backToSpot = input[2] - '0'; // Convert from ASCII to number

    updateConfig();
  }

  else if (input[0] == 'I') { // IMU transmitting states
    if (input[1] == 'B') // Begin sending IMU values
      sendData = true; // Send output to Processing/Android application
    else if (input[1] == 'S') // Stop sending IMU values
      sendData = false; // Stop sending output to Processing/Android application
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
#ifdef ENABLE_WII
    else if (input[1] == 'W') { // Pair with a new Wiimote or Wii U Pro Controller
      Wii.pair();
      sendPairConfirmation = true;
    }
#endif // ENABLE_WII
    else if (input[1] == 'R') {
      restoreEEPROMValues(); // Restore the default EEPROM values
      sendPIDValues = true;
      sendKalmanValues = true;
      sendSettings = true;
    }
  }
}
#endif // ENABLE_TOOLS or ENABLE_SPP

void calibrateGyro() {
  int16_t gyroXbuffer[25];
  for (uint8_t i = 0; i < 25; i++) {
    while (i2cRead(0x43, i2cBuffer, 2));
    gyroXbuffer[i] = ((i2cBuffer[0] << 8) | i2cBuffer[1]);
    delay(10);
  }
  if (!checkMinMax(gyroXbuffer, 25, 200)) {
    Serial.print(F("Gyro calibration error"));
    digitalWrite(buzzer, HIGH);
    while (1); // Halt
  }
  for (uint8_t i = 0; i < 25; i++)
    gyroXzero += gyroXbuffer[i];
  gyroXzero /= 25;
}

bool checkMinMax(int16_t *array, uint8_t length, uint16_t maxDifference) { // Used to check that the robot is laying still while calibrating
  int16_t min = array[0], max = array[0];
  for (uint8_t i = 1; i < length; i++) {
    if (array[i] < min)
      min = array[i];
    else if (array[i] > max)
      max = array[i];
  }
  return max - min < maxDifference;
}
