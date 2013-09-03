#ifdef ENABLE_USB

uint8_t ps3OldLed, wiiOldLed;
#ifdef ENABLE_XBOX
LED xboxOldLed;
#endif

#ifdef ENABLE_SPP
void sendBluetoothData() {
  if (SerialBT.connected) {
    if (sendPairConfirmation) {
      sendPairConfirmation = false;

      SerialBT.println("WC");
    } else if (sendPIDValues) {
      sendPIDValues = false;

      SerialBT.print("P,");
      SerialBT.print(cfg.P);
      SerialBT.print(',');
      SerialBT.print(cfg.I);
      SerialBT.print(',');
      SerialBT.print(cfg.D);
      SerialBT.print(',');
      SerialBT.println(cfg.targetAngle);
    } else if (sendSettings) {
      sendSettings = false;

      SerialBT.print("S,");
      SerialBT.print(cfg.backToSpot);
      SerialBT.print(',');
      SerialBT.print(cfg.controlAngleLimit);
      SerialBT.print(',');
      SerialBT.println(cfg.turningLimit);
    } else if (sendInfo) {
      sendInfo = false;

      SerialBT.print("I,");
      SerialBT.print(version);

      #if defined(__AVR_ATmega644__)
        SerialBT.print(",ATmega644,");
      #elif defined(__AVR_ATmega1284P__)
        SerialBT.print(",ATmega1284P,");
      #else
        SerialBT.print(",Unknown,");
      #endif

      SerialBT.print(batteryVoltage);
      SerialBT.print(",");
      SerialBT.println((double)millis()/60000.0);
    } else if (sendKalmanValues) {
      sendKalmanValues = false;

      SerialBT.print("K,");
      SerialBT.print(kalman.getQangle(), 4);
      SerialBT.print(',');
      SerialBT.print(kalman.getQbias(), 4);
      SerialBT.print(',');
      SerialBT.println(kalman.getRmeasure(), 4);
    } else if (sendData && (millis() - dataTimer > 50)) { // Only send data every 50ms
      dataTimer = millis();

      SerialBT.print("V,");
      SerialBT.print(accAngle);
      SerialBT.print(',');
      SerialBT.print(gyroAngle);
      SerialBT.print(',');
      SerialBT.println(pitch);
    }
  }
}

void readSPPData() {
  if (SerialBT.connected) { // The SPP connection won't return data as fast as the controllers, so we will handle it separately
    if (SerialBT.available()) {
      char input[30];
      uint8_t i = 0;
      while (1) {
        input[i] = SerialBT.read();
        if (input[i] == -1) // Error while reading the string
          return;
        if (input[i] == ';') // Keep reading until it reads a semicolon
          break;
        i++;
        if (i >= sizeof(input)/sizeof(input[0])) // String is too long
          return;
      }

      if (input[0] == 'A') { // Abort
        stopAndReset();
        while (SerialBT.read() != 'C') // Wait until continue is sent
          Usb.Task();
      }

      /* For sending PID and IMU values */
      else if (input[0] == 'G') { // The Processing/Android application sends when it needs the PID, settings or info
        if (input[1] == 'P') // Get PID Values
          sendPIDValues = true;
        else if (input[1] == 'S') // Get settings
          sendSettings = true;
        else if (input[1] == 'I') // Get info
          sendInfo = true;
        else if (input[1] == 'K') // Get Kalman values
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
        } else if (input[1] == 'U') { // Turning max angle
          strtok(input, ","); // Ignore 'U'
          cfg.turningLimit = atoi(strtok(NULL, ";"));
        }
        else if (input[1] == 'B') { // Set Back To Spot
          cfg.backToSpot = input[2] - '0'; // Convert from ASCII to number
        }
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
          restoreEEPROMValues(); // Restore the default PID values and target angle
          sendPIDValues = true;
          sendKalmanValues = true;
          sendSettings = true;
        }
      }
    }
  }
}
#endif // ENABLE_SPP

void readUsb() {
  Usb.Task(); // The SPP data is actually not send until this is called, one could call SerialBT.send() directly as well

#ifdef ENABLE_SPP
  readSPPData();
#endif // ENABLE_SPP

  if (millis() > (receiveControlTimer+receiveControlTimeout)) {
    commandSent = false; // We use this to detect when there has already been sent a command by one of the controllers
#ifdef ENABLE_PS3
    if (PS3.PS3Connected) {
      if (PS3.getButtonPress(SELECT)) {
        stopAndReset();
        while (!PS3.getButtonPress(START))
          Usb.Task();
      }
      else if ((PS3.getAnalogHat(LeftHatY) < 117) || (PS3.getAnalogHat(RightHatY) < 117) || (PS3.getAnalogHat(LeftHatY) > 137) || (PS3.getAnalogHat(RightHatY) > 137))
        steer(updatePS3);
      else if (PS3.getButtonPress(CROSS))
        steer(updatePS3);
    } else if (PS3.PS3NavigationConnected) {
      if (PS3.getAnalogHat(LeftHatX) > 200 || PS3.getAnalogHat(LeftHatX) < 55 || PS3.getAnalogHat(LeftHatY) > 137 || PS3.getAnalogHat(LeftHatY) < 117)
        steer(updatePS3);
    }
#endif // ENABLE_PS3
#ifdef ENABLE_WII
    if (Wii.wiimoteConnected && !Wii.wiiUProControllerConnected && !commandSent) {
      if (Wii.getButtonPress(B))
        steer(updateWii);
      else if (Wii.nunchuckConnected && (Wii.getAnalogHat(HatX) > 137 || Wii.getAnalogHat(HatX) < 117 || Wii.getAnalogHat(HatY) > 137 || Wii.getAnalogHat(HatY) < 117))
        steer(updateWii);
    } else if (Wii.wiiUProControllerConnected && !commandSent) { // The Wii U Pro Controller Joysticks has an range from approximately 800-3200
      if (Wii.getButtonPress(MINUS)) {
        stopAndReset();
        while (!Wii.getButtonPress(PLUS))
          Usb.Task();
      }
      else if (Wii.getAnalogHat(LeftHatY) > 2200 || Wii.getAnalogHat(LeftHatY) < 1800 || Wii.getAnalogHat(RightHatY) > 2200 || Wii.getAnalogHat(RightHatY) < 1800)
        steer(updateWii);
    }
#endif // ENABLE_WII
#ifdef ENABLE_XBOX
    if (Xbox.Xbox360Connected[0] && !commandSent) { // We will only read from the first controller, up to four is supported by one receiver
      if (Xbox.getButtonPress(BACK)) {
        stopAndReset();
        while (!Xbox.getButtonPress(START))
          Usb.Task();
      }
      else if ((Xbox.getAnalogHat(LeftHatY) < -7500) || (Xbox.getAnalogHat(RightHatY) < -7500) || (Xbox.getAnalogHat(LeftHatY) > 7500) || (Xbox.getAnalogHat(RightHatY) > 7500))
        steer(updateXbox);
    }
#endif // ENABLE_XBOX
    if (!commandSent) // If there hasn't been send a command by now, then send stop
      steer(stop);
  }

#ifdef ENABLE_PS3
  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {
    if (PS3.getButtonClick(PS)) {
      PS3.disconnect();
      ps3OldLed = 0; // Reset value
    }
  }
  if (!PS3.PS3Connected && !PS3.PS3NavigationConnected)
    ps3Initialized = false;
#endif // ENABLE_PS3
#ifdef ENABLE_WII
  if (Wii.wiimoteConnected || Wii.wiiUProControllerConnected) {
    if (Wii.getButtonClick(HOME)) {
      Wii.disconnect();
      wiiOldLed = 0; // Reset value
    }
  }
  if (!Wii.wiimoteConnected && !Wii.wiiUProControllerConnected)
    wiiInitialized = false;
#endif // ENABLE_WII
#ifdef ENABLE_XBOX
  if (!Xbox.Xbox360Connected[0])
    xboxInitialized = false;
#endif
#if defined(ENABLE_PS3) || defined(ENABLE_WII) || defined(ENABLE_XBOX)
  if (millis() - ledTimer > 1000) { // Update every 1s
    ledTimer = millis();
    updateLEDs();
  }
#endif // defined(ENABLE_PS3) || defined(ENABLE_WII) || defined(ENABLE_XBOX)
}

#if defined(ENABLE_PS3) || defined(ENABLE_WII) || defined(ENABLE_XBOX)
void updateLEDs() {
  uint8_t Led;
#ifdef ENABLE_PS3
  if (PS3.PS3Connected) {
    if (ps3RumbleEnable) {
      ps3RumbleEnable = false;
      PS3.setRumbleOn(RumbleLow);
      ps3RumbleDisable = true;
      ledTimer -= 500; // Turn off again in 500ms
    } else if (ps3RumbleDisable) {
      ps3RumbleDisable = false;
      PS3.setRumbleOff();
    } else {
      if (PS3.getStatus(Shutdown)) { // Blink all LEDs
        if (ps3OldLed)
          Led = 0x00; // All off
        else
          Led = 0x0F; // All on
      }
      else if (PS3.getStatus(Dying))
        Led = 0x01; // LED1 on
      else if (PS3.getStatus(Low))
        Led = 0x03; // LED1 and LED2 on
      else if (PS3.getStatus(High))
        Led = 0x07; // LED1, LED2 and LED3 on
      else if (PS3.getStatus(Full))
        Led = 0x0F; // LED1, LED2, LED3 and LED4 on
      else if (PS3.getStatus(Charging))
        Led = (ps3OldLed == 0x0F ? 0x01 : (ps3OldLed << 1 | 1) & 0x0F); // Indicate charging using the LEDs
      else
        Led = ps3OldLed;

      if (Led != ps3OldLed) {
        ps3OldLed = Led;
        PS3.setLedRaw(Led);
      }
    }
  } else if (PS3.PS3NavigationConnected) {
    if (PS3.getStatus(Shutdown))
      PS3.setLedToggle(LED1); // Blink LED
    else {
      Led = 0x01; // LED on
      if (Led != ps3OldLed) {
        ps3OldLed = Led;
        PS3.setLedRaw(Led);
      }
    }
  }
#endif // ENABLE_PS3
#ifdef ENABLE_WII
  if (Wii.wiimoteConnected || Wii.wiiUProControllerConnected) {
    if (wiiRumbleEnabled) {
      wiiRumbleEnabled = false;
      Wii.setRumbleOff();
    } else {
      uint8_t batteryLevel = Wii.getBatteryLevel();
      if (batteryLevel < 60) { // Blink all LEDs
        if (wiiOldLed)
          Led = 0x00; // All off
        else
          Led = 0xF0; // All on
      }
      else if (batteryLevel < 110)
        Led = 0x10; // LED1 on
      else if (batteryLevel < 160)
        Led = 0x30; // LED1 and LED2 on
      else if (batteryLevel < 210)
        Led = 0x70; // LED1, LED2 and LED3 on
      else
        Led = 0xF0; // LED1, LED2, LED3 and LED4 on
      if (Led != wiiOldLed) {
        wiiOldLed = Led;
        Wii.setLedRaw(Led);
      }
    }
  }
#endif // ENABLE_WII
#ifdef ENABLE_XBOX
  if (Xbox.Xbox360Connected[0]) {
    if (xboxRumbleEnabled) {
      xboxRumbleEnabled = false;
      Xbox.setRumbleOff();
    } else {
      uint8_t batteryLevel = Xbox.getBatteryLevel();
      LED xboxLed;
      if (batteryLevel == 0)
        xboxLed = LED1;
      else if (batteryLevel == 1)
        xboxLed = LED2;
      else if (batteryLevel == 2)
        xboxLed = LED3;
      else
        xboxLed = LED4;

      if (xboxLed != xboxOldLed) {
        xboxOldLed = xboxLed;
        Xbox.setLedOn(xboxLed);
      }
    }
  }
#endif // ENABLE_XBOX
}

void onInit() { // This function is called when a controller is first initialized
  updateLEDs(); // Turn the LEDs on according to the voltage level
#ifdef ENABLE_PS3
  if (PS3.PS3Connected && !ps3Initialized) { // We only check the normal PS3 controller as the Navigation controller doesn't have rumble support
    ps3Initialized = true;
    ledTimer = millis() - 500; // Wait 500ms before turning rumble on
    ps3RumbleEnable = true; // We can't sent commands to the PS3 controller that often, so we don't send the command here like the other controllers
  }
#endif // ENABLE_PS3
#ifdef ENABLE_WII
  if (Wii.wiimoteConnected && !wiiInitialized) { // Both the Wiimote and the Wii U Pro Controller
    wiiInitialized = true;
    ledTimer = millis() - 500; // This will turn the rumble off again after 500ms
    Wii.setRumbleOn();
    wiiRumbleEnabled = true;
  }
#endif // ENABLE_WII
#ifdef ENABLE_XBOX
  if (Xbox.Xbox360Connected[0] && !xboxInitialized) { // Xbox wireless controller
    xboxInitialized = true;
    ledTimer = millis() - 500; // This will turn the rumble off again after 500ms
    Xbox.setRumbleOn(0x00,0xFF);
    xboxRumbleEnabled = true;
  }
#endif // ENABLE_XBOX
}
#endif // defined(ENABLE_PS3) || defined(ENABLE_WII) || defined(ENABLE_XBOX)

#if defined(ENABLE_SPP) || defined(ENABLE_PS3) || defined(ENABLE_WII) || defined(ENABLE_XBOX)
void steer(Command command) {
  commandSent = true; // Used to see if there has already been send a command or not

  // Set all to false
  steerForward = false;
  steerBackward = false;
  steerStop = false;
  steerLeft = false;
  steerRight = false;

#ifdef ENABLE_SPP
  if (command == joystick) {
    if (sppData2 > 0) {
      targetOffset = scale(sppData2, 0, 1, 0, cfg.controlAngleLimit);
      steerForward = true;
    } else if (sppData2 < 0) {
      targetOffset = scale(sppData2, 0, -1, 0, cfg.controlAngleLimit);
      steerBackward = true;
    }
    if (sppData1 > 0) {
      turningOffset = scale(sppData1, 0, 1, 0, cfg.turningLimit);
      steerRight = true;
    } else if (sppData1 < 0) {
      turningOffset = scale(sppData1, 0, -1, 0, cfg.turningLimit);
      steerLeft = true;
    }
  } else if (command == imu) {
      if (sppData1 > 0) {
        targetOffset = scale(sppData1, 0, 36, 0, cfg.controlAngleLimit);
        steerForward = true;
      }
      else if (sppData1 < 0) {
        targetOffset = scale(sppData1, 0, -36, 0, cfg.controlAngleLimit);
        steerBackward = true;
      }
      if (sppData2 < 0) {
        turningOffset = scale(sppData2, 0, -45, 0, cfg.turningLimit);
        steerLeft = true;
      }
      else if (sppData2 > 0) {
        turningOffset = scale(sppData2, 0, 45, 0, cfg.turningLimit);
        steerRight = true;
      }
  }
#endif // ENABLE_SPP
#ifdef ENABLE_PS3
  if (command == updatePS3) {
    if (PS3.PS3Connected) {
      if (PS3.getAnalogHat(LeftHatY) < 117 && PS3.getAnalogHat(RightHatY) < 117) {
        targetOffset = scale(PS3.getAnalogHat(LeftHatY)+PS3.getAnalogHat(RightHatY), 232, 0, 0, cfg.controlAngleLimit); // Scale from 232-0 to 0-controlAngleLimit
        steerForward = true;
      } else if (PS3.getAnalogHat(LeftHatY) > 137 && PS3.getAnalogHat(RightHatY) > 137) {
        targetOffset = scale(PS3.getAnalogHat(LeftHatY)+PS3.getAnalogHat(RightHatY), 276, 510, 0, cfg.controlAngleLimit); // Scale from 276-510 to 0-controlAngleLimit
        steerBackward = true;
      }
      if (((int16_t)PS3.getAnalogHat(LeftHatY) - (int16_t)PS3.getAnalogHat(RightHatY)) > 15) {
        turningOffset = scale(abs((int16_t)PS3.getAnalogHat(LeftHatY) - (int16_t)PS3.getAnalogHat(RightHatY)), 0, 255, 0, cfg.turningLimit); // Scale from 0-255 to 0-turningLimit
        steerLeft = true;
      } else if (((int16_t)PS3.getAnalogHat(RightHatY) - (int16_t)PS3.getAnalogHat(LeftHatY)) > 15) {
        turningOffset = scale(abs((int16_t)PS3.getAnalogHat(LeftHatY) - (int16_t)PS3.getAnalogHat(RightHatY)), 0, 255, 0, cfg.turningLimit); // Scale from 0-255 to 0-turningLimit
        steerRight = true;
      }
      else if (PS3.getButtonPress(CROSS)) {
        if (PS3.getAngle(Pitch) > 180) {
          targetOffset = scale(PS3.getAngle(Pitch), 180, 216, 0, cfg.controlAngleLimit);
          steerForward = true;
        }
        else if (PS3.getAngle(Pitch) < 180) {
          targetOffset = scale(PS3.getAngle(Pitch), 180, 144, 0, cfg.controlAngleLimit);
          steerBackward = true;
        }
        if (PS3.getAngle(Roll) > 180) {
          turningOffset = scale(PS3.getAngle(Roll), 180, 225, 0, cfg.turningLimit);
          steerRight = true;
        }
        else if (PS3.getAngle(Roll) < 180) {
          turningOffset = scale(PS3.getAngle(Roll), 180, 135, 0, cfg.turningLimit);
          steerLeft = true;
        }
      }
    } else { // It must be a Navigation controller then
      if (PS3.getAnalogHat(LeftHatY) < 117) {
        targetOffset = scale(PS3.getAnalogHat(LeftHatY), 116, 0, 0, cfg.controlAngleLimit); // Scale from 116-0 to 0-controlAngleLimit
        steerForward = true;
      } else if (PS3.getAnalogHat(LeftHatY) > 137) {
        targetOffset = scale(PS3.getAnalogHat(LeftHatY), 138, 255, 0, cfg.controlAngleLimit); // Scale from 138-255 to 0-controlAngleLimit
        steerBackward = true;
      }
      if (PS3.getAnalogHat(LeftHatX) < 55) {
        turningOffset = scale(PS3.getAnalogHat(LeftHatX), 54, 0, 0, cfg.turningLimit); // Scale from 54-0 to 0-turningLimit
        steerLeft = true;
      } else if (PS3.getAnalogHat(LeftHatX) > 200) {
        turningOffset = scale(PS3.getAnalogHat(LeftHatX), 201, 255, 0, cfg.turningLimit); // Scale from 201-255 to 0-turningLimit
        steerRight = true;
      }
    }
  }
#endif // ENABLE_PS3
#ifdef ENABLE_WII
  if (command == updateWii) {
    if (!Wii.wiiUProControllerConnected) {
      if (Wii.getButtonPress(B)) {
        if (Wii.getPitch() > 180) {
          targetOffset = scale(Wii.getPitch(), 180, 216, 0, cfg.controlAngleLimit);
          steerForward = true;
        }
        else if (Wii.getPitch() < 180) {
          targetOffset = scale(Wii.getPitch(), 180, 144, 0, cfg.controlAngleLimit);
          steerBackward = true;
        }
        if (Wii.getRoll() > 180) {
          turningOffset = scale(Wii.getRoll(), 180, 225, 0, cfg.turningLimit);
          steerRight = true;
        }
        else if (Wii.getRoll() < 180) {
          turningOffset = scale(Wii.getRoll(), 180, 135, 0, cfg.turningLimit);
          steerLeft = true;
        }
      }
      else { // Read the Navigation controller
        if (Wii.getAnalogHat(HatY) > 137) {
          targetOffset = scale(Wii.getAnalogHat(HatY), 138, 230, 0, cfg.controlAngleLimit);
          steerForward = true;
        }
        else if (Wii.getAnalogHat(HatY) < 117) {
          targetOffset = scale(Wii.getAnalogHat(HatY), 116, 25, 0, cfg.controlAngleLimit);
          steerBackward = true;
        }
        if (Wii.getAnalogHat(HatX) > 137) {
          turningOffset = scale(Wii.getAnalogHat(HatX), 138, 230, 0, cfg.turningLimit);
          steerRight = true;
        }
        else if (Wii.getAnalogHat(HatX) < 117) {
          turningOffset = scale(Wii.getAnalogHat(HatX), 116, 25, 0, cfg.turningLimit);
          steerLeft = true;
        }
      }
    } else { // It must be a Wii U Pro Controller then
      if (Wii.getAnalogHat(LeftHatY) > 2200 && Wii.getAnalogHat(RightHatY) > 2200) {
        targetOffset = scale(Wii.getAnalogHat(LeftHatY)+Wii.getAnalogHat(RightHatY), 4402, 6400, 0, cfg.controlAngleLimit); // Scale from 4402-6400 to 0-controlAngleLimit
        steerForward = true;
      } else if (Wii.getAnalogHat(LeftHatY) < 1800 && Wii.getAnalogHat(RightHatY) < 1800) {
        targetOffset = scale(Wii.getAnalogHat(LeftHatY)+Wii.getAnalogHat(RightHatY), 3598, 1600, 0, cfg.controlAngleLimit); // Scale from 3598-1600 to 0-controlAngleLimit
        steerBackward = true;
      }
      if (((int16_t)Wii.getAnalogHat(RightHatY) - (int16_t)Wii.getAnalogHat(LeftHatY)) > 200) {
        turningOffset = scale(abs((int16_t)Wii.getAnalogHat(LeftHatY) - (int16_t)Wii.getAnalogHat(RightHatY)), 0, 2400, 0, cfg.turningLimit); // Scale from 0-2400 to 0-turningLimit
        steerLeft = true;
      } else if (((int16_t)Wii.getAnalogHat(LeftHatY) - (int16_t)Wii.getAnalogHat(RightHatY)) > 200) {
        turningOffset = scale(abs((int16_t)Wii.getAnalogHat(LeftHatY) - (int16_t)Wii.getAnalogHat(RightHatY)), 0, 2400, 0, cfg.turningLimit); // Scale from 0-2400 to 0-turningLimit
        steerRight = true;
      }
    }
  }
#endif // ENABLE_WII
#ifdef ENABLE_XBOX
  if (command == updateXbox) {
    if (Xbox.getAnalogHat(LeftHatY) > 7500 && Xbox.getAnalogHat(RightHatY) > 7500) {
      targetOffset = scale((int32_t)Xbox.getAnalogHat(LeftHatY)+(int32_t)Xbox.getAnalogHat(RightHatY), 15002, 65534, 0, cfg.controlAngleLimit); // Scale from 15002-65534 to 0-controlAngleLimit
      steerForward = true;
    } else if (Xbox.getAnalogHat(LeftHatY) < -7500 && Xbox.getAnalogHat(RightHatY) < -7500) {
      targetOffset = scale((int32_t)Xbox.getAnalogHat(LeftHatY)+(int32_t)Xbox.getAnalogHat(RightHatY), -15002, -65536, 0, cfg.controlAngleLimit); // Scale from -15002-(-65536) to 0-controlAngleLimit
      steerBackward = true;
    }
    if (((int32_t)Xbox.getAnalogHat(RightHatY) - (int32_t)Xbox.getAnalogHat(LeftHatY)) > 7500) {
      turningOffset = scale(abs((int32_t)Xbox.getAnalogHat(LeftHatY) - (int32_t)Xbox.getAnalogHat(RightHatY)), 0, 65535, 0, cfg.turningLimit); // Scale from 0-65535 to 0-turningLimit
      steerLeft = true;
    } else if (((int32_t)Xbox.getAnalogHat(LeftHatY) - (int32_t)Xbox.getAnalogHat(RightHatY)) > 7500) {
      turningOffset = scale(abs((int32_t)Xbox.getAnalogHat(LeftHatY) - (int32_t)Xbox.getAnalogHat(RightHatY)), 0, 65535, 0, cfg.turningLimit); // Scale from 0-65535 to 0-turningLimit
      steerRight = true;
    }
  }
#endif // ENABLE_XBOX

  if (command == stop) {
    steerStop = true;
    if (lastCommand != stop) { // Set new stop position
      targetPosition = getWheelPosition();
      stopped = false;
    }
  }
  lastCommand = command;
}
double scale(double input, double inputMin, double inputMax, double outputMin, double outputMax) { // Like map() just returns a double
  double output;
  if (inputMin < inputMax)
    output = (input-inputMin)/((inputMax-inputMin)/(outputMax-outputMin));
  else
    output = (inputMin-input)/((inputMin-inputMax)/(outputMax-outputMin));
  if (output > outputMax)
    output = outputMax;
  else if (output < outputMin)
    output = outputMin;
  return output;
}
#endif // defined(ENABLE_SPP) || defined(ENABLE_PS3) || defined(ENABLE_WII) || defined(ENABLE_XBOX)

#endif // ENABLE_USB
