#ifdef ENABLE_USB

uint8_t ps3OldLed, wiiOldLed;
#ifdef ENABLE_XBOX
LED xboxOldLed;
#endif

#ifdef ENABLE_SPP
void sendBluetoothData() {
  if(SerialBT.connected && (millis() - dataTimer > 50)) {  // Only send data every 50ms
    if(sendPairConfirmation) {
      sendPairConfirmation = false;
      dataTimer = millis(); // Reset the timer, to prevent it from sending data in the next loop
      
      SerialBT.println("WC");
    } else if(sendPIDValues) {
      sendPIDValues = false;
      dataTimer = millis(); // Reset the timer, to prevent it from sending data in the next loop
      
      SerialBT.print("P,");
      SerialBT.print(Kp);
      SerialBT.print(',');
      SerialBT.print(Ki);
      SerialBT.print(',');
      SerialBT.print(Kd);
      SerialBT.print(',');
      SerialBT.println(targetAngle);
    } else if(sendSettings) {
      sendSettings = false;
      dataTimer = millis(); // Reset the timer, to prevent it from sending data in the next loop
      
      SerialBT.print("S,");
      SerialBT.print(BackToSpot);
      SerialBT.print(',');
      SerialBT.print(controlAngleLimit);
      SerialBT.print(',');
      SerialBT.println(turningAngleLimit);
    } else if(sendInfo) {      
      sendInfo = false;
      dataTimer = millis(); // Reset the timer, to prevent it from sending data in the next loop
      
      SerialBT.print("I,");
      SerialBT.print(version);
      
      #if defined(__AVR_ATmega644__)
        SerialBT.print(",ATmega644,");
      #elif defined(__AVR_ATmega1284P__)
        SerialBT.print(",ATmega1284P,");
      #else
        SerialBT.print(",Unknown,");
      #endif
      
      SerialBT.print(batteryLevel);
      SerialBT.print("V,");
      SerialBT.println((double)millis()/60000.0);
    } else if(sendData) {
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
  if(SerialBT.connected) { // The SPP connection won't return data as fast as the controllers, so we will handle it separately
    if(SerialBT.available()) {
      char input[30];
      uint8_t i = 0;
      while(1) {
        input[i] = SerialBT.read();   
        if(input[i] == -1) // Error while reading the string
          return;
        if(input[i] == ';') // Keep reading until it reads a semicolon
          break;
        i++;
        if(i >= sizeof(input)/sizeof(input[0])) // String is too long
          return;
      }
      
      if(input[0] == 'A') { // Abort
        stopAndReset();
        while(SerialBT.read() != 'C') // Wait until continue is send
          Usb.Task();
      }
      
      /* For sending PID and IMU values */
      else if(input[0] == 'G') { // The Processing/Android application sends when it needs the PID, settings or info
        if(input[1] == 'P') // Get PID Values
          sendPIDValues = true;
        else if(input[1] == 'S') // Get settings
          sendSettings = true;
        else if(input[1] == 'I') // Get info
          sendInfo = true;
      }
      
      else if(input[0] == 'S') { // Set different values     
        /* Set PID and target angle */
        if(input[1] == 'P') {
          strtok(input, ","); // Ignore 'P'
          Kp = atof(strtok(NULL, ";"));
          updateKp();
        } else if(input[1] == 'I') {
          strtok(input, ","); // Ignore 'I'
          Ki = atof(strtok(NULL, ";"));
          updateKi();
        } else if(input[1] == 'D') {
          strtok(input, ","); // Ignore 'D'
          Kd = atof(strtok(NULL, ";"));
          updateKd();
        } else if(input[1] == 'T') { // Target Angle
          strtok(input, ","); // Ignore 'T'
          targetAngle = atof(strtok(NULL, ";"));
          updateTargetAngle();        
        }   
        else if(input[1] == 'A') { // Controlling max angle
          strtok(input, ","); // Ignore 'A'
          controlAngleLimit = atoi(strtok(NULL, ";"));
          updateControlAngleLimit();
        } else if(input[1] == 'U') { // Turning max angle
          strtok(input, ","); // Ignore 'U'
          turningAngleLimit = atoi(strtok(NULL, ";"));
          updateTurningAngleLimit();             
        }

        else if(input[1] == 'B') { // Set Back To Spot
          BackToSpot = input[3] - '0'; // Convert from ASCII to number
          updateBackToSpot();
        }
      }

      else if(input[0] == 'I') { // IMU trasmitting states
        if(input[1] == 'B') // Begin sending IMU values
          sendData = true; // Send output to Processing/Android application
        else if(input[1] == 'S') // Stop sending IMU values
          sendData = false; // Stop sending output to Processing/Android application
      }     

      else if(input[0] == 'C') { // Commands
        if(input[1] == 'S') // Stop
          steer(stop);
        else if(input[1] == 'J') { // Joystick
          SPPreceiveControlTimestamp = millis();
          strtok(input, ","); // Ignore 'J'
          sppData1 = atof(strtok(NULL, ",")); // x-axis
          sppData2 = atof(strtok(NULL, ";")); // y-axis
          steer(joystick);
        }
        else if(input[1] == 'M') { // IMU
          SPPreceiveControlTimestamp = millis();
          strtok(input, ","); // Ignore 'M'
          sppData1 = atof(strtok(NULL, ",")); // Pitch
          sppData2 = atof(strtok(NULL, ";")); // Roll
          steer(imu);
        }
#ifdef ENABLE_WII
        else if(input[1] == 'W') { // Pair with a new Wiimote or Wii U Pro Controller
          Wii.pair();
          sendPairConfirmation = true;  
        }
#endif // ENABLE_WII
        else if(input[1] == 'R') {
          restoreEEPROMValues(); // Restore the default PID values and target angle
          sendPIDValues = true;
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
  
  if(millis() > (SPPreceiveControlTimestamp+SPPreceiveControlTimeout)) {
    commandSent = false; // We use this to detect when there has already been sent a command by one of the controllers
#ifdef ENABLE_PS3
    if(PS3.PS3Connected) {
      if(PS3.getButtonPress(SELECT)) {
        stopAndReset();
        while(!PS3.getButtonPress(START))
          Usb.Task();
      }
      else if((PS3.getAnalogHat(LeftHatY) < 117) || (PS3.getAnalogHat(RightHatY) < 117) || (PS3.getAnalogHat(LeftHatY) > 137) || (PS3.getAnalogHat(RightHatY) > 137))
        steer(updatePS3);
    } else if(PS3.PS3NavigationConnected) {
      if(PS3.getAnalogHat(LeftHatX) > 200 || PS3.getAnalogHat(LeftHatX) < 55 || PS3.getAnalogHat(LeftHatY) > 137 || PS3.getAnalogHat(LeftHatY) < 117)
        steer(updatePS3);
    }
#endif // ENABLE_PS3
#ifdef ENABLE_WII
    if(Wii.wiimoteConnected && !Wii.wiiUProControllerConnected && !commandSent) {
      if(Wii.getButtonPress(B))
        steer(updateWii);
      else if(Wii.nunchuckConnected && (Wii.getAnalogHat(HatX) > 137 || Wii.getAnalogHat(HatX) < 117 || Wii.getAnalogHat(HatY) > 137 || Wii.getAnalogHat(HatY) < 117))
        steer(updateWii);
    } else if(Wii.wiiUProControllerConnected && !commandSent) { // The Wii U Pro Controller Joysticks has an range from approximately 800-3200
      if(Wii.getButtonPress(MINUS)) {
        stopAndReset();
        while(!Wii.getButtonPress(PLUS))
          Usb.Task();
      }
      else if(Wii.getAnalogHat(LeftHatY) > 2200 || Wii.getAnalogHat(LeftHatY) < 1800 || Wii.getAnalogHat(RightHatY) > 2200 || Wii.getAnalogHat(RightHatY) < 1800)
        steer(updateWii);
    }
#endif // ENABLE_WII
#ifdef ENABLE_XBOX
    if(Xbox.Xbox360Connected[0] && !commandSent) { // We will only read from the first controller, up to four is supported by one receiver
      if(Xbox.getButtonPress(0,BACK)) {
        stopAndReset();
        while(!Xbox.getButtonPress(0,START))
          Usb.Task();
      }
      else if((Xbox.getAnalogHat(0,LeftHatY) < -7500) || (Xbox.getAnalogHat(0,RightHatY) < -7500) || (Xbox.getAnalogHat(0,LeftHatY) > 7500) || (Xbox.getAnalogHat(0,RightHatY) > 7500))
        steer(updateXbox);
    }
#endif // ENABLE_XBOX
    if(!commandSent) // If there hasn't been send a command by now, then send stop
      steer(stop);
  }
  
#ifdef ENABLE_PS3
  if(PS3.PS3Connected) { // Normal PS3 controller
    if(!ps3Rumble) {
      ledTimer = millis() - 500; // Wait 500ms before turning rumble on
      ps3Rumble = true;
      ps3RumbleEnable = true; // We can't sent command to the PS3 controller that often, so we don't send the command here like the other controllers
    }
  } else
    ps3Rumble = false;
#endif // ENABLE_PS3
#ifdef ENABLE_WII
  if(Wii.wiimoteConnected) { // Both the Wiimote and the Wii U Pro Controller
    if(!wiiRumble) {
      ledTimer = millis() - 500; // This will turn the rumble off again after 500ms
      Wii.setRumbleOn();
      wiiRumble = true;
      wiiRumbleEnabled = true;      
    }
  } else
    wiiRumble = false;
#endif // ENABLE_WII
#ifdef ENABLE_XBOX
  if(Xbox.Xbox360Connected[0]) { // Xbox wireless controller
    if(!xboxRumble) {
      ledTimer = millis() - 500; // This will turn the rumble off again after 500ms
      Xbox.setRumbleOn(0,0x00,0xFF);
      xboxRumble = true;
      xboxRumbleEnabled = true;      
    }
  } else
    xboxRumble = false;
#endif // ENABLE_XBOX

#ifdef ENABLE_PS3
  if(PS3.PS3Connected || PS3.PS3NavigationConnected) {
    if(PS3.getButtonClick(PS)) {
      PS3.disconnect();
      ps3OldLed = 0; // Reset value
    }
  }
#endif // ENABLE_PS3
#ifdef ENABLE_WII
  if(Wii.wiimoteConnected || Wii.wiiUProControllerConnected) {
    if(Wii.getButtonClick(HOME)) {
      Wii.disconnect();
      wiiOldLed = 0; // Reset value
    }
  }
#endif // ENABLE_WII
#if defined(ENABLE_PS3) || defined(ENABLE_WII) || defined(ENABLE_XBOX)
  if(millis() - ledTimer > 1000) { // Update every 1s
    ledTimer = millis();
    updateLEDs();    
  }
#endif // defined(ENABLE_PS3) || defined(ENABLE_WII) || defined(ENABLE_XBOX)
}

#if defined(ENABLE_PS3) || defined(ENABLE_WII) || defined(ENABLE_XBOX)
void updateLEDs() {
  uint8_t Led;
#ifdef ENABLE_PS3
  if(PS3.PS3Connected) {
    if(ps3RumbleEnable) {
      ps3RumbleEnable = false;      
      PS3.setRumbleOn(RumbleLow);
      ps3RumbleDisable = true;
      ledTimer -= 500; // Turn off again in 500ms
    } else if(ps3RumbleDisable) {
      ps3RumbleDisable = false;
      PS3.setRumbleOff();
      ledTimer -= 500; // Run this function again in 500ms
    } else {
      if(PS3.getStatus(Shutdown)) { // Blink all LEDs
        if(ps3OldLed)
          Led = 0x00;
        else
          Led = 0x0F;
      }
      else if(PS3.getStatus(Dying))
        Led = 0x01;
      else if(PS3.getStatus(Low))
        Led = 0x03;
      else if(PS3.getStatus(High))
        Led = 0x07;
      else if(PS3.getStatus(Full))
        Led = 0x0F;
      if(Led != ps3OldLed) {
        ps3OldLed = Led;
        PS3.setLedRaw(Led << 1);
      }
    }
  } else if(PS3.PS3NavigationConnected) {
    if(PS3.getStatus(Shutdown))
      PS3.setLedToggle(LED1); // Blink LED
  }
#endif // ENABLE_PS3
#ifdef ENABLE_WII
  if(Wii.wiimoteConnected || Wii.wiiUProControllerConnected) {
    if(wiiRumbleEnabled) {
      wiiRumbleEnabled = false;
      Wii.setRumbleOff();
    }
    uint8_t batteryLevel = Wii.getBatteryLevel();
    //Serial.print("BatteryLevel: ");Serial.println(batteryLevel);
    if(batteryLevel < 60) { // Blink all LEDs
      if(wiiOldLed)
        Led = 0x00;
      else
        Led = 0xF0;
    }
    else if(batteryLevel < 110)
      Led = 0x10;
    else if(batteryLevel < 160)
      Led = 0x30;
    else if(batteryLevel < 210)
      Led = 0x70;
    else
      Led = 0xF0;
    if(Led != wiiOldLed) {
      wiiOldLed = Led;
      Wii.setLedRaw(Led);      
    }
    Wii.statusRequest(); // This will update battery level
  }
#endif // ENABLE_WII
#ifdef ENABLE_XBOX
  if(Xbox.Xbox360Connected[0]) {
    if(xboxRumbleEnabled) {
      xboxRumbleEnabled = false;
      Xbox.setRumbleOff(0);
    }
    uint8_t batteryLevel = Xbox.getBatteryLevel(0);
    //Serial.print("BatteryLevel: ");Serial.println(batteryLevel);
    LED xboxLed;
    if(batteryLevel == 0)
      xboxLed = LED1;
    else if(batteryLevel == 1)
      xboxLed = LED2;
    else if(batteryLevel == 2)
      xboxLed = LED3;
    else
      xboxLed = LED4;
    if(xboxLed != xboxOldLed) {
      xboxOldLed = xboxLed;
      Xbox.setLedOn(0,xboxLed);
    }
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
  if(command == joystick) {
    if(sppData2 > 0) {
      targetOffset = scale(sppData2,0,1,0,controlAngleLimit);
      steerForward = true;
    } else if(sppData2 < 0) {
      targetOffset = scale(sppData2,0,-1,0,controlAngleLimit);
      steerBackward = true;
    }
    if(sppData1 > 0) {
      turningOffset = scale(sppData1,0,1,0,turningAngleLimit);
      steerRight = true;
    } else if(sppData1 < 0) {
      turningOffset = scale(sppData1,0,-1,0,turningAngleLimit);
      steerLeft = true;
    }
  } else if(command == imu) {
      if(sppData2 > 0) {
        targetOffset = scale(sppData2,0,36,0,controlAngleLimit);
        steerForward = true;
      }
      else if(sppData2 < 0) {
        targetOffset = scale(sppData2,0,-36,0,controlAngleLimit);
        steerBackward = true;
      }
      if(sppData1 > 0) {
        turningOffset = scale(sppData1,0,45,0,turningAngleLimit);
        steerLeft = true;
      }
      else if(sppData1 < 0) {
        turningOffset = scale(sppData1,0,-45,0,turningAngleLimit);
        steerRight = true;
      }
  }
#endif // ENABLE_SPP
#ifdef ENABLE_PS3
  if(command == updatePS3) {
    if(PS3.PS3Connected) {
      if(PS3.getAnalogHat(LeftHatY) < 117 && PS3.getAnalogHat(RightHatY) < 117) {
        targetOffset = scale(PS3.getAnalogHat(LeftHatY)+PS3.getAnalogHat(RightHatY),232,0,0,controlAngleLimit); // Scale from 232-0 to 0-controlAngleLimit
        steerForward = true;
      } else if(PS3.getAnalogHat(LeftHatY) > 137 && PS3.getAnalogHat(RightHatY) > 137) {
        targetOffset = scale(PS3.getAnalogHat(LeftHatY)+PS3.getAnalogHat(RightHatY),276,510,0,controlAngleLimit); // Scale from 276-510 to 0-controlAngleLimit
        steerBackward = true;
      }
      if(((int16_t)PS3.getAnalogHat(LeftHatY) - (int16_t)PS3.getAnalogHat(RightHatY)) > 15) {
        turningOffset = scale(abs((int16_t)PS3.getAnalogHat(LeftHatY) - (int16_t)PS3.getAnalogHat(RightHatY)),0,255,0,turningAngleLimit); // Scale from 0-255 to 0-turningAngleLimit
        steerLeft = true;
      } else if(((int16_t)PS3.getAnalogHat(RightHatY) - (int16_t)PS3.getAnalogHat(LeftHatY)) > 15) {
        turningOffset = scale(abs((int16_t)PS3.getAnalogHat(LeftHatY) - (int16_t)PS3.getAnalogHat(RightHatY)),0,255,0,turningAngleLimit); // Scale from 0-255 to 0-turningAngleLimit
        steerRight = true;
      }
    } else { // It must be a Navigation controller then
      if(PS3.getAnalogHat(LeftHatY) < 117) {
        targetOffset = scale(PS3.getAnalogHat(LeftHatY),116,0,0,controlAngleLimit); // Scale from 116-0 to 0-controlAngleLimit
        steerForward = true;
      } else if(PS3.getAnalogHat(LeftHatY) > 137) {
        targetOffset = scale(PS3.getAnalogHat(LeftHatY),138,255,0,controlAngleLimit); // Scale from 138-255 to 0-controlAngleLimit
        steerBackward = true;
      }
      if(PS3.getAnalogHat(LeftHatX) < 55) {
        turningOffset = scale(PS3.getAnalogHat(LeftHatX),54,0,0,turningAngleLimit); // Scale from 54-0 to 0-turningAngleLimit
        steerLeft = true;
      } else if(PS3.getAnalogHat(LeftHatX) > 200) {
        turningOffset = scale(PS3.getAnalogHat(LeftHatX),201,255,0,turningAngleLimit); // Scale from 201-255 to 0-turningAngleLimit
        steerRight = true;
      }
    }
  }
#endif // ENABLE_PS3
#ifdef ENABLE_WII
  if(command == updateWii) {
    if(!Wii.wiiUProControllerConnected) {
      if(Wii.getButtonPress(B)) {
        if(Wii.getPitch() > 180) {
          targetOffset = scale(Wii.getPitch(),180,216,0,controlAngleLimit);
          steerForward = true;
        }
        else if(Wii.getPitch() < 180) {
          targetOffset = scale(Wii.getPitch(),180,144,0,controlAngleLimit);
          steerBackward = true;
        }
        if(Wii.getRoll() > 180) {
          turningOffset = scale(Wii.getRoll(),180,225,0,turningAngleLimit);
          steerRight = true;
        }
        else if(Wii.getRoll() < 180) {
          turningOffset = scale(Wii.getRoll(),180,135,0,turningAngleLimit);
          steerLeft = true;
        }
      }
      else { // Read the Navigation controller
        if(Wii.getAnalogHat(HatY) > 137) {
          targetOffset = scale(Wii.getAnalogHat(HatY),138,230,0,controlAngleLimit);
          steerForward = true;
        }
        else if(Wii.getAnalogHat(HatY) < 117) {
          targetOffset = scale(Wii.getAnalogHat(HatY),116,25,0,controlAngleLimit);
          steerBackward = true;
        }
        if(Wii.getAnalogHat(HatX) > 137) {
          turningOffset = scale(Wii.getAnalogHat(HatX),138,230,0,turningAngleLimit);
          steerRight = true;
        }
        else if(Wii.getAnalogHat(HatX) < 117) {
          turningOffset = scale(Wii.getAnalogHat(HatX),116,25,0,turningAngleLimit);
          steerLeft = true;
        }
      }
    } else { // It must be a Wii U Pro Controller then
      if(Wii.getAnalogHat(LeftHatY) > 2200 && Wii.getAnalogHat(RightHatY) > 2200) {
        targetOffset = scale(Wii.getAnalogHat(LeftHatY)+Wii.getAnalogHat(RightHatY),4402,6400,0,controlAngleLimit); // Scale from 4402-6400 to 0-controlAngleLimit
        steerForward = true;
      } else if(Wii.getAnalogHat(LeftHatY) < 1800 && Wii.getAnalogHat(RightHatY) < 1800) {
        targetOffset = scale(Wii.getAnalogHat(LeftHatY)+Wii.getAnalogHat(RightHatY),3598,1600,0,controlAngleLimit); // Scale from 3598-1600 to 0-controlAngleLimit
        steerBackward = true;
      }
      if(((int16_t)Wii.getAnalogHat(RightHatY) - (int16_t)Wii.getAnalogHat(LeftHatY)) > 200) {
        turningOffset = scale(abs((int16_t)Wii.getAnalogHat(LeftHatY) - (int16_t)Wii.getAnalogHat(RightHatY)),0,2400,0,turningAngleLimit); // Scale from 0-2400 to 0-turningAngleLimit
        steerLeft = true;
      } else if(((int16_t)Wii.getAnalogHat(LeftHatY) - (int16_t)Wii.getAnalogHat(RightHatY)) > 200) {
        turningOffset = scale(abs((int16_t)Wii.getAnalogHat(LeftHatY) - (int16_t)Wii.getAnalogHat(RightHatY)),0,2400,0,turningAngleLimit); // Scale from 0-2400 to 0-turningAngleLimit
        steerRight = true;
      }
    }
  }
#endif // ENABLE_WII
#ifdef ENABLE_XBOX
  if(command == updateXbox) {
    if(Xbox.getAnalogHat(0,LeftHatY) > 7500 && Xbox.getAnalogHat(0,RightHatY) > 7500) {
      targetOffset = scale((int32_t)Xbox.getAnalogHat(0,LeftHatY)+(int32_t)Xbox.getAnalogHat(0,RightHatY),15002,65534,0,controlAngleLimit); // Scale from 15002-65534 to 0-controlAngleLimit
      steerForward = true;
    } else if(Xbox.getAnalogHat(0,LeftHatY) < -7500 && Xbox.getAnalogHat(0,RightHatY) < -7500) {
      targetOffset = scale((int32_t)Xbox.getAnalogHat(0,LeftHatY)+(int32_t)Xbox.getAnalogHat(0,RightHatY),-15002,-65536,0,controlAngleLimit); // Scale from -15002-(-65536) to 0-controlAngleLimit
      steerBackward = true;
    }
    if(((int32_t)Xbox.getAnalogHat(0,RightHatY) - (int32_t)Xbox.getAnalogHat(0,LeftHatY)) > 7500) {
      turningOffset = scale(abs((int32_t)Xbox.getAnalogHat(0,LeftHatY) - (int32_t)Xbox.getAnalogHat(0,RightHatY)),0,65535,0,turningAngleLimit); // Scale from 0-65535 to 0-turningAngleLimit
      steerLeft = true;
    } else if(((int32_t)Xbox.getAnalogHat(0,LeftHatY) - (int32_t)Xbox.getAnalogHat(0,RightHatY)) > 7500) {
      turningOffset = scale(abs((int32_t)Xbox.getAnalogHat(0,LeftHatY) - (int32_t)Xbox.getAnalogHat(0,RightHatY)),0,65535,0,turningAngleLimit); // Scale from 0-65535 to 0-turningAngleLimit
      steerRight = true;
    }
  }
#endif // ENABLE_XBOX
  
  if(command == stop) {
    steerStop = true;
    if(lastCommand != stop) { // Set new stop position
      targetPosition = wheelPosition;
      stopped = false;
    }
  }
  lastCommand = command;
}
double scale(double input, double inputMin, double inputMax, double outputMin, double outputMax) { // Like map() just returns a double
  double output;
  if(inputMin < inputMax)
    output = (input-inputMin)/((inputMax-inputMin)/(outputMax-outputMin));
  else
    output = (inputMin-input)/((inputMin-inputMax)/(outputMax-outputMin));
  if(output > outputMax)
    output = outputMax;
  else if(output < outputMin)
    output = outputMin;
  return output;
}
#endif // defined(ENABLE_SPP) || defined(ENABLE_PS3) || defined(ENABLE_WII) || defined(ENABLE_XBOX)

#endif // ENABLE_USB
