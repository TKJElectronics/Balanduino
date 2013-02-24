double runTime;
char input[30];

void sendBluetoothData() {
  if(SerialBT.connected) {
    Usb.Task();
    if (sendPairConfirmation) {
      sendPairConfirmation = false;
      stringBuf[0] = 'W';
      stringBuf[1] = 'C';
      stringBuf[2] = '\0';
      
      SerialBT.println(stringBuf);
      dataTimer = micros(); // Reset the timer, to prevent it from sending data in the next loop
    } else if(sendPIDValues) {
      sendPIDValues = false;
      
      strcpy(stringBuf,"P,");
      SerialBT.doubleToString(Kp,convBuf); // We use this helper function in the SPP library to convert from a double to a string
      strcat(stringBuf,convBuf);
      
      strcat(stringBuf,",");
      SerialBT.doubleToString(Ki,convBuf);
      strcat(stringBuf,convBuf);
      
      strcat(stringBuf,",");
      SerialBT.doubleToString(Kd,convBuf);
      strcat(stringBuf,convBuf);
      
      strcat(stringBuf,",");
      SerialBT.doubleToString(targetAngle,convBuf);
      strcat(stringBuf,convBuf);
      
      SerialBT.println(stringBuf);
      dataTimer = micros(); // Reset the timer, to prevent it from sending data in the next loop
    } else if (sendSettings) {
      sendSettings = false;
      
      stringBuf[0] = 'S';
      stringBuf[1] = ',';
      if (BackToSpot)
        stringBuf[2] = '1';
      else
        stringBuf[2] = '0';        
      stringBuf[3] = ',';
      stringBuf[4] = '\0';
      
      itoa(controlAngleLimit,convBuf,10);
      strcat(stringBuf,convBuf); 
      strcat(stringBuf,","); 
      itoa(turningAngleLimit,convBuf,10);
      strcat(stringBuf,convBuf);       

      SerialBT.println(stringBuf);
      dataTimer = micros(); // Reset the timer, to prevent it from sending data in the next loop
    } else if (sendInfo) {      
      sendInfo = false;
  
      stringBuf[0] = 'I';
      stringBuf[1] = ',';
      stringBuf[2] = Version_Major + '0';
      stringBuf[3] = '.';
      stringBuf[4] = Version_Minor + '0';
      stringBuf[5] = '.';
      stringBuf[6] = Version_Patch + '0';      
      stringBuf[7] = ',';      
      stringBuf[8] = '\0';
      
      #if defined(__AVR_ATmega644__)
        strcat(stringBuf,"644,"); 
      #elif defined(__AVR_ATmega1284__)
        strcat(stringBuf,"1284,"); 
      #else
        strcat(stringBuf,"?,"); 
      #endif
      
      //itoa(batteryLevel,convBuf,10);
      runTime = (double)millis() / 60000.0;
      SerialBT.doubleToString(runTime,convBuf);
      strcat(stringBuf,convBuf); 
      strcat(stringBuf, " min");
      
      SerialBT.println(stringBuf);
      dataTimer = micros(); // Reset the timer, to prevent it from sending data in the next loop
    } else if(sendData) {
      if(micros() - dataTimer > 50000) { // Send data every 50ms
        dataTimer = micros();
        strcpy(stringBuf,"V,");
        SerialBT.doubleToString(accAngle,convBuf); // We use this helper function in the SPP library to convert from a double to a string
        strcat(stringBuf,convBuf);
        
        strcat(stringBuf,",");
        SerialBT.doubleToString(gyroAngle,convBuf);
        strcat(stringBuf,convBuf);
        
        strcat(stringBuf,",");
        SerialBT.doubleToString(pitch,convBuf);
        strcat(stringBuf,convBuf);
        
        SerialBT.println(stringBuf);
      }
    }
  }
}
void readBTD() {
  Usb.Task();
  if(SerialBT.connected) { // The SPP connection won't return data as fast as the controllers, so we will handle it separately
    if(SerialBT.available()) {
      char input[30];
      uint8_t i = 0;
      while(1) {
        input[i] = SerialBT.read();   
        if(input[i] == 0) // Error while reading the string
          return;
        if(input[i] == ';') // Keep reading until it reads a semicolon
          break;            
        i++;
      }
      
      if(input[0] == 'A') { // Abort
        stopAndReset();
        while(SerialBT.read() != 'C') // Wait until continue is send
          Usb.Task();
      }     
      
      /* For sending PID and IMU values */
      else if(input[0] == 'G') { // The Processing/Android application sends when it needs the PID or IMU values
        if(input[1] == 'P') // PID Values
          sendPIDValues = true;      
        else if(input[1] == 'S') // Get settings
          sendSettings = true;
        else if(input[1] == 'I') // Get Firmware version
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
          if (input[2] == ',' && input[3] == '0') {
            BackToSpot = 0;
            updateBackToSpot();
          } else if (input[2] == ',' && input[3] == '1') {
            BackToSpot = 1;
            updateBackToSpot();
          }            
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
          strtok(input, ","); // Ignore 'J'
          sppData1 = atof(strtok(NULL, ",")); // x-axis
          sppData2 = atof(strtok(NULL, ";")); // y-axis
          steer(joystick);
        }
        else if(input[1] == 'M') { // IMU
          strtok(input, ","); // Ignore 'M'
          sppData1 = atof(strtok(NULL, ",")); // Pitch
          sppData2 = atof(strtok(NULL, ";")); // Roll
          steer(imu);
        }
        
        else if(input[1] == 'W') { // Pair with a new Wiimote or Wii U Pro Controller
          Btd.pairWithWiimote();
          sendPairConfirmation = true;  
        } else if(input[1] == 'R') {
          restoreEEPROMValues(); // Restore the default PID values and target angle
          sendPIDValues = true;
          sendSettings = true;
        }         
      }
    }
  } else {
    commandSent = false; // We use this to detect when there has already been sent a command by one of the controllers
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
    if(Xbox.XboxReceiverConnected && Xbox.Xbox360Connected[0] && !commandSent) { // We will only read from the first controller, up to four is supported by one receiver
      if(Xbox.getButtonPress(0,BACK)) {
        stopAndReset();
        while(!Xbox.getButtonPress(0,START))
          Usb.Task();
      }
      else if((Xbox.getAnalogHat(0,LeftHatY) < -7500) || (Xbox.getAnalogHat(0,RightHatY) < -7500) || (Xbox.getAnalogHat(0,LeftHatY) > 7500) || (Xbox.getAnalogHat(0,RightHatY) > 7500))
        steer(updateXbox);
    }
    if(!commandSent) // If there hasn't been send a command by now, then send stop
      steer(stop);
  }
  if(PS3.PS3Connected || PS3.PS3NavigationConnected) {
    if(PS3.getButtonClick(PS))
      PS3.disconnect();
  }
  if(Wii.wiimoteConnected || Wii.wiiUProControllerConnected) {
    if(Wii.getButtonClick(HOME))
      Wii.disconnect();
  }
}
void steer(Command command) {
  commandSent = true; // Used to see if there has already been send a command or not
  
  // Set all to false
  steerForward = false;
  steerBackward = false;
  steerStop = false;
  steerLeft = false;
  steerRight = false;
  
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
  } else if(command == updatePS3) {
    if(PS3.PS3Connected) {
      if(PS3.getAnalogHat(LeftHatY) < 117 && PS3.getAnalogHat(RightHatY) < 117) {
        targetOffset = scale(PS3.getAnalogHat(LeftHatY)+PS3.getAnalogHat(RightHatY),232,0,0,controlAngleLimit); // Scale from 232-0 to 0-controlAngleLimit
        steerForward = true;
      } else if(PS3.getAnalogHat(LeftHatY) > 137 && PS3.getAnalogHat(RightHatY) > 137) {
        targetOffset = scale(PS3.getAnalogHat(LeftHatY)+PS3.getAnalogHat(RightHatY),276,510,0,controlAngleLimit); // Scale from 276-510 to 0-controlAngleLimit
        steerBackward = true;
      }
      if(((int16_t)PS3.getAnalogHat(LeftHatY) - (int16_t)PS3.getAnalogHat(RightHatY)) > 15) {
        turningOffset = scale(abs((int16_t)PS3.getAnalogHat(LeftHatY) - (int16_t)PS3.getAnalogHat(RightHatY)),0,255,0,turningAngleLimit); // Scale from 0-255 to 0-20
        steerLeft = true;
      } else if(((int16_t)PS3.getAnalogHat(RightHatY) - (int16_t)PS3.getAnalogHat(LeftHatY)) > 15) {
        turningOffset = scale(abs((int16_t)PS3.getAnalogHat(LeftHatY) - (int16_t)PS3.getAnalogHat(RightHatY)),0,255,0,turningAngleLimit); // Scale from 0-255 to 0-20
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
        turningOffset = scale(PS3.getAnalogHat(LeftHatX),54,0,0,turningAngleLimit); // Scale from 54-0 to 0-20
        steerLeft = true;
      } else if(PS3.getAnalogHat(LeftHatX) > 200) {
        turningOffset = scale(PS3.getAnalogHat(LeftHatX),201,255,0,turningAngleLimit); // Scale from 201-255 to 0-20
        steerRight = true;
      }
    }
  }
  else if(command == updateWii) {
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
        turningOffset = scale(abs((int16_t)Wii.getAnalogHat(LeftHatY) - (int16_t)Wii.getAnalogHat(RightHatY)),0,2400,0,turningAngleLimit); // Scale from 0-2400 to 0-20
        steerLeft = true;
      } else if(((int16_t)Wii.getAnalogHat(LeftHatY) - (int16_t)Wii.getAnalogHat(RightHatY)) > 200) {
        turningOffset = scale(abs((int16_t)Wii.getAnalogHat(LeftHatY) - (int16_t)Wii.getAnalogHat(RightHatY)),0,2400,0,turningAngleLimit); // Scale from 0-2400 to 0-20
        steerRight = true;
      }
    }
  }
  else if(command == updateXbox) {
    if(Xbox.getAnalogHat(0,LeftHatY) > 7500 && Xbox.getAnalogHat(0,RightHatY) > 7500) {
      targetOffset = scale((int32_t)Xbox.getAnalogHat(0,LeftHatY)+(int32_t)Xbox.getAnalogHat(0,RightHatY),15002,65534,0,controlAngleLimit); // Scale from 15002-65534 to 0-controlAngleLimit
      steerForward = true;
    } else if(Xbox.getAnalogHat(0,LeftHatY) < -7500 && Xbox.getAnalogHat(0,RightHatY) < -7500) {
      targetOffset = scale((int32_t)Xbox.getAnalogHat(0,LeftHatY)+(int32_t)Xbox.getAnalogHat(0,RightHatY),-15002,-65536,0,controlAngleLimit); // Scale from -15002-(-65536) to 0-controlAngleLimit
      steerBackward = true;
    }
    if(((int32_t)Xbox.getAnalogHat(0,RightHatY) - (int32_t)Xbox.getAnalogHat(0,LeftHatY)) > 7500) {
      turningOffset = scale(abs((int32_t)Xbox.getAnalogHat(0,LeftHatY) - (int32_t)Xbox.getAnalogHat(0,RightHatY)),0,65535,0,turningAngleLimit); // Scale from 0-65535 to 0-20
      steerLeft = true;
    } else if(((int32_t)Xbox.getAnalogHat(0,LeftHatY) - (int32_t)Xbox.getAnalogHat(0,RightHatY)) > 7500) {
      turningOffset = scale(abs((int32_t)Xbox.getAnalogHat(0,LeftHatY) - (int32_t)Xbox.getAnalogHat(0,RightHatY)),0,65535,0,turningAngleLimit); // Scale from 0-65535 to 0-20
      steerRight = true;
    }
  }
  
  else if(command == stop) {
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
