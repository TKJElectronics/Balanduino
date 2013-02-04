/*
 * The code is released under the GNU General Public License.
 * Developed by Kristian Lauszus
 * This is the algorithm for the Balanduino balancing robot.
 * It can be controlled by either an Android app or a Processing application via bluetooth.
 * The Android app can be found at the following link: https://github.com/TKJElectronics/BalanduinoAndroidApp
 * The Processing application can be found here: https://github.com/TKJElectronics/BalanduinoProcessingApp
 * It can also be controlled by a PS3, Wii or a Xbox controller
 * For details, see http://blog.tkjelectronics.dk/2012/03/the-balancing-robot/
 */

#include "Balanduino.h"
#include <Wire.h>

/* 
 * These are all open source libraries written by Kristian Lauszus, TKJ Electronics
 * The USB libraries are located at the following link: https://github.com/felis/USB_Host_Shield_2.0
 */
#include <Kalman.h> // Kalman filter library see: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/
#include <XBOXRECV.h>
#include <SPP.h>
#include <PS3BT.h>
#include <Wii.h>

// Create the Kalman library instance
Kalman kalman; // See https://github.com/TKJElectronics/KalmanFilter for source code

// This will take care of all USB communication
USB Usb;

// You have to connect a Xbox wireless receiver to the Arduino to control it with a wirelessly Xbox controller
XBOXRECV Xbox(&Usb); // Uncomment DEBUG in "XBOXRECV.cpp" to save space
BTD Btd(&Usb); // This is the main Bluetooth library, it will take care of all the Bluetooth communication - Uncomment DEBUG in "BTD.cpp" to save space
SPP SerialBT(&Btd,"Balanduino","0000"); // Also uncomment DEBUG in "SPP.cpp"
PS3BT PS3(&Btd,0x00,0x15,0x83,0x3D,0x0A,0x57); // Also remember to uncomment DEBUG in "PS3BT.cpp" to save space
WII Wii(&Btd); // Also uncomment DEBUG in "Wii.cpp"

// You have to pair with your Wiimote first by creating the instance like this and the press 1+2 on the Wiimote - you only have to do this once
//WII Wii(&Btd,PAIR);

void setup() {
  /* Initialize UART */
  Serial.begin(115200);
  
  /* Setup encoders */
  pinMode(leftEncoder1,INPUT);
  pinMode(leftEncoder2,INPUT);
  pinMode(rightEncoder1,INPUT);
  pinMode(rightEncoder2,INPUT);
  attachInterrupt(0,leftEncoder,RISING);
  attachInterrupt(1,rightEncoder,RISING);

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

  /* Set PWM frequency to 20kHz - see the datasheet http://www.atmel.com/Images/doc8025.pdf page 128-135 */
  // Set up PWM, Phase and Frequency Correct on pin 18 (OC1A) & pin 17 (OC1B) with ICR1 as TOP using Timer1
  TCCR1B = _BV(WGM13) | _BV(CS10); // Set PWM Phase and Frequency Correct with ICR1 as TOP and no prescaling
  ICR1H = (PWMVALUE >> 8); // ICR1 is the TOP value - this is set so the frequency is equal to 20kHz
  ICR1L = (PWMVALUE & 0xFF);

  /* Enable PWM on pin 18 (OC1A) & pin 17 (OC1B) */
  // Clear OC1A/OC1B on compare match when up-counting
  // Set OC1A/OC1B on compare match when downcountin
  TCCR1A = _BV(COM1A1) | _BV(COM1B1);
  setPWM(leftPWM,0); // Turn off pwm on both pins
  setPWM(rightPWM,0);

  /* Setup pin for buzzer to beep when finished calibrating */
  pinMode(buzzer,OUTPUT);
  
  if (Usb.Init() == -1) { // Check if USB Host is working
   Serial.print(F("OSC did not start"));
   while(1); // Halt
  }

  /* Setup IMU Inputs */
  Wire.begin();
  i2cWrite(0x1B,0x00); // Set Gyro Full Scale Range to ±250deg/s
  i2cWrite(0x1C,0x00); // Set Accelerometer Full Scale Range to ±2g
  i2cWrite(0x19,0x27); // Set the sample rate to 200Hz
  i2cWrite(0x6B,0x01); // PLL with X axis gyroscope reference and disable sleep mode

  uint8_t buf;
  while(!i2cRead(0x75,1,&buf));
  if(buf != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while(1); 
  }

  delay(100); // Wait for the sensor to get ready
  
  // Set kalman and gyro starting angle
  uint8_t data[4];
  while(!i2cRead(0x3D,4,data));
  accY = ((data[0] << 8) | data[1]);
  accZ = ((data[2] << 8) | data[3]);
  accAngle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
  
  kalman.setAngle(accAngle); // Set starting angle
  gyroAngle = accAngle;
  
  /* Beep to indicate that it is now ready */
  cbi(TCCR0B,CS00); // Set precaler to 8
  analogWrite(buzzer,128);
  delay(800); // This is really 100ms
  analogWrite(buzzer,0);
  sbi(TCCR0B,CS00); // Set precaler back to 64

  /* Setup timing */
  loopStartTime = micros();
  timer = loopStartTime;
}

void loop() {
  /* Calculate pitch */
  uint8_t data[8];
  while(!i2cRead(0x3D,8,data));
  accY = ((data[0] << 8) | data[1]);
  accZ = ((data[2] << 8) | data[3]);
  gyroX = ((data[6] << 8) | data[7]);

  accAngle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
  gyroRate = (double)gyroX/131.0;
  gyroAngle += gyroRate*((double)(micros()-timer)/1000000.0); // Gyro angle is only used for debugging

  pitch = kalman.getAngle(accAngle, gyroRate, (double)(micros()-timer)/1000000.0); // Calculate the angle using a Kalman filter
  timer = micros();
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
    PID(targetAngle,targetOffset,turningOffset);        
  }

  /* Update wheel velocity every 100ms */
  loopCounter++;
  if (loopCounter == 10) {
    loopCounter = 0; // Reset loop counter
    wheelPosition = readLeftEncoder() + readRightEncoder();
    wheelVelocity = wheelPosition - lastWheelPosition;
    lastWheelPosition = wheelPosition;
    if (abs(wheelVelocity) <= 20 && !stopped) { // Set new targetPosition if braking
      targetPosition = wheelPosition;
      stopped = true;
    }
  }

  /* Read the Bluetooth dongle */
  readBTD();    
  
  if(SerialBT.connected) {
    Usb.Task();
    if(sendPIDValues) {
      sendPIDValues = false;
      
      strcpy(stringBuf,"P,");
      SerialBT.doubleToString(Kp,convBuf);
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
      dataCounter = 1; // Set the counter to 1, to prevent it from sending data in the next loop
    } else if(sendData) {
      if(dataCounter == 0) {
          strcpy(stringBuf,"V,");
          SerialBT.doubleToString(accAngle,convBuf);
          strcat(stringBuf,convBuf);
          
          strcat(stringBuf,",");
          SerialBT.doubleToString(gyroAngle,convBuf);
          strcat(stringBuf,convBuf);
          
          strcat(stringBuf,",");
          SerialBT.doubleToString(pitch,convBuf);
          strcat(stringBuf,convBuf);
          
          SerialBT.println(stringBuf);
      }
      dataCounter++;
      if(dataCounter >= 5)
        dataCounter = 0;    
    }
  }
  
  /* Use a time fixed loop */
  lastLoopUsefulTime = micros() - loopStartTime;
  if (lastLoopUsefulTime < STD_LOOP_TIME) {
    while((micros() - loopStartTime) < STD_LOOP_TIME)
      if((micros() - loopStartTime) < (STD_LOOP_TIME-1000))
        Usb.Task();
  }
  //Serial.print(lastLoopUsefulTime);Serial.print(',');Serial.println(micros() - loopStartTime);
  loopStartTime = micros();    
}
void PID(double restAngle, double offset, double turning) {
  /* Steer robot */
  if (steerForward) {
    offset += (double)wheelVelocity/velocityScaleMove; // Scale down offset at high speed and scale up when reversing
    restAngle -= offset;
  } 
  else if (steerBackward) {
    offset -= (double)wheelVelocity/velocityScaleMove; // Scale down offset at high speed and scale up when reversing
    restAngle += offset;
  }
  /* Brake */
  else if (steerStop) {
    int32_t positionError = wheelPosition - targetPosition;
    if (abs(positionError) > zoneA) // Inside zone A
      restAngle -= (double)positionError/positionScaleA;
    else if (abs(positionError) > zoneB) // Inside zone B
      restAngle -= (double)positionError/positionScaleB;
    else // Inside zone C
      restAngle -= (double)positionError/positionScaleC;   
    restAngle -= (double)wheelVelocity/velocityScaleStop;
    if (restAngle < 160) // Limit rest Angle
      restAngle = 160;
    else if (restAngle > 200)
      restAngle = 200;
  }
  /* Update PID values */
  double error = (restAngle - pitch);
  double pTerm = Kp * error;
  iTerm += Ki * error;
  double dTerm = Kd * (error - lastError);
  lastError = error;
  double PIDValue = pTerm + iTerm + dTerm;

  /* Steer robot sideways */
  if (steerLeft) {
    turning -= abs((double)wheelVelocity/velocityScaleTurning); // Scale down at high speed
    if(turning < 0)
      turning = 0;
    PIDLeft = PIDValue-turning;
    PIDRight = PIDValue+turning;
  }
  else if (steerRight) {
    turning -= abs((double)wheelVelocity/velocityScaleTurning); // Scale down at high speed
    if(turning < 0)
      turning = 0;
    PIDLeft = PIDValue+turning;
    PIDRight = PIDValue-turning;
  }
  else {
    PIDLeft = PIDValue;
    PIDRight = PIDValue;
  }

  PIDLeft *= 0.95; // compensate for difference in the motors

  /* Set PWM Values */
  if (PIDLeft >= 0)
    moveMotor(left, forward, PIDLeft);
  else
    moveMotor(left, backward, PIDLeft * -1);
  if (PIDRight >= 0)
    moveMotor(right, forward, PIDRight);
  else
    moveMotor(right, backward, PIDRight * -1);
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
        if (input[i] == ';') // Keep reading until it reads a semicolon
          break;
        i++;
      }      
      /*Serial.print("Data: ");
      Serial.write((uint8_t*)input,i);
      Serial.println();*/
      if(input[0] == 'A') { // Abort
        stopAndReset();
        while(SerialBT.read() != 'C') // Wait until continue is send
          Usb.Task();
      } 
      
      /* Set PID and target angle */
      else if(input[0] == 'P') {
        strtok(input, ","); // Ignore 'P'
        Kp = atof(strtok(NULL, ";"));
      } else if(input[0] == 'I') {
        strtok(input, ","); // Ignore 'I'
        Ki = atof(strtok(NULL, ";"));
      } else if(input[0] == 'D') {
        strtok(input, ","); // Ignore 'D'
        Kd = atof(strtok(NULL, ";"));  
      } else if(input[0] == 'T') { // Target Angle
        strtok(input, ","); // Ignore 'T'
        targetAngle = atof(strtok(NULL, ";"));  
      } else if(input[0] == 'G') { // The processing/Android application sends when it need the current values
        if(input[1] == 'P') // PID Values
          sendPIDValues = true;
        else if(input[1] == 'B') // Begin
          sendData = true; // Send output to processing/Android application
        else if(input[1] == 'S') // Stop
          sendData = false; // Stop sending output to processing/Android application
      }
      /* Remote control */
      else if(input[0] == 'S') // Stop
        steer(stop);
      else if(input[0] == 'J') { // Joystick
        strtok(input, ","); // Ignore 'J'
        sppData1 = atof(strtok(NULL, ",")); // x-axis
        sppData2 = atof(strtok(NULL, ";")); // y-axis
        steer(joystick);
      }
      else if(input[0] == 'M') { // IMU
        strtok(input, ","); // Ignore 'M'
        sppData1 = atof(strtok(NULL, ",")); // Pitch
        sppData2 = atof(strtok(NULL, ";")); // Roll
        steer(imu);
        //SerialBT.printNumberln(sppData1);
        //SerialBT.printNumberln(sppData2);
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
    if(Xbox.XboxReceiverConnected && Xbox.Xbox360Connected[0] && !commandSent) { // We will only read from the first controller
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
    
  // Set all false
  steerForward = false;
  steerBackward = false;
  steerStop = false;
  steerLeft = false;
  steerRight = false;
  if(command == joystick) {    
    if(sppData2 > 0) {
      targetOffset = scale(sppData2,0,1,0,7);        
      steerForward = true;
    } else if(sppData2 < 0) {
      targetOffset = scale(sppData2,0,-1,0,7);
      steerBackward = true;
    } 
    if(sppData1 > 0) {
      turningOffset = scale(sppData1,0,1,0,20);        
      steerRight = true;
    } else if(sppData1 < 0) {
      turningOffset = scale(sppData1,0,-1,0,20);
      steerLeft = true;     
    }
  } else if(command == imu) {
      if(sppData2 > 0) {
        targetOffset = scale(sppData2,0,36,0,7);        
        steerForward = true;
      }     
      else if(sppData2 < 0) {
        targetOffset = scale(sppData2,0,-36,0,7);
        steerBackward = true;
      }
      if(sppData1 > 0) {
        turningOffset = scale(sppData1,0,45,0,20);        
        steerLeft = true;
      }
      else if(sppData1 < 0) {
        turningOffset = scale(sppData1,0,-45,0,20);
        steerRight = true;     
      }
  } else if(command == updatePS3) {
    if(PS3.PS3Connected) {
      if(PS3.getAnalogHat(LeftHatY) < 117 && PS3.getAnalogHat(RightHatY) < 117) {
        targetOffset = scale(PS3.getAnalogHat(LeftHatY)+PS3.getAnalogHat(RightHatY),232,0,0,7); // Scale from 232-0 to 0-7
        steerForward = true;
      } else if(PS3.getAnalogHat(LeftHatY) > 137 && PS3.getAnalogHat(RightHatY) > 137) {
        targetOffset = scale(PS3.getAnalogHat(LeftHatY)+PS3.getAnalogHat(RightHatY),276,510,0,7); // Scale from 276-510 to 0-7
        steerBackward = true;
      }
      if(((int16_t)PS3.getAnalogHat(LeftHatY) - (int16_t)PS3.getAnalogHat(RightHatY)) > 15) {
        turningOffset = scale(abs((int16_t)PS3.getAnalogHat(LeftHatY) - (int16_t)PS3.getAnalogHat(RightHatY)),0,255,0,20); // Scale from 0-255 to 0-20
        steerLeft = true;      
      } else if (((int16_t)PS3.getAnalogHat(RightHatY) - (int16_t)PS3.getAnalogHat(LeftHatY)) > 15) {   
        turningOffset = scale(abs((int16_t)PS3.getAnalogHat(LeftHatY) - (int16_t)PS3.getAnalogHat(RightHatY)),0,255,0,20); // Scale from 0-255 to 0-20  
        steerRight = true;  
      }  
    } else { // It must be a Navigation controller then
      if(PS3.getAnalogHat(LeftHatY) < 117) {
        targetOffset = scale(PS3.getAnalogHat(LeftHatY),116,0,0,7); // Scale from 116-0 to 0-7
        steerForward = true;
      } else if(PS3.getAnalogHat(LeftHatY) > 137) {
        targetOffset = scale(PS3.getAnalogHat(LeftHatY),138,255,0,7); // Scale from 138-255 to 0-7
        steerBackward = true;
      }
      if(PS3.getAnalogHat(LeftHatX) < 55) {
        turningOffset = scale(PS3.getAnalogHat(LeftHatX),54,0,0,20); // Scale from 54-0 to 0-20
        steerLeft = true;     
      } else if(PS3.getAnalogHat(LeftHatX) > 200) {
        turningOffset = scale(PS3.getAnalogHat(LeftHatX),201,255,0,20); // Scale from 201-255 to 0-20
        steerRight = true;
      }
    }
  } 
  else if(command == updateWii) {
    if(!Wii.wiiUProControllerConnected) {
      if(Wii.getButtonPress(B)) {
        if(Wii.getPitch() > 180) {
          targetOffset = scale(Wii.getPitch(),180,216,0,7);        
          steerForward = true;
        }     
        else if(Wii.getPitch() < 180) {
          targetOffset = scale(Wii.getPitch(),180,144,0,7);
          steerBackward = true;
        }
        if(Wii.getRoll() > 180) {
          turningOffset = scale(Wii.getRoll(),180,225,0,20);        
          steerRight = true;
        }
        else if(Wii.getRoll() < 180) {
          turningOffset = scale(Wii.getRoll(),180,135,0,20);
          steerLeft = true;     
        }
      }
      else {
        if(Wii.getAnalogHat(HatY) > 137) {
          targetOffset = scale(Wii.getAnalogHat(HatY),138,230,0,7);
          steerForward = true;
        } 
        else if(Wii.getAnalogHat(HatY) < 117) {
          targetOffset = scale(Wii.getAnalogHat(HatY),116,25,0,7);
          steerBackward = true;
        }
        if(Wii.getAnalogHat(HatX) > 137) {
          turningOffset = scale(Wii.getAnalogHat(HatX),138,230,0,20);
          steerRight = true;     
        } 
        else if(Wii.getAnalogHat(HatX) < 117) {
          turningOffset = scale(Wii.getAnalogHat(HatX),116,25,0,20);
          steerLeft = true;
        }
      }
    } else { // It must be a Wii U Pro Controller then
      if(Wii.getAnalogHat(LeftHatY) > 2200 && Wii.getAnalogHat(RightHatY) > 2200) {
        targetOffset = scale(Wii.getAnalogHat(LeftHatY)+Wii.getAnalogHat(RightHatY),4402,6400,0,7); // Scale from 4402-6400 to 0-7
        steerForward = true;
      } else if(Wii.getAnalogHat(LeftHatY) < 1800 && Wii.getAnalogHat(RightHatY) < 1800) {
        targetOffset = scale(Wii.getAnalogHat(LeftHatY)+Wii.getAnalogHat(RightHatY),3598,1600,0,7); // Scale from 3598-1600 to 0-7
        steerBackward = true;
      }
      if (((int32_t)Wii.getAnalogHat(RightHatY) - (int32_t)Wii.getAnalogHat(LeftHatY)) > 200) {
        turningOffset = scale(abs((int32_t)Wii.getAnalogHat(LeftHatY) - (int32_t)Wii.getAnalogHat(RightHatY)),0,2400,0,20); // Scale from 0-2400 to 0-20
        steerLeft = true;
      } else if(((int32_t)Wii.getAnalogHat(LeftHatY) - (int32_t)Wii.getAnalogHat(RightHatY)) > 200) {
        turningOffset = scale(abs((int32_t)Wii.getAnalogHat(LeftHatY) - (int32_t)Wii.getAnalogHat(RightHatY)),0,2400,0,20); // Scale from 0-2400 to 0-20
        steerRight = true;
      }   
    }
  }
  else if(command == updateXbox) {
    if(Xbox.getAnalogHat(0,LeftHatY) < -7500 && Xbox.getAnalogHat(0,RightHatY) < -7500) {
      targetOffset = scale(Xbox.getAnalogHat(0,LeftHatY)+Xbox.getAnalogHat(0,RightHatY),-7500,-32768,0,7); // Scale from -7500 to -32768 to 0-7
      steerForward = true;
    } else if(Xbox.getAnalogHat(0,LeftHatY) > 7500 && Xbox.getAnalogHat(0,RightHatY) > 7500) {
      targetOffset = scale(Xbox.getAnalogHat(0,LeftHatY)+Xbox.getAnalogHat(0,RightHatY),7500,32767,0,7); // Scale from 7500-32767 to 0-7
      steerBackward = true;
    }
    if(((int64_t)Xbox.getAnalogHat(0,LeftHatY) - (int64_t)Xbox.getAnalogHat(0,RightHatY)) > 7500) {
      turningOffset = scale(abs((int64_t)Xbox.getAnalogHat(0,LeftHatY) - (int64_t)Xbox.getAnalogHat(0,RightHatY)),0,65535,0,20); // Scale from 0-65535 to 0-20
       steerLeft = true;      
    } else if (((int64_t)Xbox.getAnalogHat(0,RightHatY) - (int64_t)Xbox.getAnalogHat(0,LeftHatY)) > 7500) {   
      turningOffset = scale(abs((int64_t)Xbox.getAnalogHat(0,LeftHatY) - (int64_t)Xbox.getAnalogHat(0,RightHatY)),0,65535,0,20); // Scale from 0-65535 to 0-20  
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
void stopAndReset() {
  stopMotor(left);
  stopMotor(right);  
  lastError = 0;
  iTerm = 0;
  targetPosition = wheelPosition;
}
void i2cWrite(uint8_t registerAddress, uint8_t data) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data);
  Wire.endTransmission(); // Send stop
}
uint8_t i2cRead(uint8_t registerAddress, uint8_t nbytes, uint8_t * data) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.endTransmission(false); // Don't release the bus
  Wire.requestFrom(IMUAddress, nbytes); // Send a repeated start and then release the bus after reading
  for(uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while(((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else
        return 0; // Error in communication
    }
  }
  return 1;
}

void moveMotor(Command motor, Command direction, double speedRaw) { // Speed is a value in percentage 0-100%
  if(speedRaw > 100)
    speedRaw = 100;
  uint16_t speed = speedRaw*((double)PWMVALUE)/100.0; // Scale from 100 to PWMVALUE
  if (motor == left) {
    setPWM(leftPWM,speed); // Left motor pwm
    if (direction == forward) {
      cbi(leftPort,leftA);
      sbi(leftPort,leftB);
    } 
    else if (direction == backward) {
      sbi(leftPort,leftA);
      cbi(leftPort,leftB);
    }
  } 
  else if (motor == right) {
    setPWM(rightPWM,speed); // Right motor pwm
    if (direction == forward) {
      sbi(rightPort,rightA);
      cbi(rightPort,rightB);
    } 
    else if (direction == backward) {
      cbi(rightPort,rightA);
      sbi(rightPort,rightB);
    }
  }
}
void stopMotor(Command motor) {  
  if (motor == left) {
    setPWM(leftPWM,PWMVALUE); // Set high
    sbi(leftPort,leftA);
    sbi(leftPort,leftB);
  } 
  else if (motor == right) {
    setPWM(rightPWM,PWMVALUE); // Set high
    sbi(rightPort,rightA);
    sbi(rightPort,rightB);
  }
}
void setPWM(uint8_t pin, uint16_t dutyCycle) { // dutyCycle is a value between 0-ICR
  if(pin == leftPWM) {
    OCR1AH = (dutyCycle >> 8); 
    OCR1AL = (dutyCycle & 0xFF);
  } else if (pin == rightPWM) {
    OCR1BH = (dutyCycle >> 8);
    OCR1BL = (dutyCycle & 0xFF);    
  }
}

/* Interrupt routine and encoder read functions - I read using the port registers for faster processing */
void leftEncoder() { 
  if(PINA & _BV(PINA6))
    leftCounter--;
  else
    leftCounter++;
}
void rightEncoder() {
  if(PINA & _BV(PINA7))
    rightCounter++;
  else
    rightCounter--;
}
int32_t readLeftEncoder() { // The encoders decrease when motors are traveling forward and increase when traveling backward
  return leftCounter;
}
int32_t readRightEncoder() {
  return rightCounter;
}
