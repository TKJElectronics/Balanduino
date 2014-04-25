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

#if defined(ENABLE_USB) || defined(ENABLE_SPEKTRUM)

uint8_t ps3OldLed, wiiOldLed, ps4OldBatteryLevel;

#ifdef ENABLE_XBOX
LEDEnum xboxOldLed;
#endif

#ifdef ENABLE_SPP
void readSPPData() {
  if (SerialBT.connected) { // The SPP connection won't return data as fast as the controllers, so we will handle it separately
    if (SerialBT.available()) {
      uint8_t i = 0;
      while (1) {
        dataInput[i] = SerialBT.read();
        if (dataInput[i] == -1) // Error while reading the string
          return;
        if (dataInput[i] == ';') // Keep reading until it reads a semicolon
          break;
        if (++i >= sizeof(dataInput) / sizeof(dataInput[0])) // String is too long
          return;
      }
      bluetoothData = true;
      setValues(dataInput);
    }
  }
}
#endif // ENABLE_SPP

void readUsb() {
#ifdef ENABLE_USB
  Usb.Task(); // The SPP data is actually not send until this is called, one could call SerialBT.send() directly as well

  if (Usb.getUsbTaskState() == USB_STATE_ERROR && layingDown) { // Check if the USB state machine is in an error state, but also make sure the robot is laying down
    Serial.println(F("USB fail"));
    Usb.vbusPower(vbus_off);
    delay(1000);
    Usb.vbusPower(vbus_on);
    Usb.setUsbTaskState(USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE); // Reset state machine
  }
#endif // ENABLE_USB

#ifdef ENABLE_SPP
  readSPPData();
#endif // ENABLE_SPP

  if (millis() > (receiveControlTimer + receiveControlTimeout)) {
    commandSent = false; // We use this to detect when there has already been sent a command by one of the controllers
#ifdef ENABLE_PS3
    if (PS3.PS3Connected) {
      if (PS3.getButtonPress(SELECT)) {
        stopAndReset();
        while (!PS3.getButtonPress(START))
          Usb.Task();
      }
      else if (PS3.getButtonPress(CROSS) || (PS3.getAnalogHat(LeftHatY) < 117 || PS3.getAnalogHat(RightHatY) < 117 || PS3.getAnalogHat(LeftHatY) > 137 || PS3.getAnalogHat(RightHatY) > 137))
        steer(updatePS3);
    } else if (PS3.PS3NavigationConnected && (PS3.getAnalogHat(LeftHatX) > 200 || PS3.getAnalogHat(LeftHatX) < 55 || PS3.getAnalogHat(LeftHatY) > 137 || PS3.getAnalogHat(LeftHatY) < 117))
      steer(updatePS3);
#endif // ENABLE_PS3
#ifdef ENABLE_PS4
    if (PS4.connected() && !commandSent) {
      if (PS4.getButtonPress(SHARE)) {
        stopAndReset();
        while (!PS4.getButtonPress(OPTIONS))
          Usb.Task();
      }
      else if (PS4.getButtonPress(CROSS) || PS4.getAnalogHat(LeftHatY) < 117 || PS4.getAnalogHat(RightHatY) < 117 || PS4.getAnalogHat(LeftHatY) > 137 || PS4.getAnalogHat(RightHatY) > 137)
        steer(updatePS4);
    }
#endif // ENABLE_PS4
#ifdef ENABLE_WII
    if (Wii.wiimoteConnected && !Wii.wiiUProControllerConnected && !commandSent) {
      if (Wii.getButtonPress(B) || (Wii.nunchuckConnected && (Wii.getAnalogHat(HatX) > 137 || Wii.getAnalogHat(HatX) < 117 || Wii.getAnalogHat(HatY) > 137 || Wii.getAnalogHat(HatY) < 117)))
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
      else if (Xbox.getAnalogHat(LeftHatY) < -7500 || Xbox.getAnalogHat(RightHatY) < -7500 || Xbox.getAnalogHat(LeftHatY) > 7500 || Xbox.getAnalogHat(RightHatY) > 7500)
        steer(updateXbox);
    }
#endif // ENABLE_XBOX
#ifdef ENABLE_SPEKTRUM
    if (spekConnected) {
      if (millis() - spekConnectedTimer > 100) // If it has been more than 100ms since last data, then it must be disconnected
        spekConnected = false;
      else {
        if (!commandSent && (rcValue[RC_CHAN_ROLL] < 1490 || rcValue[RC_CHAN_ROLL] > 1510 || rcValue[RC_CHAN_PITCH] < 1490 || rcValue[RC_CHAN_PITCH] > 1510))
          steer(updateSpektrum);
      }
    }
#endif // ENABLE_SPEKTRUM
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
#endif // ENABLE_PS3
#ifdef ENABLE_PS4
  if (PS4.connected()) {
    if (PS4.getButtonClick(PS)) {
      PS4.disconnect();
      ps4OldBatteryLevel = 0; // Reset value
    }
  }
#endif // ENABLE_PS4
#ifdef ENABLE_WII
  if (Wii.wiimoteConnected || Wii.wiiUProControllerConnected) {
    if (Wii.getButtonClick(HOME)) {
      Wii.disconnect();
      wiiOldLed = 0; // Reset value
    }
  }
#endif // ENABLE_WII
#ifdef ENABLE_XBOX
  if (Xbox.Xbox360Connected[0]) {
    if (Xbox.getButtonClick(XBOX)) {
      Xbox.disconnect();
      xboxOldLed = OFF; // Reset value
    }
  }
#endif // ENABLE_XBOX

#if defined(ENABLE_PS3) || defined(ENABLE_PS4) || defined(ENABLE_WII) || defined(ENABLE_XBOX)
  if (millis() - ledTimer > 1000) { // Update every 1s
    ledTimer = millis();
    updateLEDs();
  }
#endif // defined(ENABLE_PS3) || defined(ENABLE_PS4) || defined(ENABLE_WII) || defined(ENABLE_XBOX)
}

#if defined(ENABLE_PS3) || defined(ENABLE_PS4) || defined(ENABLE_WII) || defined(ENABLE_XBOX)
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
      ps3OldLed = 0; // Reset value
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
#ifdef ENABLE_PS4
  if (PS4.connected()) {
    if (ps4RumbleEnabled) {
      ps4RumbleEnabled = false;
      PS4.setRumbleOff();
      ps4OldBatteryLevel = 0; // Reset value
    } else {
      uint8_t ps4BatteryLevel = PS4.getBatteryLevel();
      if (ps4BatteryLevel != ps4OldBatteryLevel) {
        ps4OldBatteryLevel = ps4BatteryLevel;
        PS4.setLed(0, 0, ps4BatteryLevel * 17); // The battery status is in the range from 0-15, by multiplying we get it in the range 0-255
        if (ps4BatteryLevel < 2)
          PS4.setLedFlash(10, 10); // Blink rapidly
        else
          PS4.setLedFlash(0xFF, 0); // Turn on
      }
    }
  }
#endif // ENABLE_PS4
#ifdef ENABLE_WII
  if (Wii.wiimoteConnected || Wii.wiiUProControllerConnected) {
    if (wiiRumbleEnabled) {
      wiiRumbleEnabled = false;
      Wii.setRumbleOff();
      wiiOldLed = 0; // Reset value
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
    if (xboxRumbleDisable) {
      xboxRumbleDisable = false;
      Xbox.setRumbleOff();
      xboxOldLed = OFF; // Reset value
    } else {
      uint8_t batteryLevel = Xbox.getBatteryLevel();
      LEDEnum xboxLed;
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

#ifdef ENABLE_PS3
void onInitPS3() { // This function is called when the controller is first initialized
  if (PS3.PS3Connected) { // We only check the normal PS3 controller as the Navigation controller doesn't have rumble support
    updateLEDs(); // Turn the LEDs on according to the voltage level
    ledTimer = millis() - 500; // Wait 500ms before turning rumble on
    ps3RumbleEnable = true; // We can't sent commands to the PS3 controller that often, so we don't send the command here
  }
}
#endif // ENABLE_PS3

#ifdef ENABLE_PS4
void onInitPS4() { // This function is called when the controller is first initialized
  if (PS4.connected()) { // Check if the PS4 controller is connected
    updateLEDs(); // Turn the LEDs on according to the voltage level
    ledTimer = millis() - 500; // This will turn the rumble off again after 500ms
    PS4.setRumbleOn(RumbleLow);
    ps4RumbleEnabled = true;
  }
}
#endif // ENABLE_PS4

#ifdef ENABLE_WII
void onInitWii() { // This function is called when the controller is first initialized
  if (Wii.wiimoteConnected) { // Both the Wiimote and the Wii U Pro Controller
    updateLEDs(); // Turn the LEDs on according to the voltage level
    ledTimer = millis() - 500; // This will turn the rumble off again after 500ms
    Wii.setRumbleOn();
    wiiRumbleEnabled = true;
  }
}
#endif // ENABLE_WII

#ifdef ENABLE_XBOX
void onInitXbox() { // This function is called when the controller is first initialized
  if (Xbox.Xbox360Connected[0]) { // Xbox wireless controller
    ledTimer = millis() - 500; // This will turn the rumble off again after 500ms
    Xbox.setRumbleOn(0x00, 0xFF);
    xboxRumbleDisable = true;
  }
}
#endif // ENABLE_XBOX

#endif // defined(ENABLE_PS3) || defined(ENABLE_PS4) || defined(ENABLE_WII) || defined(ENABLE_XBOX)

#endif // defined(ENABLE_USB) || defined(ENABLE_SPEKTRUM)

#if defined(ENABLE_SPP) || defined(ENABLE_PS3) || defined(ENABLE_PS4) || defined(ENABLE_WII) || defined(ENABLE_XBOX) || defined(ENABLE_TOOLS) || defined(ENABLE_SPEKTRUM)
void steer(Command command) {
  commandSent = true; // Used to see if there has already been send a command or not

  steerStop = false;
  targetOffset = turningOffset = 0; // Set both to 0

#if defined(ENABLE_SPP) || defined(ENABLE_TOOLS)
  if (command == joystick) {
    if (sppData2 > 0) // Forward
      targetOffset = scale(sppData2, 0, 1, 0, cfg.controlAngleLimit);
    else if (sppData2 < 0) // Backward
      targetOffset = -scale(sppData2, 0, -1, 0, cfg.controlAngleLimit);
    if (sppData1 > 0) // Right
      turningOffset = scale(sppData1, 0, 1, 0, cfg.turningLimit);
    else if (sppData1 < 0) // Left
      turningOffset = -scale(sppData1, 0, -1, 0, cfg.turningLimit);
  } else if (command == imu) {
    if (sppData1 > 0) // Forward
      targetOffset = scale(sppData1, 0, 36, 0, cfg.controlAngleLimit);
    else if (sppData1 < 0) // Backward
      targetOffset = -scale(sppData1, 0, -36, 0, cfg.controlAngleLimit);
    if (sppData2 > 0) // Right
      turningOffset = scale(sppData2, 0, 45, 0, cfg.turningLimit);
    else if (sppData2 < 0) // Left
      turningOffset = -scale(sppData2, 0, -45, 0, cfg.turningLimit);
  }
#endif // ENABLE_SPP or ENABLE_TOOLS
#ifdef ENABLE_PS3
  if (command == updatePS3) {
    if (PS3.PS3Connected) {
      if (PS3.getButtonPress(CROSS)) {
        if (PS3.getAngle(Pitch) > 180) // Forward
          targetOffset = scale(PS3.getAngle(Pitch), 180, 216, 0, cfg.controlAngleLimit);
        else if (PS3.getAngle(Pitch) < 180) // Backward
          targetOffset = -scale(PS3.getAngle(Pitch), 180, 144, 0, cfg.controlAngleLimit);
        if (PS3.getAngle(Roll) > 180) // Right
          turningOffset = scale(PS3.getAngle(Roll), 180, 225, 0, cfg.turningLimit);
        else if (PS3.getAngle(Roll) < 180) // Left
          turningOffset = -scale(PS3.getAngle(Roll), 180, 135, 0, cfg.turningLimit);
      } else {
        if (PS3.getAnalogHat(LeftHatY) < 117 && PS3.getAnalogHat(RightHatY) < 117) // Forward
          targetOffset = scale(PS3.getAnalogHat(LeftHatY) + PS3.getAnalogHat(RightHatY), 232, 0, 0, cfg.controlAngleLimit); // Scale from 232-0 to 0-controlAngleLimit
        else if (PS3.getAnalogHat(LeftHatY) > 137 && PS3.getAnalogHat(RightHatY) > 137) // Backward
          targetOffset = -scale(PS3.getAnalogHat(LeftHatY) + PS3.getAnalogHat(RightHatY), 276, 510, 0, cfg.controlAngleLimit); // Scale from 276-510 to 0-controlAngleLimit
        if (((int16_t)PS3.getAnalogHat(LeftHatY) - (int16_t)PS3.getAnalogHat(RightHatY)) > 15) // Left
          turningOffset = -scale(abs((int16_t)PS3.getAnalogHat(LeftHatY) - (int16_t)PS3.getAnalogHat(RightHatY)), 0, 255, 0, cfg.turningLimit); // Scale from 0-255 to 0-turningLimit
        else if (((int16_t)PS3.getAnalogHat(RightHatY) - (int16_t)PS3.getAnalogHat(LeftHatY)) > 15) // Right
          turningOffset = scale(abs((int16_t)PS3.getAnalogHat(LeftHatY) - (int16_t)PS3.getAnalogHat(RightHatY)), 0, 255, 0, cfg.turningLimit); // Scale from 0-255 to 0-turningLimit
      }
    } else { // It must be a Navigation controller then
      if (PS3.getAnalogHat(LeftHatY) < 117) // Forward
        targetOffset = scale(PS3.getAnalogHat(LeftHatY), 116, 0, 0, cfg.controlAngleLimit); // Scale from 116-0 to 0-controlAngleLimit
      else if (PS3.getAnalogHat(LeftHatY) > 137) // Backward
        targetOffset = -scale(PS3.getAnalogHat(LeftHatY), 138, 255, 0, cfg.controlAngleLimit); // Scale from 138-255 to 0-controlAngleLimit
      if (PS3.getAnalogHat(LeftHatX) < 55) // Left
        turningOffset = -scale(PS3.getAnalogHat(LeftHatX), 54, 0, 0, cfg.turningLimit); // Scale from 54-0 to 0-turningLimit
      else if (PS3.getAnalogHat(LeftHatX) > 200) // Right
        turningOffset = scale(PS3.getAnalogHat(LeftHatX), 201, 255, 0, cfg.turningLimit); // Scale from 201-255 to 0-turningLimit
    }
  }
#endif // ENABLE_PS3
#ifdef ENABLE_PS4
  if (command == updatePS4) {
    if (PS4.getButtonPress(CROSS)) {
      if (PS4.getAngle(Pitch) > 180) // Forward
        targetOffset = scale(PS4.getAngle(Pitch), 180, 216, 0, cfg.controlAngleLimit);
      else if (PS4.getAngle(Pitch) < 180) // Backward
        targetOffset = -scale(PS4.getAngle(Pitch), 180, 144, 0, cfg.controlAngleLimit);
      if (PS4.getAngle(Roll) < 180) // Right
        turningOffset = scale(PS4.getAngle(Roll), 180, 135, 0, cfg.turningLimit);
      else if (PS4.getAngle(Roll) > 180) // Left
        turningOffset = -scale(PS4.getAngle(Roll), 180, 225, 0, cfg.turningLimit);
    } else {
      if (PS4.getAnalogHat(LeftHatY) < 117 && PS4.getAnalogHat(RightHatY) < 117) // Forward
        targetOffset = scale(PS4.getAnalogHat(LeftHatY) + PS4.getAnalogHat(RightHatY), 232, 0, 0, cfg.controlAngleLimit); // Scale from 232-0 to 0-controlAngleLimit
      else if (PS4.getAnalogHat(LeftHatY) > 137 && PS4.getAnalogHat(RightHatY) > 137) // Backward
        targetOffset = -scale(PS4.getAnalogHat(LeftHatY) + PS4.getAnalogHat(RightHatY), 276, 510, 0, cfg.controlAngleLimit); // Scale from 276-510 to 0-controlAngleLimit
      if (((int16_t)PS4.getAnalogHat(LeftHatY) - (int16_t)PS4.getAnalogHat(RightHatY)) > 15) // Left
        turningOffset = -scale(abs((int16_t)PS4.getAnalogHat(LeftHatY) - (int16_t)PS4.getAnalogHat(RightHatY)), 0, 255, 0, cfg.turningLimit); // Scale from 0-255 to 0-turningLimit
      else if (((int16_t)PS4.getAnalogHat(RightHatY) - (int16_t)PS4.getAnalogHat(LeftHatY)) > 15) // Right
        turningOffset = scale(abs((int16_t)PS4.getAnalogHat(LeftHatY) - (int16_t)PS4.getAnalogHat(RightHatY)), 0, 255, 0, cfg.turningLimit); // Scale from 0-255 to 0-turningLimit
    }
  }
#endif // ENABLE_PS4
#ifdef ENABLE_WII
  if (command == updateWii) {
    if (!Wii.wiiUProControllerConnected) {
      if (Wii.getButtonPress(B)) {
        if (Wii.getPitch() > 180) // Forward
          targetOffset = scale(Wii.getPitch(), 180, 216, 0, cfg.controlAngleLimit);
        else if (Wii.getPitch() < 180) // Backward
          targetOffset = -scale(Wii.getPitch(), 180, 144, 0, cfg.controlAngleLimit);
        if (Wii.getRoll() > 180) // Right
          turningOffset = scale(Wii.getRoll(), 180, 225, 0, cfg.turningLimit);
        else if (Wii.getRoll() < 180) // Left
          turningOffset = -scale(Wii.getRoll(), 180, 135, 0, cfg.turningLimit);
      }
      else { // Read the Nunchuck controller
        if (Wii.getAnalogHat(HatY) > 137) // Forward
          targetOffset = scale(Wii.getAnalogHat(HatY), 138, 230, 0, cfg.controlAngleLimit);
        else if (Wii.getAnalogHat(HatY) < 117) // Backward
          targetOffset = -scale(Wii.getAnalogHat(HatY), 116, 25, 0, cfg.controlAngleLimit);
        if (Wii.getAnalogHat(HatX) > 137) // Right
          turningOffset = scale(Wii.getAnalogHat(HatX), 138, 230, 0, cfg.turningLimit);
        else if (Wii.getAnalogHat(HatX) < 117) // Left
          turningOffset = -scale(Wii.getAnalogHat(HatX), 116, 25, 0, cfg.turningLimit);
      }
    } else { // It must be a Wii U Pro Controller then
      if (Wii.getAnalogHat(LeftHatY) > 2200 && Wii.getAnalogHat(RightHatY) > 2200) // Forward
        targetOffset = scale(Wii.getAnalogHat(LeftHatY) + Wii.getAnalogHat(RightHatY), 4402, 6400, 0, cfg.controlAngleLimit); // Scale from 4402-6400 to 0-controlAngleLimit
      else if (Wii.getAnalogHat(LeftHatY) < 1800 && Wii.getAnalogHat(RightHatY) < 1800) // Backward
        targetOffset = -scale(Wii.getAnalogHat(LeftHatY) + Wii.getAnalogHat(RightHatY), 3598, 1600, 0, cfg.controlAngleLimit); // Scale from 3598-1600 to 0-controlAngleLimit
      if (((int16_t)Wii.getAnalogHat(RightHatY) - (int16_t)Wii.getAnalogHat(LeftHatY)) > 200) // Left
        turningOffset = -scale(abs((int16_t)Wii.getAnalogHat(LeftHatY) - (int16_t)Wii.getAnalogHat(RightHatY)), 0, 2400, 0, cfg.turningLimit); // Scale from 0-2400 to 0-turningLimit
      else if (((int16_t)Wii.getAnalogHat(LeftHatY) - (int16_t)Wii.getAnalogHat(RightHatY)) > 200) // Right
        turningOffset = scale(abs((int16_t)Wii.getAnalogHat(LeftHatY) - (int16_t)Wii.getAnalogHat(RightHatY)), 0, 2400, 0, cfg.turningLimit); // Scale from 0-2400 to 0-turningLimit
    }
  }
#endif // ENABLE_WII
#ifdef ENABLE_XBOX
  if (command == updateXbox) {
    if (Xbox.getAnalogHat(LeftHatY) > 7500 && Xbox.getAnalogHat(RightHatY) > 7500) // Forward
      targetOffset = scale((int32_t)Xbox.getAnalogHat(LeftHatY) + (int32_t)Xbox.getAnalogHat(RightHatY), 15002, 65534, 0, cfg.controlAngleLimit); // Scale from 15002-65534 to 0-controlAngleLimit
    else if (Xbox.getAnalogHat(LeftHatY) < -7500 && Xbox.getAnalogHat(RightHatY) < -7500) // Backward
      targetOffset = -scale((int32_t)Xbox.getAnalogHat(LeftHatY) + (int32_t)Xbox.getAnalogHat(RightHatY), -15002, -65536, 0, cfg.controlAngleLimit); // Scale from -15002-(-65536) to 0-controlAngleLimit
    if (((int32_t)Xbox.getAnalogHat(RightHatY) - (int32_t)Xbox.getAnalogHat(LeftHatY)) > 7500) // Left
      turningOffset = -scale(abs((int32_t)Xbox.getAnalogHat(LeftHatY) - (int32_t)Xbox.getAnalogHat(RightHatY)), 0, 65535, 0, cfg.turningLimit); // Scale from 0-65535 to 0-turningLimit
    else if (((int32_t)Xbox.getAnalogHat(LeftHatY) - (int32_t)Xbox.getAnalogHat(RightHatY)) > 7500) // Right
      turningOffset = scale(abs((int32_t)Xbox.getAnalogHat(LeftHatY) - (int32_t)Xbox.getAnalogHat(RightHatY)), 0, 65535, 0, cfg.turningLimit); // Scale from 0-65535 to 0-turningLimit
  }
#endif // ENABLE_XBOX
#ifdef ENABLE_SPEKTRUM
  if (command == updateSpektrum) {
    if (rcValue[RC_CHAN_PITCH] > 1500) // Forward
      targetOffset = scale(rcValue[RC_CHAN_PITCH], 1500, 2000, 0, cfg.controlAngleLimit);
    else if (rcValue[RC_CHAN_PITCH] < 1500) // Backward
      targetOffset = -scale(rcValue[RC_CHAN_PITCH], 1500, 1000, 0, cfg.controlAngleLimit);
    if (rcValue[RC_CHAN_ROLL] < 1500) // Left
      turningOffset = -scale(rcValue[RC_CHAN_ROLL], 1500, 1000, 0, cfg.turningLimit);
    else if (rcValue[RC_CHAN_ROLL] > 1500) // Right
      turningOffset = scale(rcValue[RC_CHAN_ROLL], 1500, 2000, 0, cfg.turningLimit);
  }
#endif // ENABLE_SPEKTRUM

  if (command == stop) {
    steerStop = true;
    if (lastCommand != stop) { // Set new stop position
      targetPosition = getWheelsPosition();
      stopped = false;
    }
  }
  lastCommand = command;
}
double scale(double input, double inputMin, double inputMax, double outputMin, double outputMax) { // Like map() just returns a double
  double output;
  if (inputMin < inputMax)
    output = (input - inputMin) / ((inputMax - inputMin) / (outputMax - outputMin));
  else
    output = (inputMin - input) / ((inputMin - inputMax) / (outputMax - outputMin));
  if (output > outputMax)
    output = outputMax;
  else if (output < outputMin)
    output = outputMin;
  return output;
}
#endif // defined(ENABLE_SPP) || defined(ENABLE_PS3) || defined(ENABLE_PS4) || defined(ENABLE_WII) || defined(ENABLE_XBOX) || defined(ENABLE_TOOLS) || defined(ENABLE_SPEKTRUM)
