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

#include "EEPROMAnything.h"

cfg_t cfg; //  Struct for all the configuration values

bool checkInitializationFlags() {
  uint8_t initFlag;
  EEPROM_readAnything(initFlagsAddr, initFlag);
  if (initFlag != eepromVersion) { // Check if the EEPROM version matches the current one
    restoreEEPROMValues();
    EEPROM_updateAnything(initFlagsAddr, eepromVersion); // After the default values have been restored, set the flag
    return true; // Indicate that the values have been restored
  }
  return false;
}

void readEEPROMValues() {
  EEPROM_readAnything(configAddr, cfg);

  kalman.setQangle(cfg.Qangle);
  kalman.setQbias(cfg.Qbias);
  kalman.setRmeasure(cfg.Rmeasure);
}

void updateConfig() {
  EEPROM_updateAnything(configAddr, cfg);

  kalman.setQangle(cfg.Qangle);
  kalman.setQbias(cfg.Qbias);
  kalman.setRmeasure(cfg.Rmeasure);
}

void restoreEEPROMValues() {
  cfg.P = 9.0;
  cfg.I = 2.0;
  cfg.D = 3.0;

  cfg.targetAngle = 180.0;
  cfg.backToSpot = 1;
  cfg.controlAngleLimit = 7;
  cfg.turningLimit = 25;

  cfg.Qangle = 0.001;
  cfg.Qbias = 0.003;
  cfg.Rmeasure = 0.03;

  cfg.accYzero = cfg.accZzero = 0;
  cfg.leftMotorScaler = cfg.rightMotorScaler = 1;

  cfg.bindSpektrum = false;

  updateConfig();
}
