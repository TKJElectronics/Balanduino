#include "EEPROMAnything.h"

cfg_t cfg; //  Struct for all the configuration values

void checkInitializationFlags() {
  char initFlag;
  for (uint8_t i = 0; i < strlen(eepromVersion); i++) {
    EEPROM_readAnything(initFlagsAddr+i, initFlag);
    if (initFlag != eepromVersion[i]) { // Check if the EEPROM version matches the current one
    restoreEEPROMValues();
    for (uint8_t i = 0; i < strlen(eepromVersion); i++)
      EEPROM_updateAnything(initFlagsAddr+i, eepromVersion[i]); // After the default values have been restored, set the flags
      break;
    }
  }
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
  cfg.P = 10;
  cfg.I = 2;
  cfg.D = 3;

  cfg.targetAngle = 180;
  cfg.backToSpot = 1;
  cfg.controlAngleLimit = 8;
  cfg.turningLimit = 20;

  cfg.Qangle = 0.001;
  cfg.Qbias = 0.003;
  cfg.Rmeasure = 0.03;

  updateConfig();
}
