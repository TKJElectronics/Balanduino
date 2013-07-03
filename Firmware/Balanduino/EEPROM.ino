#include "EEPROMAnything.h"

cfg_t cfg;

void checkInitializationFlags() {
  char initFlags[3];
  EEPROM_readAnything(initFlagsAddr,initFlags);
  if (initFlags[0] != 'T' || initFlags[1] != 'K' || initFlags[2] != 'J') {
    restoreEEPROMValues();
    EEPROM_updateAnything(initFlagsAddr,"TKJ"); // After the default values have been restored, set the flags
  }
}
  
void readEEPROMValues() {
  EEPROM_readAnything(configAddr,cfg);

  kalman.setQangle(cfg.Qangle);
  kalman.setQbias(cfg.Qbias);
  kalman.setRmeasure(cfg.Rmeasure);
}

void updateConfig() {
  EEPROM_updateAnything(configAddr,cfg);

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