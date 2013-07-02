cfg_t cfg;

void checkInitializationFlags() {
  char initFlags[3];
  for (uint8_t i = 0; i < 3; i++) {
    eeprom_busy_wait(); // Wait until the eeprom is ready
    initFlags[i] = eeprom_read_byte((uint8_t*)(initFlagsAddr+i));
  }
    
  if (initFlags[0] != 'T' || initFlags[1] != 'K' || initFlags[2] != 'J') {
    restoreEEPROMValues();
    setInitializationFlags();
  }
}    
  
void setInitializationFlags() {
  eeprom_busy_wait(); // Wait until the eeprom is ready
  eeprom_write_byte((uint8_t*)(initFlagsAddr+0), 'T');
  eeprom_busy_wait(); // Wait until the eeprom is ready
  eeprom_write_byte((uint8_t*)(initFlagsAddr+1), 'K');
  eeprom_busy_wait(); // Wait until the eeprom is ready
  eeprom_write_byte((uint8_t*)(initFlagsAddr+2), 'J');
}
  
void readEEPROMValues() {
  EEPROM_readAnything(configAddr,cfg);
}

void updateConfig() {
  EEPROM_updateAnything(configAddr,cfg);
}

void restoreEEPROMValues() {
  cfg.P = 10;
  cfg.I = 2;
  cfg.D = 3;
  cfg.targetAngle = 180;
  cfg.backToSpot = 1;
  cfg.controlAngleLimit = 8;
  cfg.turningLimit = 20;

  void updateConfig();
}