double oldKp, oldKi, oldKd, oldTargetAngle;
uint8_t oldControlAngleLimit, oldTurningAngleLimit, oldBackToSpot;

void checkInitializationFlags() {
  char i;
  char initFlags[3];
  for (i = 0; i < 3; i++) {
    eeprom_busy_wait(); // Wait until the eeprom is ready
    initFlags[i] = eeprom_read_byte((uint8_t*)(InitializationFlagsAddr+i));    
  }
    
  if (initFlags[0] != 'T' || initFlags[1] != 'K' || initFlags[2] != 'J') {
    restoreEEPROMValues();
    setInitializationFlags();
  }
}    
  
void setInitializationFlags() {
  eeprom_busy_wait(); // Wait until the eeprom is ready
  eeprom_write_byte((uint8_t*)(InitializationFlagsAddr+0), 'T');
  eeprom_busy_wait(); // Wait until the eeprom is ready
  eeprom_write_byte((uint8_t*)(InitializationFlagsAddr+1), 'K');  
  eeprom_busy_wait(); // Wait until the eeprom is ready
  eeprom_write_byte((uint8_t*)(InitializationFlagsAddr+2), 'J');  
}
  
void readEEPROMValues() {
  EEPROM_readAnything(KpAddr,Kp);
  EEPROM_readAnything(KiAddr,Ki);
  EEPROM_readAnything(KdAddr,Kd);
  EEPROM_readAnything(targetAngleAddr,targetAngle);
  EEPROM_readAnything(controlAngleLimitAddr,controlAngleLimit);  
  EEPROM_readAnything(turningAngleLimitAddr,turningAngleLimit);    
  EEPROM_readAnything(BackToSpotAddr,BackToSpot);  
  
  oldKp = Kp;
  oldKi = Ki;
  oldKd = Kd;
  oldTargetAngle = targetAngle;
  oldControlAngleLimit = controlAngleLimit;
  oldTurningAngleLimit = turningAngleLimit;  
  oldBackToSpot = BackToSpot;
}

void updateKp() {
  if(Kp != oldKp)
    EEPROM_writeAnything(KpAddr,Kp);
  oldKp = Kp;
}
void updateKi() {
  if(Ki != oldKi)
    EEPROM_writeAnything(KiAddr,Ki);
  oldKi = Ki;
}
void updateKd() {
  if(Kd != oldKd)
    EEPROM_writeAnything(KdAddr,Kd);
  oldKd = Kd;
}
void updateTargetAngle() {
  if(targetAngle != oldTargetAngle)
    EEPROM_writeAnything(targetAngleAddr,targetAngle);
  oldTargetAngle = targetAngle;
}
void updateControlAngleLimit() {
  if(controlAngleLimit != oldControlAngleLimit)
    EEPROM_writeAnything(controlAngleLimitAddr,controlAngleLimit);
  oldControlAngleLimit = controlAngleLimit;
}
void updateTurningAngleLimit() {
  if(turningAngleLimit != oldTurningAngleLimit)
    EEPROM_writeAnything(turningAngleLimitAddr,turningAngleLimit);
  oldTurningAngleLimit = turningAngleLimit;
}
void updateBackToSpot() {
  if(BackToSpot != oldBackToSpot)
    EEPROM_writeAnything(BackToSpotAddr,BackToSpot);
  oldBackToSpot = BackToSpot;
}

void restoreEEPROMValues() {
  if(Kp != defaultKp) {
    Kp = defaultKp;
    EEPROM_writeAnything(KpAddr,Kp);
  }
  if(Ki != defaultKi) {
    Ki = defaultKi;
    EEPROM_writeAnything(KiAddr,Ki);
  }
  if(Kd != defaultKd) {
    Kd = defaultKd;
    EEPROM_writeAnything(KdAddr,Kd);
  }
  if(targetAngle != defaultTargetAngle) {
    targetAngle = defaultTargetAngle;
    EEPROM_writeAnything(targetAngleAddr,targetAngle);
  }
  if(controlAngleLimit != defaultControlAngleLimit) {
    controlAngleLimit = defaultControlAngleLimit;
    EEPROM_writeAnything(controlAngleLimitAddr,controlAngleLimit);
  }  
  if(turningAngleLimit != defaultTurningAngleLimit) {
    turningAngleLimit = defaultTurningAngleLimit;
    EEPROM_writeAnything(turningAngleLimitAddr,turningAngleLimit);
  }    
  if(BackToSpot != defaultBackToSpot) {
    BackToSpot = defaultBackToSpot;
    EEPROM_writeAnything(BackToSpotAddr,BackToSpot);
  }    
  
  oldKp = Kp;
  oldKi = Ki;
  oldKd = Kd;
  oldTargetAngle = targetAngle;  
  oldControlAngleLimit = controlAngleLimit;  
  oldTurningAngleLimit = turningAngleLimit;  
  oldBackToSpot = BackToSpot;  
}
