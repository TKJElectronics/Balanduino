double oldKp, oldKi, oldKd, oldTargetAngle;

void readEEPROMValues() {
  EEPROM_readAnything(KpAddr,Kp);
  EEPROM_readAnything(KiAddr,Ki);
  EEPROM_readAnything(KdAddr,Kd);
  EEPROM_readAnything(targetAngleAddr,targetAngle);
  
  oldKp = Kp;
  oldKi = Ki;
  oldKd = Kd;
  oldTargetAngle = targetAngle;
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
  
  oldKp = Kp;
  oldKi = Ki;
  oldKd = Kd;
  oldTargetAngle = targetAngle;
}
