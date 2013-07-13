void moveMotor(Command motor, Command direction, double speedRaw) { // Speed is a value in percentage 0-100%
  if (speedRaw > 100)
    speedRaw = 100.0;
  uint16_t speed = speedRaw*((double)PWMVALUE)/100.0; // Scale from 0-100 to 0-PWMVALUE
  if (motor == left) {
    setPWM(leftPWM,speed); // Left motor PWM
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
    setPWM(rightPWM,speed); // Right motor PWM
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
  if (pin == leftPWM) {
    OCR1AH = (dutyCycle >> 8);
    OCR1AL = (dutyCycle & 0xFF);
  } else if (pin == rightPWM) {
    OCR1BH = (dutyCycle >> 8);
    OCR1BL = (dutyCycle & 0xFF);
  }
}
void stopAndReset() {
  stopMotor(left);
  stopMotor(right);
  lastError = 0;
  integratedError = 0;
  targetPosition = wheelPosition;
  lastRestAngle = cfg.targetAngle;
}

/* Interrupt routine and encoder read functions - I read using the port registers for faster processing */
void leftEncoder() { 
  if ((bool)(PIND & _BV(PIND2)) == (bool)(PINA & _BV(PINA6)))
    leftCounter--;
  else
    leftCounter++;
}
void rightEncoder() {
  if ((bool)(PIND & _BV(PIND3)) == (bool)(PINA & _BV(PINA7)))
    rightCounter++;
  else
    rightCounter--;
}
int32_t readLeftEncoder() { // The encoders decrease when motors are travelling forward and increase when travelling backward
  return leftCounter;
}
int32_t readRightEncoder() {
  return rightCounter;
}
