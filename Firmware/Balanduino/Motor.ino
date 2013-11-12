void updatePID(double angle, double offset, double turning, double dt) {
  //restAngle = angle;
  /* Steer robot */
  if (steerForward) {
    if (wheelVelocity < 0)
      offset += (double)wheelVelocity/velocityScaleMove; // Scale down offset at high speed - wheel velocity is negative when driving forward
    restAngle = angle - offset;
  }
  else if (steerBackward) {
    if (wheelVelocity > 0)
      offset -= (double)wheelVelocity/velocityScaleMove; // Scale down offset at high speed - wheel velocity is positive when driving backward
    restAngle = angle + offset;
  }
  /* Brake */
  else if (steerStop) {
    int32_t wheelPosition = getWheelsPosition();
    // The input is 'wheelPosition'
    // The setpoint is 'targetPosition'
    // The output is stored in 'restAngle'
    encoders_pid.Compute();
    restAngle += cfg.targetAngle;

    /*
    int32_t wheelPosition = getWheelsPosition();
    int32_t positionError = wheelPosition - targetPosition;
    if (cfg.backToSpot) {
      if (abs(positionError) > zoneA) // Inside zone A
        restAngle -= (double)positionError/positionScaleA;
      else if (abs(positionError) > zoneB) // Inside zone B
        restAngle -= (double)positionError/positionScaleB;
      else if (abs(positionError) > zoneC) // Inside zone C
        restAngle -= (double)positionError/positionScaleC;
      else // Inside zone D
        restAngle -= (double)positionError/positionScaleD;
    } else {
      if (abs(positionError) < zoneC)
        restAngle -= (double)positionError/positionScaleD;
      else
        targetPosition = wheelPosition;
    }
    restAngle -= (double)wheelVelocity/velocityScaleStop;

    if (restAngle < cfg.targetAngle-10) // Limit rest Angle
      restAngle = cfg.targetAngle-10;
    else if (restAngle > cfg.targetAngle+10)
      restAngle = cfg.targetAngle+10;
    */
  }

  if (restAngle - lastRestAngle > 1) // Don't change restAngle with more than 1 degree in each loop
    restAngle = lastRestAngle+1;
  else if (restAngle - lastRestAngle < -1)
    restAngle = lastRestAngle-1;
  lastRestAngle = restAngle;

  /* Update PID value */
  // The input is 'pitch'
  // The setpoint is 'restAngle'
  // The output is stored in 'PIDValue'
  main_pid.Compute();

  /* Steer robot sideways */
  if (steerLeft) {
    turning -= abs((double)wheelVelocity/velocityScaleTurning); // Scale down at high speed
    if (turning < 0)
      turning = 0;
    PIDLeft = PIDValue-turning;
    PIDRight = PIDValue+turning;
  }
  else if (steerRight) {
    turning -= abs((double)wheelVelocity/velocityScaleTurning); // Scale down at high speed
    if (turning < 0)
      turning = 0;
    PIDLeft = PIDValue+turning;
    PIDRight = PIDValue-turning;
  }
  else {
    PIDLeft = PIDValue;
    PIDRight = PIDValue;
  }

  PIDLeft *= cfg.leftMotorScaler; // Compensate for difference in some of the motors
  PIDRight *= cfg.rightMotorScaler;

  /* Set PWM Values */
  if (PIDLeft >= 0)
    moveMotor(left, forward, PIDLeft);
  else
    moveMotor(left, backward, -PIDLeft);
  if (PIDRight >= 0)
    moveMotor(right, forward, PIDRight);
  else
    moveMotor(right, backward, -PIDRight);
}

void moveMotor(Command motor, Command direction, double speedRaw) { // Speed is a value in percentage 0-100%
  if (speedRaw > 100)
    speedRaw = 100.0;
  uint16_t speed = speedRaw*((double)PWMVALUE)/100.0; // Scale from 0-100 to 0-PWMVALUE
  if (motor == left) {
    setPWM(leftPWM, speed); // Left motor PWM
    if (direction == forward) {
      cbi(leftPort, leftA);
      sbi(leftPort, leftB);
    }
    else if (direction == backward) {
      sbi(leftPort, leftA);
      cbi(leftPort, leftB);
    }
  }
  else if (motor == right) {
    setPWM(rightPWM, speed); // Right motor PWM
    if (direction == forward) {
      sbi(rightPort, rightA);
      cbi(rightPort, rightB);
    }
    else if (direction == backward) {
      cbi(rightPort, rightA);
      sbi(rightPort, rightB);
    }
  }
}
void stopMotor(Command motor) {
  if (motor == left) {
    setPWM(leftPWM, PWMVALUE); // Set high
    sbi(leftPort, leftA);
    sbi(leftPort, leftB);
  }
  else if (motor == right) {
    setPWM(rightPWM, PWMVALUE); // Set high
    sbi(rightPort, rightA);
    sbi(rightPort, rightB);
  }
}
void setPWM(uint8_t pin, uint16_t dutyCycle) { // dutyCycle is a value between 0-ICR
  if (pin == leftPWM)
    OCR1A = dutyCycle;
  else if (pin == rightPWM)
    OCR1B = dutyCycle;
}
void stopAndReset() {
  stopMotor(left);
  stopMotor(right);
  targetPosition = getWheelsPosition();
  lastRestAngle = cfg.targetAngle;
}

/* Interrupt routine and encoder read functions - we read using the port registers for faster processing */
void leftEncoder() {
  if ((bool)(leftEncoder1Port & leftEncoder1Mask) == (bool)(leftEncoder2Port & leftEncoder2Mask)) // Compare pin 15 and pin 30
    leftCounter--;
  else
    leftCounter++;
}
void rightEncoder() {
  if ((bool)(rightEncoder1Port & rightEncoder1Mask) == (bool)(rightEncoder2Port & rightEncoder2Mask)) // Compare pin 16 and pin 31
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
int32_t getWheelsPosition() {
  return leftCounter + rightCounter;
}
