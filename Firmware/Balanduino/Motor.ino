void updatePID(double restAngle, double offset, double turning, double dt) {
  /* Steer robot */
  if (offset > 0) {
    if (wheelVelocity < 0)
      offset += (double)wheelVelocity / velocityScaleMove; // Scale down offset at high speed - wheel velocity is negative when driving forward
    restAngle -= offset;
  }
  else if (offset < 0) {
    if (wheelVelocity > 0)
      offset += (double)wheelVelocity / velocityScaleMove; // Scale down offset at high speed - wheel velocity is positive when driving backward
    restAngle -= offset;
  }
  /* Brake */
  else if (steerStop) {
    int32_t wheelPosition = getWheelsPosition();
    int32_t positionError = wheelPosition - targetPosition;
    if (cfg.backToSpot) {
      if (abs(positionError) > zoneA) // Inside zone A
        restAngle -= (double)positionError / positionScaleA;
      else if (abs(positionError) > zoneB) // Inside zone B
        restAngle -= (double)positionError / positionScaleB;
      else if (abs(positionError) > zoneC) // Inside zone C
        restAngle -= (double)positionError / positionScaleC;
      else // Inside zone D
        restAngle -= (double)positionError / positionScaleD;
    } else {
      if (abs(positionError) < zoneC)
        restAngle -= (double)positionError / positionScaleD;
      else
        targetPosition = wheelPosition;
    }
    restAngle -= (double)wheelVelocity / velocityScaleStop;

    if (restAngle < cfg.targetAngle - 10) // Limit rest Angle
      restAngle = cfg.targetAngle - 10;
    else if (restAngle > cfg.targetAngle + 10)
      restAngle = cfg.targetAngle + 10;
  }

  if (restAngle - lastRestAngle > 1) // Don't change restAngle with more than 1 degree in each loop
    restAngle = lastRestAngle + 1;
  else if (restAngle - lastRestAngle < -1)
    restAngle = lastRestAngle - 1;
  lastRestAngle = restAngle;

  /* Update PID values */
  error = (restAngle - pitch);
  pTerm = cfg.P * error;
  integratedError += error * dt;
  integratedError = constrain(integratedError, -1.0, 1.0); // Limit the integrated error
  iTerm = (cfg.I * 100.0) * integratedError;
  dTerm = (cfg.D / 100.0) * (error - lastError) / dt;
  lastError = error;
  PIDValue = pTerm + iTerm + dTerm;

  /* Steer robot sideways */
  if (turning < 0) {
    turning += abs((double)wheelVelocity / velocityScaleTurning); // Scale down at high speed
    if (turning > 0)
      turning = 0;
    PIDLeft = PIDValue + turning;
    PIDRight = PIDValue - turning;
  }
  else if (turning > 0) {
    turning -= abs((double)wheelVelocity / velocityScaleTurning); // Scale down at high speed
    if (turning < 0)
      turning = 0;
    PIDLeft = PIDValue + turning;
    PIDRight = PIDValue - turning;
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
  uint16_t speed = speedRaw * ((double)PWMVALUE) / 100.0; // Scale from 0-100 to 0-PWMVALUE
  if (motor == left) {
    setPWM(leftPWM::Number, speed); // Left motor PWM
    if (direction == forward) {
      leftA::Clear();
      leftB::Set();
    }
    else if (direction == backward) {
      leftA::Set();
      leftB::Clear();
    }
  }
  else if (motor == right) {
    setPWM(rightPWM::Number, speed); // Right motor PWM
    if (direction == forward) {
      rightA::Set();
      rightB::Clear();
    }
    else if (direction == backward) {
      rightA::Clear();
      rightB::Set();
    }
  }
}
void stopMotor(Command motor) {
  if (motor == left) {
    setPWM(leftPWM::Number, PWMVALUE); // Set high
    leftA::Set();
    leftB::Set();
  }
  else if (motor == right) {
    setPWM(rightPWM::Number, PWMVALUE); // Set high
    rightA::Set();
    rightB::Set();
  }
}
void setPWM(uint8_t pin, uint16_t dutyCycle) { // dutyCycle is a value between 0-ICR1
  if (pin == leftPWM::Number)
    OCR1A = dutyCycle;
  else if (pin == rightPWM::Number)
    OCR1B = dutyCycle;
}
void stopAndReset() {
  stopMotor(left);
  stopMotor(right);
  lastError = 0;
  integratedError = 0;
  targetPosition = getWheelsPosition();
  lastRestAngle = cfg.targetAngle;
}

/* Interrupt routine and encoder read functions - we read using the port registers for faster processing */
void leftEncoder() {
  if ((bool)leftEncoder1::IsSet() == (bool)leftEncoder2::IsSet()) // Compare pin 15 and pin 30
    leftCounter--;
  else
    leftCounter++;
}
void rightEncoder() {
  if ((bool)rightEncoder1::IsSet() == (bool)rightEncoder2::IsSet()) // Compare pin 16 and pin 31
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
