void PID(double restAngle, double offset, double turning, double dt) {
  /* Steer robot */
  if (steerForward) {
    if (wheelVelocity < 0)
      offset += (double)wheelVelocity/velocityScaleMove; // Scale down offset at high speed - wheel velocity is negative when driving forward
    restAngle -= offset;
  }
  else if (steerBackward) {
    if (wheelVelocity > 0)
      offset -= (double)wheelVelocity/velocityScaleMove; // Scale down offset at high speed - wheel velocity is positive when driving backward
    restAngle += offset;
  }
  /* Brake */
  else if (steerStop) {
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
  }
  
  if (restAngle - lastRestAngle > 1) // Don't change restAngle with more than 1 degree in each loop
    restAngle = lastRestAngle+1;
  else if (restAngle - lastRestAngle < -1)
    restAngle = lastRestAngle-1;
  lastRestAngle = restAngle;
  
  /* Update PID values */
  error = (restAngle - pitch);
  pTerm = cfg.P * error;
  integratedError += error*dt;
  integratedError = constrain(integratedError, -1.0, 1.0); // Limit the integrated error
  iTerm = (cfg.I*100.0) * integratedError;
  dTerm = (cfg.D/100.0) * (error - lastError)/dt;
  lastError = error;
  PIDValue = pTerm + iTerm + dTerm;
  
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
