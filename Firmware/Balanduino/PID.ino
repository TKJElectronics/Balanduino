void PID(double restAngle, double offset, double turning, double dt) {
  /* Steer robot */
  if(steerForward) {
    if(wheelVelocity < 0)
      offset += (double)wheelVelocity/(velocityScaleMove*0.8); // Scale down offset at high speed - wheel velocity is negative when driving forward
    restAngle -= offset;
  }
  else if(steerBackward) {
    if(wheelVelocity > 0)
      offset -= (double)wheelVelocity/velocityScaleMove; // Scale down offset at high speed - wheel velocity is positive when driving backward
    restAngle += offset;
  }
  /* Brake */
  else if(steerStop) {
    int32_t positionError = wheelPosition - targetPosition;
    if(abs(positionError) > zoneA) // Inside zone A
      restAngle -= (double)positionError/positionScaleA;
    else if(abs(positionError) > zoneB) // Inside zone B
      restAngle -= (double)positionError/positionScaleB;
    else // Inside zone C
      restAngle -= (double)positionError/positionScaleC;
    restAngle -= (double)wheelVelocity/velocityScaleStop;
    if(restAngle < 170) // Limit rest Angle
      restAngle = 170;
    else if(restAngle > 190)
      restAngle = 190;
  }
  
  if(restAngle - lastRestAngle > 1) // Don't change restAngle with more than 1 degree in each loop
    restAngle = lastRestAngle+1;
  else if(restAngle - lastRestAngle < -1)
    restAngle = lastRestAngle-1;
  lastRestAngle = restAngle;
  
  /* Update PID values */
  double error = (restAngle - pitch);
  double pTerm = Kp * error;
  integratedError += error*dt;
  double iTerm = (Ki*100.0) * integratedError;
  double dTerm = (Kd/100.0) * (error - lastError)/dt;
  lastError = error;
  double PIDValue = pTerm + iTerm + dTerm;

  double PIDLeft;
  double PIDRight;
  
  /* Steer robot sideways */
  if(steerLeft) {
    turning -= abs((double)wheelVelocity/velocityScaleTurning); // Scale down at high speed
    if(turning < 0)
      turning = 0;
    PIDLeft = PIDValue-turning;
    PIDRight = PIDValue+turning;
  }
  else if(steerRight) {
    turning -= abs((double)wheelVelocity/velocityScaleTurning); // Scale down at high speed
    if(turning < 0)
      turning = 0;
    PIDLeft = PIDValue+turning;
    PIDRight = PIDValue-turning;
  }
  else {
    PIDLeft = PIDValue;
    PIDRight = PIDValue;
  }

  //PIDLeft *= 0.95; // compensate for difference in the motors

  /* Set PWM Values */
  if(PIDLeft >= 0)
    moveMotor(left, forward, PIDLeft);
  else
    moveMotor(left, backward, -PIDLeft);
  if(PIDRight >= 0)
    moveMotor(right, forward, PIDRight);
  else
    moveMotor(right, backward, -PIDRight);
}
