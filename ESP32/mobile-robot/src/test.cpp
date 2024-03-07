/* void controlStraightLine(){
  if(distance > 0 && distance >=targetDistance){
    stop();
    control_needed = false;
    activateLineController = false;
    currentCommand.state = CommandState::COMPLETED;
  }
  if (abs(targetAngle - heading) < 3){
    // Indicates that the robot has been aligned for a long enough duration
    if (countStraight < 10){
      countStraight ++;
    } 
    else {
      countStraight = 0;
      adjustedSpeed = AMotorSpeed; 
    }
  } 
  else {
    countStraight = 0;
  } 

  int deltaAngle = round(targetAngle - heading);
  // Target angle change is used to achieve desired motor speed and achieve the target direction.
  // Angle increases as the robot turns right (rotates right). GyroX also increases (has positive sign).
  // If robot is turning left gyroX increases and has a negative sign.
  int targetGyroX;
  
  // If the angle offset is very large than than set the targetGyroX (desired angle change to 30)
  // Proportional control
  if (deltaAngle > 30){
      targetGyroX = 30;
  } 
  else if (deltaAngle < -30){
    targetGyroX = -30;
  } 
  // For smaller delta angles, we need smaller rotation speed
  else {
    targetGyroX = 1 * deltaAngle;
  }
  // Motor A is stronger than motor B. Speed of motor B will remain the same. Speed of motor A will be adjusted accordingly.
  // If the targetGyroX is greater than the current reading, the motor speed (AMotorSpeed) is adjusted by calling the 
  // changeSpeed function with a positive increment (+1). This will increase the motor speed to rotate towards the target angle.
  if (round(targetGyroX - gyroX_degs) != 0) {
    AMotorSpeed = (targetGyroX > gyroX_degs) ? changeSpeed(AMotorSpeed, +1) : changeSpeed(AMotorSpeed, -1);
  }
} */