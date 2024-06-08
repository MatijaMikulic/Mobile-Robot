#pragma region Includes
//################### <INCLUDE> #############################
#include <Arduino.h>
#include <BluetoothSerial.h>
#include "L298N.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "MPU6050_P.h"
//#include <MadgwickAHRS.h>
#include <esp32-hal-bt.h>
#include "CommandQueue.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
/*################### <\INCLUDE> #############################*/
#pragma endregion

#pragma region Global Constants
/*################### <GLOBAL CONSTANTS> ####################*/
// Motor A
const uint8_t enA = 15;
const uint8_t in1 = 2;
const uint8_t in2 = 4;

// Motor B
const uint8_t enB = 19;
const uint8_t in3 = 5;
const uint8_t in4 = 18;

//IR sensor A
const uint8_t ir_A = 33;
//IR sensor B
const uint8_t ir_B = 25;
//Encoder disc
const uint8_t discHoles = 20;
//Wheel radius [mm]
const float wheelRadius = 32.5f; //31
//Complementary filter
const float alpha = 0.98f;

const float factor = 0.91f;
/*################### <\GLOBAL CONSTANTS> ####################*/
#pragma endregion

#pragma region Timers
/*################### <TIMERS> ##############################*/

hw_timer_t *Timer0_Cfg = NULL;
hw_timer_t *Timer1_Cfg = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
/*################### <\TIMERS> ##############################*/
#pragma endregion

#pragma region Global Variables
/*################### <GLOBAL VARIABLES> ####################*/
MPU6050 mpu6050_1;
BluetoothSerial SerialBT;
L298N motors(enA, enB, in1, in2, in3, in4, 8);

//initial motor speeds
volatile uint16_t AMotorSpeed = 170;
volatile uint16_t BMotorSpeed = 170;
uint16_t adjustedSpeed = 150; // estimated speed at which mobile robot moves in a straight line when BMotorSpeed = 170

//madgwickfilter output
//Madgwick filter;
unsigned long microsPrevious, currentTime, previousTime;

//orientation, desired orientation ,angular speed
double heading;
double targetAngle;
float gyroX_degs;

//IR holes counter
volatile int numOfHolesA=0;
volatile int numOfHolesB=0;

//flags for activating controlStraightLine and rotate
volatile bool activateLineController = false;
volatile bool control_needed=false;
volatile bool activateRotateController = false;
volatile bool is_rotate_needed = false;

//distance travelled 
double distance = 0;
double targetDistance=0;

//For controlStraightLine and controlRotate
int countRotate = 0;
int countStraight = 0;

//For storing commands
CommandQueue queue;
Command currentCommand;

//PID controller for controlling gyroX
float kp = 0.2;
float ki = 0.05;
float kd = 0.05;
//this params need to be reset every time control algorithm starts.
float integral = 0.0f;
int previousError = 0.0f;
/*################### <\GLOBAL VARIABLES> ####################*/
#pragma endregion

#pragma region Declarations
/*################### <DECLARATIONS> ########################*/

void setOffsetsMPU6050();
void setupMPU6050();
void moveForward();
void moveBackward();
void turnRight();
void turnLeft();
void updateSpeed(uint16_t speedA, uint16_t speedB);
void stop();
void setupTimer();
void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t* param);
void ExecuteManualCommand(const char* command);
void ProcessData(const char* data, size_t length);
Command parseCommand(const char* commandString);
void executeAutoCommand();
void execute();
void MPUCalculation(void *parameter);
void runPID();
void countHolesA();
void countHolesB();
void controlStraightLine();
void controlRotate();
int  calculateAngleOffset(float target_angle, float current_angle);
bool should_turn_right(int deltaAngle);
bool should_turn_left(int deltaAngle);
void IRAM_ATTR Timer1_ISR();
void IRAM_ATTR Timer0_ISR();

/*################### <\DECLARATIONS> ########################*/
#pragma endregion

void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  Serial.begin(115200);
  
  /* Setting up MPU 6050 sensor */
  setupMPU6050();
  setOffsetsMPU6050();
  //mpu6050_1.CalibrateAccel(6);
  //mpu6050_1.CalibrateGyro(6);

  /* Setting bluetooth serial */
  SerialBT.begin("ESP32test");
  SerialBT.register_callback(callback);

  // Setting up both timers
  setupTimer();

  // Setup task for second core 
  xTaskCreatePinnedToCore(MPUCalculation, "MPUCalculation", 4096, NULL, 1, NULL, 0);

  currentTime = micros();

  // Attaching interrupts for encoders
  pinMode(ir_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ir_A), countHolesA, RISING);  
  pinMode(ir_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ir_B), countHolesB, RISING);

  currentCommand.type = CommandType::NONE;
  currentCommand.angle = -1;
  currentCommand.distance = -1;
  currentCommand.state = CommandState::COMPLETED;   
}

void loop() {
  //pooling implementation
  if (control_needed) {
    controlStraightLine();
    moveForward();
    ledcWrite(0, AMotorSpeed);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(1, BMotorSpeed);
    digitalWrite(in3,HIGH);
    digitalWrite(in4, LOW);
    
    // stop the robot if robot has travelled desired distance
    
    if(targetDistance > 0 && distance >=targetDistance && currentCommand.type == CommandType::FORWARD){
      stop();
      control_needed = false;
      activateLineController = false;
      currentCommand.state = CommandState::COMPLETED;
    }
    control_needed = false;
  }

  if (is_rotate_needed){
    controlRotate();
    is_rotate_needed = false;
  }
  executeAutoCommand();
}

/**
 * @brief Initializes and sets up the MPU6050 sensor.
 */
void setupMPU6050(){
  mpu6050_1.initialize();
  if(mpu6050_1.testConnection()){
    Serial.println("Success");
  }
  else{
    Serial.println("Error");
  }
  //mpu6050_1.setRate(20);
  mpu6050_1.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu6050_1.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
}

/**
 * @brief Function to set calibration offsets for MPU6050 sensor.
 */
void setOffsetsMPU6050(){
  mpu6050_1.setXAccelOffset(AX_OFFSET);
  mpu6050_1.setYAccelOffset(AY_OFFSET);
  mpu6050_1.setZAccelOffset(AZ_OFFSET);
  mpu6050_1.setXGyroOffset (GX_OFFSET);
  mpu6050_1.setYGyroOffset (GY_OFFSET);
  mpu6050_1.setZGyroOffset (GZ_OFFSET);
}

/**
 * @brief Initializes timers.
 */
void setupTimer(){
    // 80 MHz frequency for system clock

    // Every 50 ms
    Timer0_Cfg = timerBegin(0, 80, true);
    timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
    timerAlarmWrite(Timer0_Cfg, 50000, true);
    timerAlarmEnable(Timer0_Cfg);

    // Every 20 ms
    Timer1_Cfg = timerBegin(1, 80, true);
    timerAttachInterrupt(Timer1_Cfg, &Timer1_ISR, true);
    timerAlarmWrite(Timer1_Cfg, 20000, true);
    timerAlarmEnable(Timer1_Cfg);
} 

/**
 * @brief Interrupt Service Routine for Timer0.
 *  Calculates the distance traveled by the robot.
 */
void IRAM_ATTR Timer0_ISR(){
  double avgNumOfHoles = (numOfHolesA + numOfHolesB) / 2;
  //discHoles * 2 because for every hole, ir sensor on RISING picks up 2
  //in total, for one rotation with 20 holes, the sensor reads 40 RISING edges
  double numOfRotations = avgNumOfHoles/(discHoles*2);
  distance = numOfRotations * wheelRadius * 2 * PI / 10.0; //in cm

  //temporarily sending data via bluetooth
  char buffer[30]; 
  itoa((int16_t)adjustedSpeed, buffer, 10);              
  strcat(buffer, ",");                     
  itoa((int16_t)distance, buffer + strlen(buffer), 10); 
  strcat(buffer, ",");
  itoa((int16_t)heading, buffer + strlen(buffer), 10);
  //dtostrf(roll, 0, 6, buffer + strlen(buffer));
  buffer[sizeof(buffer) - 1] = '\0';
  SerialBT.println(buffer);

}

/**
 * @brief Interrupt Service Routine for Timer1.
 *  Activates robot control based on conditions.
 */
void IRAM_ATTR Timer1_ISR(){
  if(activateLineController){
    control_needed = true;
  }
  if(activateRotateController){
    is_rotate_needed = true;
  }
}

/**
 * @brief Task to run on a separate core for MPU calculation
 * @param parameter Unused parameter
 */
void MPUCalculation(void *parameter) {
  (void) parameter; // Unused parameter
  for (;;) {

      // Read accelerometer and gyroscope data from MPU1
      int16_t accX_raw_1, accY_raw_1, accZ_raw_1;
      int16_t gyroX_raw_1, gyroY_raw_1, gyroZ_raw_1;
      mpu6050_1.getMotion6(&accX_raw_1, &accY_raw_1, &accZ_raw_1, &gyroX_raw_1, &gyroY_raw_1, &gyroZ_raw_1);

      // Acceleration [m/s^2] for MPU1
      float accX_ms2_1 = accX_raw_1 / ACCEL_DIVIDER_2G;
      float accY_ms2_1 = accY_raw_1 / ACCEL_DIVIDER_2G;
      float accZ_ms2_1 = accZ_raw_1 / ACCEL_DIVIDER_2G;

      // Angular velocity [deg/s] for MPU1
      gyroX_degs = gyroX_raw_1 / GYRO_DIVIDER_250;
      float gyroY_degs_1 = gyroY_raw_1 / GYRO_DIVIDER_250;
      float gyroZ_degs_1 = gyroZ_raw_1 / GYRO_DIVIDER_250;

      // Find the angle formed by the projection of the Y-axis acceleration onto the plane formed by the X and Z axes.
      float accYaw = (atan(accY_ms2_1 / sqrt(pow(accX_ms2_1, 2) + pow(accZ_ms2_1, 2))) * 180 / PI);
      float gyroYaw;

      // Calculating yaw angle based on gyrosope data by integrating
      previousTime = currentTime;
      currentTime = micros();
      float DT = (currentTime - previousTime) / 1000000.0; // convert to seconds
      gyroYaw += gyroX_degs * DT;

      // Combine accelerometer and gyroscope yaw to get the final yaw (complementary filter)
      double yaw = alpha * gyroYaw + (1-alpha) * accYaw;

      // Sets the angle between 0 and 360
      yaw = fmod(yaw, 360.0);

      // Sets the angle between -180 and 180
      if(yaw > 180){
        yaw -=360;
      }
      else if(yaw<-180){
        yaw +=360;
      }
      heading = yaw;
      //Serial.print(heading);
      //Serial.print(", ");
      //Serial.println(gyroX_degs);
      //vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}

/**
 * @brief Bluetooth callback function
 * @param event Bluetooth callback event
 * @param param Structure containing data.
 */
void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {
  uint8_t* sourceData; 
  char* command; //every command starts with character '&' and ends with ';'
  size_t length;

  switch (event) {
    // If connection to client is lost
    case ESP_SPP_NO_CONNECTION:
    case ESP_SPP_CLOSE_EVT:
     ExecuteManualCommand("&x;"); //stop the mobile robot
     break; 
    // If new data has arrived
    case ESP_SPP_DATA_IND_EVT:
      sourceData = param->data_ind.data;
      length = param->data_ind.len;
      command = (char*) malloc(length+1);
      memcpy(command,sourceData,length);
      command[length]='\0';
      //Serial.println(command);
      if(length==3){ //this means it is a manual command (e.g. "&w;")
        ExecuteManualCommand(command); 
      }
      else{ // e.g. "&w100;"
        ProcessData(command,length);
      }
      free(command);
      break;
    default:
      break;
  }
}

/**
 * @brief ExecuteCommand function for processing manual commands
 * @param command Bluetooth command to execute
 */
void ExecuteManualCommand(const char* command){
  if(strcmp(command,"&w;")==0){
    AMotorSpeed = adjustedSpeed;
    BMotorSpeed = 170;
    targetAngle = heading;
    integral = 0.0f;
    previousError = 0;
    targetDistance = 0;
    numOfHolesA=0;
    numOfHolesB=0;
    distance=0;
    activateLineController = true;
    moveForward();
    updateSpeed(AMotorSpeed,BMotorSpeed);
  }
  else if(strcmp(command,"&s;")==0){
    moveBackward();
    updateSpeed(170,170);
  }
  else if(strcmp(command,"&a;")==0){
    turnLeft();
    updateSpeed(150,150);
    Serial.println("left");

  }
  else if(strcmp(command,"&d;")==0){
    turnRight();
    updateSpeed(150,150);
    Serial.println("right");

  }
  else if(strcmp(command,"&x;")==0){
    activateLineController = false;
    activateRotateController = false;
    is_rotate_needed = false;
    control_needed=false;
    stop();
    Serial.println("stop");
  }
}

/**
 * @brief Function to process commands for automatic control
 * @param data Data to be processed
 * @param length Length of data
 */
void ProcessData(const char* data, size_t length){
  char *buffer = (char*)malloc(length+1);

  int bufferIndex = 0;
  for(int i=0;i<length;i++){
      if(data[i]=='&'){
        bufferIndex=0;
      }

      buffer[bufferIndex++] = data[i];

      if(data[i]==';'){
        buffer[bufferIndex] ='\0';
        Command command = parseCommand(buffer);
        Serial.println(buffer);
        queue.push(command);
      }
  }
}

/**
 * @brief Function to parse string to Command type
 * @param commandString String to be parsed
 * @return Parsed command
 */
Command parseCommand(const char* commandString){
  Command command;
  command.type = CommandType::NONE; 
  command.distance = -1;
  command.angle = -1;
  command.state = CommandState::IDLE;
  if(commandString[0]=='&' && commandString[strlen(commandString)-1] == ';'){
     char type;
     int value;
     int result = sscanf(commandString, "&%c%d;", &type, &value);
     if(result == 2) // Assigned both parameters
     {
        switch (type)
        {
        case 'w':
          command.type=CommandType::FORWARD;
          command.distance=value;
          command.angle = -1;
          break;
        case 's':
          command.type=CommandType::BACKWARD;
          command.distance=value;
          command.angle = -1;
          break;
        case 'a':
          command.type=CommandType::LEFT;
          command.angle=value;
          command.distance=-1;
          break;
        case 'd':
          command.type=CommandType::RIGHT;
          command.angle=value;
          command.distance=-1;   
          break;
        default:
          break;
        }
     }
  }
  return command;
}
/**
 * @brief Checks if there is a new command to be executed
 */
void executeAutoCommand(){
  if(!queue.isEmpty() && currentCommand.state == CommandState::COMPLETED){
    currentCommand = queue.pop();
    if(currentCommand.state != CommandState::COMPLETED){
      execute();
    }
  }
}

/**
 * @brief Implementation of command execution
 */
void execute(){
  switch (currentCommand.type)
  {
  case CommandType::FORWARD:
    AMotorSpeed = adjustedSpeed;
    BMotorSpeed = 170;
    targetAngle = heading;
    targetDistance = currentCommand.distance;
    integral = 0.0f;
    previousError = 0;
    distance=0;
    numOfHolesA=0;
    numOfHolesB=0;
    activateLineController = true;
    moveForward();
    updateSpeed(AMotorSpeed,BMotorSpeed);
    break;

  case CommandType::LEFT:
    AMotorSpeed = 150;
    BMotorSpeed = 150;
    targetAngle = heading + currentCommand.angle * factor;
    if (targetAngle <= -180) {
        targetAngle += 360;
    }
    else if (targetAngle > 180) {
        targetAngle -= 360;
    }
    integral = 0.0f;
    previousError = 0;
    activateRotateController=true;
    break;

  case CommandType::RIGHT:
    AMotorSpeed = 150;
    BMotorSpeed = 150;
    targetAngle = heading - currentCommand.angle * factor;
    if (targetAngle <= -180) {
        targetAngle += 360;
    }
    else if (targetAngle > 180) {
        targetAngle -= 360;
    }
    integral = 0.0f;
    previousError = 0;
    activateRotateController=true;
    break;
  default:
    break;
  }
}

/**
 * @brief Change the motor speed by the specified increment.
 * @param motorSpeed Current motor speed.
 * @param increment Amount by which to increment the motor speed.
 * @return The updated motor speed.
 */
int changeSpeed (int motorSpeed, int increment){
  motorSpeed += increment;
  // Ensure the new speed is inside the interval [140,255].
  // Those are the minimum and maximum pwm values 
  if (motorSpeed > 255){ 
    motorSpeed = 255;
  } else if (motorSpeed <140){
    motorSpeed = 140;
  }
  return motorSpeed;
}


/**
 * @brief Control the robot to move in a straight line.
 */
void controlStraightLine(){
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
  // The robot needs to make a sharper turn (stronger response). It is restricted to a max value of 30
  // Proportional control
  if (deltaAngle > 30){
      targetGyroX = 30;
  } 
  else if (deltaAngle < -30){
    targetGyroX = -30;
  } 
  // For smaller delta angles, we need smaller rotation speed
  else {
    targetGyroX = 1.5 * deltaAngle;
  }

  /* int error = targetGyroX - gyroX_degs;
  integral += error *0.02;
  float output = kp*error + ki*integral + kd*(error - previousError)/0.02; 
  previousError = error;
  AMotorSpeed = changeSpeed(AMotorSpeed,output); */
  
  // Motor A is stronger than motor B. Speed of motor B will remain the same. Speed of motor A will be adjusted accordingly.
  // If the targetGyroX is greater than the current reading, the motor speed (AMotorSpeed) is adjusted by calling the 
  // changeSpeed function with a positive increment (+1). This will increase the motor speed to rotate towards the target angle.
  
  if (round(targetGyroX - gyroX_degs) != 0) {
    AMotorSpeed = (targetGyroX > gyroX_degs) ? changeSpeed(AMotorSpeed, +1) : changeSpeed(AMotorSpeed, -1);
  }
}

/**
 * @brief Control the robot to rotate based on the target angle.
 */
void controlRotate(){
  int deltaAngle = calculateAngleOffset(targetAngle,heading);
  int targetGyroX;
  // Check if the angle difference is small
  if (abs(deltaAngle) <= 2){
    stop();
    // Indicates that the robot has been within the acceptable range for a period of time
    if(countRotate<10){
      countRotate++;
    }else{
      stop();
      activateRotateController = false;
      is_rotate_needed=false;
      countRotate = 0;
      currentCommand.state = CommandState::COMPLETED;
    }
  } 
  else {
    countRotate=0;
    if (should_turn_left(deltaAngle)) {
      turnLeft();
    } 
    else if (should_turn_right(deltaAngle)) {
      turnRight();
    }

    // The robot needs to make a sharper turn (stronger response).
    // With targetGyroX we can influence speed of rotation
    // By increasing or decreasing speed we can change speed of rotation
    if (abs(deltaAngle) > 30){
      targetGyroX = 70;
    } 
    // For smaller delta angles, we need smaller rotation speed
    else {
      targetGyroX = 2 * abs(deltaAngle);
    }

    /* int error = targetGyroX - abs(gyroX_degs);
    integral += error *0.02;
    float output = kp*error + ki*integral + kd*(error - previousError)/0.02; 
    previousError = error;
    AMotorSpeed = changeSpeed(AMotorSpeed,output); */
    
    // similar logic to controlStraightLine
    if (round(targetGyroX - abs(gyroX_degs)) != 0) {
      AMotorSpeed = (targetGyroX > abs(gyroX_degs)) ? changeSpeed(AMotorSpeed, +1) : changeSpeed(AMotorSpeed, -1);
    }

    // B motor has the same speed as motorA so that robot rotates on spot.
    BMotorSpeed = AMotorSpeed;
    updateSpeed(AMotorSpeed,BMotorSpeed);
  }
}   

/**
 * @brief Count the holes for encoder A.
 */
void countHolesA(){
  //count holes only when robot is going foward or backwards
  if ((motors.getMotorAStatus() == motors.FORWARD  && motors.getMotorBStatus() == motors.FORWARD) 
      ||  (motors.getMotorAStatus() == motors.BACKWARD  && motors.getMotorBStatus() == motors.BACKWARD)){
    numOfHolesA++;
  }
}

/**
 * @brief Count the holes for encoder B.
 */
void countHolesB(){
  //count holes only when robot is going foward or backwards
  if ((motors.getMotorAStatus() == motors.FORWARD  && motors.getMotorBStatus() == motors.FORWARD) 
      ||  (motors.getMotorAStatus() == motors.BACKWARD  && motors.getMotorBStatus() == motors.BACKWARD)){
    numOfHolesB++;
  }
}

/**
 * @brief Calculate the angle offset between the target and current angles.
 * @param target_angle Target angle in degrees.
 * @param current_angle Current angle in degrees.
 * @return The angle offset in degrees.
 */
int calculateAngleOffset(float target_angle, float current_angle) {
    // the sign of delta will decide whether the robot should turn right or left.
    int delta;
    if(current_angle > 0){
        delta = (int)(target_angle - current_angle + 180) % 360 - 180;
        if (delta <= -180) {
            delta += 360;
        }
        else if (delta > 180) {
            delta -= 360;
        }
    }
    else if(current_angle < 0 ){
        delta =(int)(current_angle - target_angle + 180) % 360 - 180;
        if (delta <= -180) {
            delta += 360;
        }
        else if (delta > 180) {
            delta -= 360;
        }
        delta = delta*-1;
    }
    
    return delta;
}

/**
 * @brief Check if the robot should turn right based on the delta angle.
 * @param deltaAngle Delta angle in degrees.
 * @return True if the robot should turn right, false otherwise.
 */
bool should_turn_right(int deltaAngle) {
    return deltaAngle < 0;
}

/**
 * @brief Check if the robot should turn left based on the delta angle.
 * @param deltaAngle Delta angle in degrees.
 * @return True if the robot should turn left, false otherwise.
 */
bool should_turn_left(int deltaAngle){
    return deltaAngle > 0;
}

/**
 * @brief Move the robot forward.
 */
void moveForward(){
  motors.moveForward_A();
  motors.moveForward_B();
}

/**
 * @brief Move the robot backward.
 */
void moveBackward(){
  motors.moveBackward_A();
  motors.moveBackward_B();
}

/**
 * @brief Rotate the robot right
 */
void turnRight(){
  motors.moveBackward_A();
  motors.moveForward_B();
}

/**
 * @brief Rotate the robot left
 */
void turnLeft(){
  motors.moveForward_A();
  motors.moveBackward_B();
}

/**
 * @brief Stop the robot
 */
void stop(){
  motors.stop_A();
  motors.stop_B();
}

/**
 * @brief Update the speeds of both motors.
 * @param speedA Speed of motor A.
 * @param speedB Speed of motor B.
 */
void updateSpeed(uint16_t speedA, uint16_t speedB){
  motors.updateSpeed_A(speedA);
  motors.updateSpeed_B(speedB);
}