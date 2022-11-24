// #include <Pololu3piPlus32U4.h>

#include "pins.h"
#include "motor.h"
#include "encoder.h"
#include "line_sensor.h"
#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include "states.h"
#include "imu_utils.h"

// #define USE_IMU
#define DEBUG

Motor leftMotor(LMOTOR_DIR_PIN, LMOTOR_SPEED_PIN);
Motor rightMotor(RMOTOR_DIR_PIN, RMOTOR_SPEED_PIN);

struct {
  LSM6 imu;
  LIS3MDL mag;
  LineSensor linesensor;
} DEVICES;

struct {
  uint8_t taskState;
  uint8_t noLineCounter = 0;
  float targetHeading = 0.0;
  uint8_t previousTaskState;

} TASK_DATA;

void blinkAndBeep(uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    digitalWrite(YELLOW_PIN, HIGH);
    delay(200);
    digitalWrite(YELLOW_PIN, LOW);
    delay(200);
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();

  // Yellow pin as an indicator
  pinMode(YELLOW_PIN, OUTPUT);

  // Left interrupt encoder is a "Pin change interrupt" which means an interrupt is triggered when signal is received by any pin on the same port.
  // This means we must perfrom extra operations to enable it.
  pinMode(ENC_XOR_LMOTOR, INPUT_PULLUP);
  ENC::setupPinChangeEncoder();

  // This is an external interrupt, so no special considerations
  pinMode(ENC_XOR_RMOTOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_XOR_RMOTOR), ENC::rightMotorInterrupt, CHANGE);

  ENC::ENC_DATA.lastTime = millis();
  delay(1000);

  #ifdef USE_IMU

  // Check the IMU initialised ok.
  if (!DEVICES.imu.init() ) {  // no..? :(

    // Since we failed to communicate with the
    // IMU, we put the robot into an infinite
    // while loop and report the error.
    while (1) {
      Serial.println("Failed to detect and initialize IMU!");
      delay(1000);
    }
  }

  // Check we have intialised commmuncation
  if (!DEVICES.mag.init() ) {  // no..? :(

    // Since we failed to communicate with the
    // magnetometer, we put the robot into an infinite
    // while loop and report the error.
    while(1) {
      Serial.println("Failed to detect and initialize magnetometer!");
      delay(1000);
    }
  }

  DEVICES.imu.enableDefault();
  DEVICES.mag.enableDefault();
  blinkAndBeep(1);
  GYROSCOPE::calibrateGyroscope(&DEVICES.imu);
  blinkAndBeep(2);
  MAGNETOMETER::calibrateMagnetometer(&DEVICES.mag);
  blinkAndBeep(3);

  #endif
  delay(1000);

  // linesensor.calibrateForWhite();
  

  TASK_DATA.taskState = TASK_STATES::TASK_BEGIN;
  // TASK_DATA.taskState = TASK_STATES::FOLLOWING_LINE;

}



float fusion_bias = 1;
long int gyroZ_deg_moved;
float getDegreesMoved(int timePassed) {
  gyroZ_deg_moved = GYROSCOPE::getGyroReading(timePassed, &DEVICES.imu);
  return (fusion_bias * (gyroZ_deg_moved) + (1 - fusion_bias) * MAGNETOMETER::getMagnetoReading(&DEVICES.mag));
}


int timePassed, turnerTimer;
double speedRps, leftMotorCPR, rightMotorCPR;
float heading, degreesMoved;
float deltaRpsP = 0.08, deltaRpsD = 1;
float headingError, previousHeadingError;
uint8_t fullReadings, calibratedReadings;
void loop() {
  //-----------------------------COLLECT INPUTS-----------------------------------
  // Update time passed
  timePassed = (millis() - ENC::ENC_DATA.lastTime);
  ENC::ENC_DATA.lastTime = millis();
  
  // Copy the data after disabling interrupts to avoid data change during the copy operation
  noInterrupts();
  leftMotorCPR = ENC::ENC_DATA.leftMotorCounter;
  rightMotorCPR = ENC::ENC_DATA.rightMotorCounter;
  ENC::resetCounters();
  interrupts();

  float leftMotorRotations = leftMotorCPR / ENC_COUNTS_PER_ROTATION;
  float rightMotorRotations = rightMotorCPR / ENC_COUNTS_PER_ROTATION;

  // Read line sensor
  // leftMotor.stop();
  // rightMotor.stop();
  // Motor::targetRps = 0;
  DEVICES.linesensor.read();

  //------------------------------UPDATE TASK STATE-------------------------------

  // Calculate rotations per second
  leftMotor.currentRps = leftMotorRotations / (timePassed / 1000);
  rightMotor.currentRps = rightMotorRotations / (timePassed / 1000);

  if (isnan(leftMotor.currentRps) || isinf(leftMotor.currentRps)) {
    leftMotor.currentRps = 0;
  }

  if (isnan(rightMotor.currentRps) || isinf(rightMotor.currentRps)) {
    rightMotor.currentRps = 0;
  }
  

  // Set direction of rotations
  if (leftMotor.currentDirection == REVERSE_DIRECTION) leftMotorRotations = - leftMotorRotations;
  if (rightMotor.currentDirection == REVERSE_DIRECTION) rightMotorRotations = - rightMotorRotations;

  float leftMotorDistanceMoved = (leftMotorRotations * WHEEL_CIRCUMFERENCE) / 2;
  float rightMotorDistanceMoved = (rightMotorRotations * WHEEL_CIRCUMFERENCE) / 2;
  // Serial.println(leftMotorDistanceMoved);

  //                      Gyro reading                    Mag reading
  // degreesMoved = getDegreesMoved(timePassed);
  // heading = 0;
  // degreesMoved = (rightMotorDistanceMoved - leftMotorDistanceMoved) * (360 / BOT_CIRCUMFERENCE);
  // heading += degreesMoved;
  // targetHeading -= degreesMoved;

  // Correct heading
  // headingError = TASK_DATA.targetHeading - heading;
  // Motor::deltaRps = ((headingError * deltaRpsP) + ((headingError - previousHeadingError) / timePassed) * deltaRpsD);
  // previousHeadingError = headingError;

  TASK_DATA.previousTaskState = TASK_DATA.taskState;
  fullReadings = DEVICES.linesensor.getCalibratedReadings();
  calibratedReadings = (fullReadings & FRONT_SENSOR_MASK) >> 1;  // These will be mirrored. Rightmost will represent the leftmost sensor
  switch (TASK_DATA.taskState) {
    case TASK_STATES::TASK_BEGIN: {
      TASK_DATA.targetHeading = 0;
      rightMotor.setDirection(FORWARD_DIRECTION);
      leftMotor.setDirection(FORWARD_DIRECTION);
      Motor::targetRps = 0;
      TASK_DATA.taskState = TASK_STATES::WHITE_CALIBRATION;
      break;
    }
    case TASK_STATES::WHITE_CALIBRATION: {
      blinkAndBeep(1);
      DEVICES.linesensor.calibrateForWhite();
      blinkAndBeep(1);
      Motor::targetRps = SLOW_RPS;
      TASK_DATA.taskState = TASK_STATES::FIND_BLACK_LINE;
      break;
    }
    case TASK_STATES::FIND_BLACK_LINE: {
      // Get the readings in a easy to use format using only the front 3 sensors
      // Determine if the front 3 line sensors are getting anything except 0. This is your black line.
      if (calibratedReadings > 0b000 && calibratedReadings < 0b1000) {
        // Stop the robot to get readings
        // leftMotor.stop();
        // rightMotor.stop();
        // Motor::targetRps = 0;
        TASK_DATA.taskState = TASK_STATES::FOLLOWING_LINE;
      }
      break;
    }
    
    case TASK_STATES::FOLLOWING_LINE: {
      // digitalWrite(YELLOW_PIN, HIGH);
      Motor::targetRps = SLOW_RPS;
      // Bang bang strategy
      switch (calibratedReadings) {
        // No line found
        case 0b000: {
          TASK_DATA.taskState = TASK_STATES::NO_LINE;
          Motor::targetRps = 0;
          Motor::deltaRps = 0;
          break;
        }
        // // Turn left - extreme
        // case 0b001: {
        //   Motor::deltaRps = 0.4;
        //   break;
        // }
        // // Turn left
        // case 0b011: {
        //   Motor::deltaRps = 0.2;
        //   break;
        // }
        // // Turn right - extreme
        // case 0b100: {
        //   Motor::deltaRps = -0.4;
        //   break;
        // }
        // // Turn right
        // case 0b110: {
        //   Motor::deltaRps = -0.2;
        //   break;
        // }
        // // This could be meeting up with a line at 90. So we default to turning left behavior
        // case 0b111: {
          
        //   break;
        // }
        default: {

          Motor::deltaRps = DEVICES.linesensor.getDifferentialError() * 3;
        }
      }
      
      // No line ? Go to no line
      // digitalWrite(YELLOW_PIN, LOW);
      break;
    }
    case TASK_STATES::TURN_LEFT: {
      turnerTimer += timePassed;
      if (calibratedReadings != 0b000 || turnerTimer >= 2000) {
        turnerTimer = 0;
        Motor::targetRps = 0;
        Motor::deltaRps = 0;
        TASK_DATA.taskState = TASK_STATES::FOLLOWING_LINE;
      }
      else {
        Motor::targetRps = 0;
        Motor::deltaRps = 2;
      }
      break;
    }
    case TASK_STATES::TURN_RIGHT: {
      turnerTimer += timePassed;
      if (calibratedReadings != 0b000) {
        turnerTimer = 0;
        Motor::targetRps = 0;
        Motor::deltaRps = 0;
        TASK_DATA.taskState = TASK_STATES::FOLLOWING_LINE;
      }
      else {
        Motor::targetRps = 0;
        Motor::deltaRps = -2;
      }
      break;
    }
    case TASK_STATES::NO_LINE: {
      DEVICES.linesensor.read(true);
      fullReadings = DEVICES.linesensor.getCalibratedReadings();
      calibratedReadings = (fullReadings & BACK_SENSOR_MASK);
      // if (fullReadings == 0b00001){
      if (DEVICES.linesensor.currentReadings[0] >= DEVICES.linesensor.currentReadings[SENSOR_COUNT]){
        TASK_DATA.taskState = TASK_STATES::TURN_LEFT;
      }
      // else if (fullReadings & BACK_SENSOR_MASK == 0b10000){
      else if (DEVICES.linesensor.currentReadings[0] < DEVICES.linesensor.currentReadings[SENSOR_COUNT]){
        TASK_DATA.taskState = TASK_STATES::TURN_RIGHT;
      }
      // else if (fullReadings & BACK_SENSOR_MASK == 0b10001) {
      //   TASK_DATA.taskState = TASK_STATES::TURN_LEFT;
      // }
      // else if (millis() < 10000) {
      //   TASK_DATA.taskState = TASK_STATES::TURN_LEFT;
      // }
      else if (DEVICES.linesensor.currentReadings[0] < READ_TIME_THRESHOLD - 1000 && DEVICES.linesensor.currentReadings[SENSOR_COUNT] < READ_TIME_THRESHOLD - 1000) {
        Motor::targetRps = SLOW_RPS;
        Motor::deltaRps = 0;
        TASK_DATA.taskState = TASK_STATES::FOLLOWING_LINE;
      }
      else {
        Motor::targetRps = SLOW_RPS;
        Motor::deltaRps = 0;
        TASK_DATA.taskState = TASK_STATES::FOLLOWING_LINE;
      }
      // else{
      //   // Join up
      //   if (millis() < 20000 && millis() > 1000){
      //     turnerTimer += timePassed;
      //     if (turnerTimer > 500) {
      //       TASK_DATA.taskState = TASK_STATES::FOLLOWING_LINE;
      //       turnerTimer = 0;
      //       Motor::targetRps = SLOW_RPS - 0.5;
      //     }
      //     else {
      //       Motor::targetRps = 0;
      //       Motor::deltaRps = -2;
      //     }
      //   }
      //   // First 90
      //   else if (millis() >= 20000 && millis() < 42000) {
      //     turnerTimer += timePassed;
      //     if (calibratedReadings != 0b000 || turnerTimer > 2000) {
      //       TASK_DATA.taskState = TASK_STATES::FOLLOWING_LINE;
      //       turnerTimer = 0;
      //     }
      //     else {
      //       Motor::targetRps = 0;
      //       Motor::deltaRps = -2;
      //     }
      //   }
      //   // Gap
      //   else if (millis() >= 42000 && millis() < 49000) {
      //     if (calibratedReadings != 0b000) {
      //       TASK_DATA.taskState = TASK_STATES::FOLLOWING_LINE;
      //     }
      //     else {
      //       Motor::targetRps = SLOW_RPS;
      //       Motor::deltaRps = 0;
      //     }
      //   }
      //   // Second 90
      //   else if (millis() >= 49000 && millis() < 60000){
      //     if (calibratedReadings != 0b000) {
      //       TASK_DATA.taskState = TASK_STATES::FOLLOWING_LINE;
      //     }
      //     else {
      //       Motor::targetRps = 0;
      //       Motor::deltaRps = 2;
      //     }
      //   }
      //   else if (millis() >= 60000) {
      //     Motor::targetRps = 0;
      //     Motor::deltaRps = 0;
      //     TASK_DATA.taskState = TASK_STATES::HOMECOMING;
      //   }
      // }
      break;
    }
    case TASK_STATES::HOMECOMING: {
      turnerTimer += timePassed;
      if (turnerTimer >= 1400) {
        Motor::targetRps = 0;
        Motor::deltaRps = 0;
        turnerTimer = 0;
        TASK_DATA.taskState = TASK_STATES::FINISH_AND_CHILL;
      }
      else {
        Motor::targetRps = 0;
        Motor::deltaRps = 2;
      }
      break;
    }
    case TASK_STATES::FINISH_AND_CHILL: {
      digitalWrite(YELLOW_PIN, HIGH);
      turnerTimer += timePassed;
      if (turnerTimer >= 15000) {
        Motor::targetRps = 0;
        Motor::deltaRps = 0;
        // turnerTimer = 0;
        // TASK_DATA.taskState = TASK_STATES::FINISH_AND_CHILL;
      }
      else {
        Motor::targetRps = 3;
        Motor::deltaRps = 0;
      }
      break;
    }
  }

  
  
  //---------------------------------OUTPUT--------------------------------------
  // Correct speed of motors
  leftMotor.correctSpeed(Motor::targetRps - Motor::deltaRps, timePassed);
  rightMotor.correctSpeed(Motor::targetRps + Motor::deltaRps, timePassed);


  #ifdef DEBUG
  // Serial.print("Error:");
  // Serial.print(static_cast<int>(headingError));
  // Serial.print(",Target:");
  // Serial.print(static_cast<int>(TASK_DATA.targetHeading));
  // Serial.print(",Current:");
  // Serial.print(static_cast<int>(heading));
  // Serial.print(Motor::targetRps - Motor::deltaRps);
  // Serial.print('\t');
  // Serial.print(Motor::targetRps + Motor::deltaRps);
  // Serial.println(Motor::deltaRps);
  // Serial.print('\t');
  Serial.print(TASK_DATA.taskState);
  Serial.print('\t');
  // Serial.print(DEVICES.linesensor.pathCount);
  // Serial.print('\t');
  // if (DEVICES.linesensor.pathCount > 0) {
  //   for (int i = 0; i < DEVICES.linesensor.pathCount; i++) {
  //     Serial.print(DEVICES.linesensor.pathsAngles[i]);
  //     Serial.print('\t');
  //   }
  // }
  Serial.print(leftMotor.currentSpeed);
  Serial.print('\t');
  Serial.print(rightMotor.currentSpeed);
  Serial.print('\t');

  // int readings = DEVICES.linesensor.getCalibratedReadings();
  // for (int i = 1; i < SENSOR_COUNT - 1; i++) {
  //   Serial.print((readings >> i) & 0b1);
  //   Serial.print('\t');
  // }

  // for (int i = 1; i < SENSOR_COUNT - 1; i++) {
  //   Serial.print(DEVICES.linesensor.currentReadings[i]);
  //   Serial.print('\t');
  // }
  Serial.println();
#endif
  
  
  // delay(50);
  // int extraDelay = 100 - timePassed;
  // if (extraDelay > 0) {
  //   delay(extraDelay);
  // }

  


}
