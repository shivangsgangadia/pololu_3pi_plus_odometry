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
#include "log_and_write.h"
#include <Pushbutton.h>
#include <math.h>

// #define USE_IMU
// #define DEBUG
#define PRE_TASK_DELAY 2000

Motor leftMotor(LMOTOR_DIR_PIN, LMOTOR_SPEED_PIN);
Motor rightMotor(RMOTOR_DIR_PIN, RMOTOR_SPEED_PIN);
Logger dataLogger;

struct {
  LSM6 imu;
  LIS3MDL mag;
  LineSensor linesensor;
} DEVICES;
Pushbutton button(BUTTON_A);

struct {
  uint8_t taskState;
  uint8_t noLineCounter = 0;
  float targetHeading = 0.0;
  float currentHeading = 0.0;
  uint8_t previousTaskState;
  float X_pos = 0.0;
  float Y_pos = 0.0;
} TASK_DATA;

void blinkAndBeep(uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    digitalWrite(YELLOW_PIN, HIGH);
    delay(200);
    digitalWrite(YELLOW_PIN, LOW);
    delay(200);
  }
}

long lineSensorTimerMicros;
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
  // Preserve this order because prepRead involves a 10us delay
  DEVICES.linesensor.prepRead();
  lineSensorTimerMicros = micros();

}


float fusion_bias = 1;
long int gyroZ_deg_moved;
float getDegreesMoved(int timePassed) {
  gyroZ_deg_moved = GYROSCOPE::getGyroReading(timePassed, &DEVICES.imu);
  return (fusion_bias * (gyroZ_deg_moved) + (1 - fusion_bias) * MAGNETOMETER::getMagnetoReading(&DEVICES.mag));
}


long timePassed, dataRecordTimer;
double speedRps, leftMotorCPR, rightMotorCPR;
float degreesMoved, distanceMoved_x;
float deltaRpsP = 0.1, deltaRpsD = 0.8;
float headingError, previousHeadingError;
uint8_t fullReadings, calibratedReadings;
void loop() {
  //-----------------------------COLLECT INPUTS-----------------------------------
  // Update time passed
  timePassed = (millis() - ENC::ENC_DATA.lastTime);
  ENC::ENC_DATA.lastTime = millis();
  dataRecordTimer += timePassed;
  
  // Copy the data after disabling interrupts to avoid data change during the copy operation
  noInterrupts();
  leftMotorCPR = ENC::ENC_DATA.leftMotorCounter;
  rightMotorCPR = ENC::ENC_DATA.rightMotorCounter;
  ENC::resetCounters();
  interrupts();

  float leftMotorRotations = leftMotorCPR / ENC_COUNTS_PER_ROTATION;
  float rightMotorRotations = rightMotorCPR / ENC_COUNTS_PER_ROTATION;

  // Read and update line sensor
  DEVICES.linesensor.blockingRead();
  fullReadings = DEVICES.linesensor.getCalibratedReadings();
  DEVICES.linesensor.findPaths();

  //------------------------------UPDATE ROBOT & SENSOR STATE-------------------------------

  // Calculate rotations per second for both motors
  leftMotor.currentRps = leftMotorRotations / (timePassed / 1000);
  rightMotor.currentRps = rightMotorRotations / (timePassed / 1000);

  // If movement of wheels is too small or time period is too small, currentRps often becomes 0 or indeterminate, respectively
  if (isnan(leftMotor.currentRps) || isinf(leftMotor.currentRps)) {
    leftMotor.currentRps = 0;
  }

  if (isnan(rightMotor.currentRps) || isinf(rightMotor.currentRps)) {
    rightMotor.currentRps = 0;
  }
  

  // Set direction of rotations
  if (leftMotor.currentDirection == REVERSE_DIRECTION) leftMotorRotations = - leftMotorRotations;
  if (rightMotor.currentDirection == REVERSE_DIRECTION) rightMotorRotations = - rightMotorRotations;

  // Calculate Distance Moved
  float leftWheelDistanceMoved = (leftMotorRotations * WHEEL_CIRCUMFERENCE);
  float rightWheelDistanceMoved = (rightMotorRotations * WHEEL_CIRCUMFERENCE);
  distanceMoved_x = (leftWheelDistanceMoved + rightWheelDistanceMoved) / 2; // TODO: make A - B and then check

  //                      Gyro reading                    Mag reading
  // degreesMoved = getDegreesMoved(timePassed);
  // heading = 0;
  degreesMoved = (rightWheelDistanceMoved - leftWheelDistanceMoved) * (360 / BOT_CIRCUMFERENCE);
  TASK_DATA.currentHeading += degreesMoved;
  TASK_DATA.X_pos += distanceMoved_x * cos(degreesMoved);
  TASK_DATA.Y_pos += distanceMoved_x * sin(degreesMoved);
  // targetHeading -= degreesMoved;

  // Correct heading
  headingError = TASK_DATA.targetHeading - TASK_DATA.currentHeading;
  Motor::deltaRps = ((headingError * deltaRpsP) + ((headingError - previousHeadingError) / timePassed) * deltaRpsD);
  previousHeadingError = headingError;

  //--------------------------UPDATE TASK STATE BASED ON SENSORS---------------------------
  TASK_DATA.previousTaskState = TASK_DATA.taskState;
  switch (TASK_DATA.taskState) {
    case TASK_STATES::PRE_TASK_SUSPEND: {
      Motor::targetRps = 0;
      Motor::deltaRps = 0;
      break;
    }
    case TASK_STATES::PRE_TASK_PAUSE: {
      // You can come to this task from suspend only using button
      TASK_DATA.targetHeading = 0;
      delay(PRE_TASK_DELAY);
      TASK_DATA.taskState = TASK_STATES::FOLLOWING_LINE;
      break;
    }
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
      // if (DEVICES.linesensor.pathCount > 0) {
      //   TASK_DATA.taskState = TASK_STATES::FOLLOWING_LINE;
      // }
      if (dataRecordTimer >= (1000 / LOG_FREQUENCY)) {
        dataLogger.addWayPoint(TASK_DATA.X_pos, TASK_DATA.Y_pos);
        dataRecordTimer = 0;
      }
      if (TASK_DATA.X_pos >= 250) {
        TASK_DATA.taskState = TASK_STATES::FOLLOWING_LINE;
      }
      break;
    }
    
    case TASK_STATES::FOLLOWING_LINE: {
      Motor::targetRps = SLOW_RPS;
      
      TASK_DATA.taskState = TASK_STATES::NO_LINE;
      // Line found
      // if (DEVICES.linesensor.pathCount > 0) {
      //   TASK_DATA.targetHeading = TASK_DATA.currentHeading - DEVICES.linesensor.pathsAngles[0];
      // }
      // // No line
      // else {
      //   TASK_DATA.taskState = TASK_STATES::NO_LINE;
      // }
      break;
    }
    
    case TASK_STATES::NO_LINE: {
      TASK_DATA.taskState = TASK_STATES::STOP;
      
      break;
    }

    case TASK_STATES::STOP: {
      // Although this is same as PRE_TASK_SUSPEND, it needs to exist independently because the button press event needs to know where in the task's runtime we are.
      Motor::targetRps = 0;
      Motor::deltaRps = 0;
      break;
    }

    case TASK_STATES::WRITE_TO_SERIAL: {
      // You can get to this state from STOP only using button
      // Write to serial
      dataLogger.writeToSerial();
      // Go to suspend
      TASK_DATA.taskState = TASK_STATES::PRE_TASK_SUSPEND;
      break;
    }
  }

  
  
  //---------------------------------OUTPUT--------------------------------------
  // Correct speed of motors every iteration
  leftMotor.correctSpeed(Motor::targetRps - Motor::deltaRps, timePassed);
  rightMotor.correctSpeed(Motor::targetRps + Motor::deltaRps, timePassed);

  if (button.getSingleDebouncedPress()) {
    if (TASK_DATA.taskState == TASK_STATES::STOP) {
      TASK_DATA.taskState = TASK_STATES::WRITE_TO_SERIAL;
    }
    else if (TASK_DATA.taskState == TASK_STATES::PRE_TASK_SUSPEND) {
      TASK_DATA.taskState = TASK_STATES::PRE_TASK_PAUSE;
    }
  }

  

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
  // Serial.print(TASK_DATA.taskState);
  // Serial.print('\t');
  // Serial.print(DEVICES.linesensor.pathCount);
  // Serial.print('\t');
  // if (DEVICES.linesensor.pathCount > 0) {
  //   for (int i = 0; i < DEVICES.linesensor.pathCount; i++) {
  //     Serial.print(DEVICES.linesensor.pathsAngles[i]);
  //     Serial.print('\t');
  //   }
  // }
  // Serial.print(leftMotor.currentSpeed);
  // Serial.print('\t');
  // Serial.print(rightMotor.currentSpeed);
  // Serial.print('\t');

  // for (int i = 0; i < SENSOR_COUNT; i++) {
  //   Serial.print((fullReadings >> i) & 0b1);
  //   Serial.print('\t');
  // }

  // for (int i = 0; i < SENSOR_COUNT; i++) {
  //   Serial.print(i);
  //   Serial.print(":");
  //   Serial.print(DEVICES.linesensor.currentReadings[i]);
  //   Serial.print(",");
  // }
  
  Serial.print("X:");
  Serial.print(TASK_DATA.X_pos);
  Serial.print(",Y:");
  Serial.print(TASK_DATA.Y_pos);
  Serial.print(",theta:");
  Serial.print(TASK_DATA.currentHeading);
  Serial.println();
#endif
  
  // int extraDelay = 100 - timePassed;
  // if (extraDelay > 0) {
  //   delay(extraDelay);
  // }

}
