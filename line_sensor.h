#include <stdint.h>
#include "Arduino.h"
#ifndef __LINESENSE_H
#define __LINESENSE_H

#define SENSOR_COUNT 5
#define MAX_PATH_COUNT 3
#define SENSOR_VARIANCE 10000
#define WHITE_CALIBRATION_TIME 2000
#define BLACK_CALIBRATION_TIME 2000
#define READ_TIME_THRESHOLD 300000U
#define FRONT_SENSOR_MASK 0b01110
#define BACK_SENSOR_MASK 0b10001
#define LINESENSOR_PIN (reinterpret_cast<uint8_t*>(&LINESENSOR_PINS))

struct {
  uint8_t LEFT_2 = 12;
  uint8_t LEFT_1 = A0;
  uint8_t CENTER = A2;
  uint8_t RIGHT_1 = A3;
  uint8_t RIGHT_2 = A4;
  uint8_t IR_ENABLE = 11;
} LINESENSOR_PINS;

const int SENSOR_ANGLES[5] = {
  -50, -15, 0, 15, 50
};

class LineSensor {
  public:
  float pathsAngles[3];
  int pathCount;
  float sensorWeights[SENSOR_COUNT];
  
  struct {
    unsigned long maxWhite[SENSOR_COUNT];
  } CALIBRATION_RANGES;

  unsigned long currentReadings[SENSOR_COUNT];

  LineSensor() {
    for (int i = 0; i < SENSOR_COUNT; i++) {
      CALIBRATION_RANGES.maxWhite[i] = 0;
    }
    pinMode(LINESENSOR_PINS.IR_ENABLE, OUTPUT);
  }

  void calibrateForWhite() {
    // int initTime = millis();
    // while (millis() - initTime <= WHITE_CALIBRATION_TIME) {
    //   this->read(true);
    // }
    delay(WHITE_CALIBRATION_TIME);
  }

  /**
  Reads the sensor values and stores them in object's public variables
  Ideally, only 1 of the two bool switch will be active at a time
  */
  void read(bool getBackReadings = false) {
    int currentPinNum = 0;
    unsigned long initTimeBuffer, timePassed;

    // 1. Set the infra-red LEDs to be enabled, to transmit infra-red light to reflect off the work surface
    pinMode(LINESENSOR_PINS.IR_ENABLE, OUTPUT);
    digitalWrite(LINESENSOR_PINS.IR_ENABLE, HIGH);
    
    // 2. Set the sensor measurement pin to an OUTPUT with state HIGH, to charge the capacitor
    for (int i = 0; i < SENSOR_COUNT; i++) {
      currentPinNum = LINESENSOR_PIN[i];
      pinMode(currentPinNum, OUTPUT);
      digitalWrite(currentPinNum, HIGH);
      this->currentReadings[i] = 0;
    }
    
    // 3. Wait for 10microseconds for the capacitor to charge to full
    delayMicroseconds(10);
    
    // 4. Set the sensor measurement pin to an INPUT
    for (int i = 0; i < SENSOR_COUNT; i++) {
      currentPinNum = LINESENSOR_PIN[i];
      pinMode(currentPinNum, INPUT);
    }
    
    // Disable the infra red LEDs
    // digitalWrite(LINESENSOR_PINS.IR_ENABLE, LOW);
    pinMode(LINESENSOR_PINS.IR_ENABLE, INPUT);
    
    // 5. Recording the passage of time, wait for digitalRead() on the sensor measurement pin to change to LOW whlist the capacitor discharges.
    initTimeBuffer = micros();
    int sensorCount;
    if (getBackReadings) {
      sensorCount = SENSOR_COUNT;
    }
    else {
      sensorCount = SENSOR_COUNT - 2;
    }
    bool timeout = false;
    
    while(true) {
      timePassed = micros() - initTimeBuffer;
      if (timePassed >= READ_TIME_THRESHOLD) timeout = true;
      if (getBackReadings) {
        for (int i = 0; i < SENSOR_COUNT; i++) {
          currentPinNum = LINESENSOR_PIN[i];
          
          // 6. Once digitalRead() has change to LOW, record the elapsed time as the sensor reading.
          if ((digitalRead(currentPinNum) == LOW || timeout) && this->currentReadings[i] == 0) {
            this->currentReadings[i] = timePassed;
            sensorCount--;
            
            // Add calibration values to ranges. White is here because we want to save iteration cycles as it reads all
            // at once for all sensors
            // if (forCalibrationWhite) {
            //   if (timePassed > CALIBRATION_RANGES.maxWhite[i]) CALIBRATION_RANGES.maxWhite[i] = timePassed + SENSOR_VARIANCE;
            // }
          }
        }
      } else {
        for (int i = 1; i < SENSOR_COUNT - 1; i++) {
          currentPinNum = LINESENSOR_PIN[i];
          
          // 6. Once digitalRead() has change to LOW, record the elapsed time as the sensor reading.
          if ((digitalRead(currentPinNum) == LOW || timeout) && this->currentReadings[i] == 0) {
            this->currentReadings[i] = timePassed;
            sensorCount--;
            
            // Add calibration values to ranges. White is here because we want to save iteration cycles as it reads all
            // at once for all sensors
            // if (forCalibrationWhite) {
            //   if (timePassed > CALIBRATION_RANGES.maxWhite[i]) CALIBRATION_RANGES.maxWhite[i] = timePassed + SENSOR_VARIANCE;
            // }
          }
        }
      }
      // Exit loop once all sensors have been read
      if (sensorCount == 0) {
        break;
      }
    }

  }

  /**
  Checks if the line sensors detect anything except white (white = 0, non-white = 1)
  */
  uint8_t getCalibratedReadings() {
    uint8_t result = 0;

    for (int i = 0; i < SENSOR_COUNT; i++) {
      // if (currentReadings[i] > CALIBRATION_RANGES.maxWhite[i]) {
      //   result |= (0b1 << i);
      // }
      if (currentReadings[i] > READ_TIME_THRESHOLD - 1000) {
        result |= (0b1 << i);
      }
    }

    return result;
  }


  // TODO : determine number of paths and their respective angles
  void findPaths() {
    this->pathCount = 0;
    // this->read();
    uint8_t readings = getCalibratedReadings();
    // Begin looping through sensors
    for (int i = 0; i < SENSOR_COUNT; i++) {
      float averagePathAngle = 0;
      // If activated sensor is found
      if ((readings >> i) & 0b1 == 1) {
        int j = i, anglesInPath = 0;
        // Loop through sensors until 0 is found.
        while ((readings >> j) & 0b1 == 1){
          averagePathAngle += SENSOR_ANGLES[j];
          anglesInPath++;
          j++;
          if (j > SENSOR_COUNT - 1) break;
        }
        i = j + 1;
        // Calculate average path angle
        averagePathAngle /= anglesInPath;
        // Add this to list of available paths
        this->pathsAngles[pathCount] = averagePathAngle;
        this->pathCount++;
      }
    }
    // Now we should have list of available path angles.
  }

  float getDifferentialError() {
    int lineDetected = 0;
    float sum = 0;
    for (int i = 1; i < SENSOR_COUNT - 1; i++) {
      if (this->currentReadings[i] > READ_TIME_THRESHOLD - 1000) {
        lineDetected++;
      }
      sum += this->currentReadings[i];
    }
    // BLACK line found
    if (lineDetected > 0) {
      for(int i = 1; i < SENSOR_COUNT - 1; i++){
        sensorWeights[i] = this->currentReadings[i] / sum;
      }
    }
    float leftWeight = sensorWeights[1] + (0.3 * sensorWeights[2]);
    float rightWeight = sensorWeights[3] + (0.3 * sensorWeights[2]);
    return (leftWeight - rightWeight);
  }

};

#endif