#include "Arduino.h"
#ifndef __LOG_WRITE_H
#define __LOG_WRITE_H

#define WAY_POINT_COUNT 200
#define LOG_FREQUENCY 1

/**
This system will enable the bot to log and write coordinates when a button is pressed
*/

class Logger {
  public:
  float wayPoints[WAY_POINT_COUNT][2];
  int wayPointCount, sentWayPointCount;

  Logger () {
    for (int i = 0; i < WAY_POINT_COUNT; i++) {
      for (int j = 0; j < 2; j++) {
        this->wayPoints[i][j] = i * j;
      }
    }
  }

  /**
  Stores waypoints to memory if memory is available
  @return true, if way point was stored. false, if way point was not stored due to lack of memory
  */
  bool addWayPoint(float x, float y) {
    this->wayPointCount++;
    // Check for overflow
    if (this->wayPointCount >= WAY_POINT_COUNT) {
      this->wayPointCount--;
      return false;
    }

    this->wayPoints[this->wayPointCount - 1][0] = x;
    this->wayPoints[this->wayPointCount - 1][1] = y;
    return true;
  }

  /**
  Writes stored waypoints to serial and clears @variable this->wayPointCount
  */
  void writeToSerial() {
    digitalWrite(YELLOW_PIN, HIGH);
    if (this->sentWayPointCount < this->wayPointCount) {
      for (int i = 0; i < this->wayPointCount; i++) {
        Serial.print(this->wayPoints[i][0]);
        Serial.print(',');
        Serial.println(this->wayPoints[i][1]);
        this->sentWayPointCount++;
        delay(100);
      }
      Serial.println("11111111111111111111");
    }
    digitalWrite(YELLOW_PIN, LOW);
  }
};


#endif