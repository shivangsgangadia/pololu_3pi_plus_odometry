#ifndef __STATES_H
#define __STATES_H

namespace TASK_STATES {
  const uint8_t TASK_BEGIN = 1;
  const uint8_t WHITE_CALIBRATION = 2;
  const uint8_t FIND_BLACK_LINE = 3;
  const uint8_t BLACK_CALIBRATION_FORWARD = 4;
  const uint8_t BLACK_CALIBRATION_BACKWARD = 5;
  const uint8_t FOLLOWING_LINE = 6;
  const uint8_t NO_LINE = 7;
  const uint8_t HOMECOMING = 8;
  const uint8_t FINISH_AND_CHILL = 9;
  const uint8_t TURN_LEFT = 10;
  const uint8_t TURN_RIGHT = 11;
}

#endif