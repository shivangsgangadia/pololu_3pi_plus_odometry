#ifndef __STATES_H
#define __STATES_H

namespace TASK_STATES {
  const uint8_t TASK_BEGIN = 1;
  const uint8_t PRE_TASK_PAUSE = 2;
  const uint8_t PRE_TASK_SUSPEND = 3;
  const uint8_t WHITE_CALIBRATION = 4;
  const uint8_t FIND_BLACK_LINE = 5;
  const uint8_t FOLLOWING_LINE = 6;
  const uint8_t NO_LINE = 7;
  const uint8_t WRITE_TO_SERIAL = 8;
  const uint8_t STOP = 9;
}

#endif