#ifndef __MOTOR_H
#define __MOTOR_H

#define FORWARD_DIRECTION 0
#define REVERSE_DIRECTION 1
#define RPS_TO_PWM_FACTOR (20 / 1.83) // Measured. Assume this is true
// #define RPS_TO_PWM_FACTOR (50 / 1.83) // This is test
#define INTEGRAL_ERROR_THRESHOLD (255 / RPS_TO_PWM_FACTOR)

#define SLOW_RPS 2.5
#define MEDIUM_RPS 2
#define FAST_RPS 3
#define SUPER_FAST_RPS 4
#define BRR 4


class Motor {
  private:
  int directionPin, pwmPin;
  
  public:
  int currentDirection, currentSpeed;
  float currentRps, integralError, previousError, speedP = 0.7, speedI = 0.01, speedD = 0.3;
  static float deltaRps, targetRps;

  Motor(int directionPin, int pwmPin) {
    this->directionPin = directionPin;
    this->pwmPin = pwmPin;
    pinMode(this->directionPin, OUTPUT);
    pinMode(this->pwmPin, OUTPUT);
  }

  void setDirection(int direction) {
    digitalWrite(this->directionPin, direction);
    this->currentDirection = direction;
  }

  void setSpeed(int pwmSpeed) {
    if (pwmSpeed < 0) {
      this->setDirection(REVERSE_DIRECTION);
      pwmSpeed = -pwmSpeed;
    }
    else {
      this->setDirection(FORWARD_DIRECTION);
    }
    this->currentSpeed = pwmSpeed;
    if (this->currentSpeed > 50) {
      this->currentSpeed = 50;
    }
    else if ( this->currentSpeed < 0) {
      this->currentSpeed = 0;
    }
    analogWrite(this->pwmPin, pwmSpeed);
  }

  void stop() {
    this->setSpeed(0);
  }

  void correctSpeed(float desiredRps, int timePassed) {

    float error = (desiredRps - this->currentRps);
    this->integralError += error;
    if (this->integralError > INTEGRAL_ERROR_THRESHOLD || this->integralError < -INTEGRAL_ERROR_THRESHOLD) {
      this->integralError = 0;
    }
    // Correct using PID
    this->currentRps += (error * this->speedP) + this->integralError * this->speedI + ((this->previousError - error) / timePassed) * this->speedD;
    this->previousError = error;
    this->setSpeed(this->currentRps * RPS_TO_PWM_FACTOR);
  }

};

float Motor::deltaRps = 0;
float Motor::targetRps = 0;

#endif