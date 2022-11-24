#ifndef _ENCODER_H
#define _ENCODER_H


#define ENC_COUNTS_PER_ROTATION 358.3 * 2
// Pi is not included in either because it cancels out
#define BOT_CIRCUMFERENCE (90)
#define WHEEL_CIRCUMFERENCE (32)

/*
All this is in a special file because the compiler optimizes out "ISR ( PCINT0_vec )".
Turns out, if you put it in a separate file, it does not optimize the function out.
Once again, we bend the knee to the mysteries of the compiler.
*/

namespace ENC {
  volatile struct {
    int lastTime;
    int rightMotorCounter = 0;
    int leftMotorCounter = 0;
  } ENC_DATA;

  // Remember this is not generic. It changes with the AtMega used, but the process may be considered generic.
  void setupPinChangeEncoder() {
    // Disable interrupts
    PCICR &= ~( 1 << 0 );
    // 1. Enable the pin change interrupt (our pin is PCINT4 which is PB4)
    PCICR |= 0b00000001; // Page 91 in datasheet
    // 2. Enable mask for PCINT4
    PCMSK0 |= 0b00010000; // Page 91 in datasheet
    // 3. Clear interrupt flag
    PCIFR |= (1 << 0);
    // attachInterrupt(digitalPinToInterrupt(leftMotor.encoder->encoderXorPin), leftMotorInterrupt, FALLING);
    PCICR |= (1 << PCIE0);
  }

  void rightMotorInterrupt() {
    ENC_DATA.rightMotorCounter++;
  }

  void resetCounters() {
    ENC_DATA.rightMotorCounter = 0;
    ENC_DATA.leftMotorCounter = 0;
  }

}

// void leftMotorInterrupt() {
//   leftMotorCounter++;
// }
// This is basically the same as leftMotorInterrupt() but special because PCINT4 is a pin change interrupt
ISR( PCINT0_vect ) {
  ENC::ENC_DATA.leftMotorCounter++;
}


#endif