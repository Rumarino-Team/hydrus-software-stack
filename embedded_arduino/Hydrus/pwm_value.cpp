#include "devices.h"

int getPWMValue(int data, bool isForward) {
  int value = 0;
    
  if (data <= -5 || data >= 5) return 0; //Sensible values please

  if (data == 0) {
    value = PWM_NEUTRAL;
  } else if (data < 0) { //If we want to go backwards
    if (isForward) {
      /*Since motor goes forward and we want to go backwards, if data == -1, then
      just write PWM_BACKWARDS (-1 + 1 = 0). If data < 1, then substract the PWM_VALUE to that
      based on the data inserted. For example, -1.5 will sum 1400 + (-0.5)*50,
      which equals 1375. */
      value = PWM_BACKWARDS + (1+data)*50;
    } else {
      /*Same idea but swapped since the motor moves backwards by default.
      It is substracted since we are dealing with negative numbers*/
      value = PWM_FORWARD - (1+data)*50;
    }
  } else { //We want to go forward
    if (isForward) {
      value = PWM_FORWARD + -1 * (1-data)*50;
    } else {
      value = PWM_BACKWARDS + (1-data)*50;
    }
  }

  return value;
}