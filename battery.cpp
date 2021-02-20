#include <stdint.h>
#include "battery.hpp"
#include "wiring_private.h"
#include "pins_arduino.h"

#define BATTERY_PIN       0   // A0
#define BATTERY_THRESHOLD 300

static uint16_t getBatteryLevel(void) {
  uint8_t low, high;
  
  // set the analog reference (high two bits of ADMUX) and select the
  // channel (low 4 bits).  this also sets ADLAR (left-adjust result)
  // to 0 (the default).
  ADMUX = (DEFAULT << 6) | (BATTERY_PIN & 0x07);
 
  // start the conversion
  sbi(ADCSRA, ADSC);
 
  // ADSC is cleared when the conversion finishes
  while (bit_is_set(ADCSRA, ADSC));
 
  // we have to read ADCL first; doing so locks both ADCL
  // and ADCH until ADCH is read.  reading ADCL second would
  // cause the results of each conversion to be discarded,
  // as ADCL and ADCH would be locked when it completed.
  low  = ADCL;
  high = ADCH;
 
  // combine the two bytes
  return (high << 8) | low;
}


bool isBatteryConnected (void) {
  return (getBatteryLevel() > BATTERY_THRESHOLD);
}
