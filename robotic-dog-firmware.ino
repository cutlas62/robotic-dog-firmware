#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

int range = SERVOMAX - SERVOMIN;
// our servo # counter
uint8_t servonum = 0;

void shutServos() {
  delay(250);
  for (byte i = 0; i < 16; i++) {
    pwm.setPWM(i, 0, 4096);
  }
}


void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(60);

  Serial.println("S");
}


void loop() {
  // Drive each servo one at a time
  if (Serial.available()) {
    int num = 0;
    while (Serial.available()) {
      int r = Serial.read() - '0';
      if (0 <= r && r <= 9) {
        num *= 10;
        num += r;
      }
    }
    Serial.println(num);
    for (int servonum = 0; servonum < 16; servonum++) {
      pwm.setPWM(servonum, 0, num);
    }

  }


  delay(100);

}
