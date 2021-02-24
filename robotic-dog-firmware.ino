#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)
#define HOME ((SERVOMIN + SERVOMAX) / 2)

int range = SERVOMAX - SERVOMIN;
// our servo # counter
uint8_t servonum = 0;
uint16_t duty = HOME;

void shutServos() {
  delay(250);
  for (byte i = 0; i < 16; i++) {
    pwm.setPWM(i, 0, 4096);
  }
}

void homeServos() {
  Serial.println(F("Homing all the servos..."));
  delay(250);
  for (byte i = 0; i < 16; i++) {
    pwm.setPWM(i, 0, HOME);
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
    servonum = 0;
    duty = 0;
    uint8_t state = 0;

    while (Serial.available() > 0) {
      char inChar = Serial.read();

      if (inChar == 'h') {
        homeServos();
      } else if (inChar == ' ') {
        state++;
      } else if ('0' <= inChar && inChar <= '9') {
        if (state == 0) {
          servonum *= 10;
          servonum += inChar - '0';
        } else {
          duty *= 10;
          duty += inChar - '0';
        }
      }
    }

    if (servonum < 0 || servonum > 15) {
      Serial.print(F("Bad servonum: "));
      Serial.println(servonum);
      servonum = 0;
      duty = HOME;
    }
    if (duty < SERVOMIN || duty > SERVOMAX) {
      Serial.print(F("Bad duty: "));
      Serial.println(duty);
      servonum = 0;
      duty = HOME;
    }

    // Last line of defense
    if ((0 <= servonum) && (servonum <= 15) && (SERVOMIN <= duty) && (duty <= SERVOMAX)) {
      char buf [50];
      sprintf(buf, "Setting servo #%u to %u\n", servonum, duty);
      Serial.print(buf);
      pwm.setPWM(servonum, 0, duty);
    }
  }

  delay(100);
}
