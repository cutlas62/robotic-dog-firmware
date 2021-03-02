#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "leg.h"


// Test legs on the right side
Leg rfLeg (FRONT_RIGHT, 8, -10); // Right-front leg
//Leg rbLeg (REAR_RIGHT, 0, 0); // Right-back leg

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
    Serial.begin(115200);

    pwm.begin();
    pwm.setPWMFreq(60);

    Serial.println("S");
}


void loop() {
    int y = -70;
    for (int x = -30; x <= 60; x += 1) {
        rfLeg.moveFoot(pwm, x, y);
        //rbLeg.moveFoot(pwm, x, y);
        delay(8);
    }
    for (int x = 60; x >= -30; x -= 1) {
        rfLeg.moveFoot(pwm, x, y);
        //rbLeg.moveFoot(pwm, x, y);
        delay(8);
    }
}
