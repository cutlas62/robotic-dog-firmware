#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "leg.h"


// Test legs on the right side
Leg rfLeg (true, 46, 48, 9, 8); // Right-front leg
Leg rbLeg (true, 46, 48, 6, 7); // Right-back leg

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
    Serial.begin(115200);
    Serial.println("S");
    
}


void loop() {
    int y = -70;
    for (int x = -30; x <= 60; x += 1) {
        rfLeg.moveFoot(pwm, x, y);
        rbLeg.moveFoot(pwm, x, y);
        delay(8);
    }
    for (int x = 60; x >= -30; x -= 1) {
        rfLeg.moveFoot(pwm, x, y);
        rbLeg.moveFoot(pwm, x, y);
        delay(8);
    }
}
