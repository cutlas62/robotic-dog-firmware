#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "leg.h"


// Test leg on the right side
Leg rLeg (true, 46, 48);
// Test leg on the left side
Leg lLeg (false, 46, 48);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
    Serial.begin(115200);
    Serial.println("S");
    
}


void loop() {
    int y = -70;
    for (int x = -30; x <= 60; x += 1) {
        rLeg.moveFoot(pwm, x, y);
        delay(8);
    }
    for (int x = 60; x >= -30; x -= 1) {
        rLeg.moveFoot(pwm, x, y);
        delay(8);
    }
}
