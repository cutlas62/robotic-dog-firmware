#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "leg.h"


// Test leg on the right side
Leg rLeg (true, 46, 48);
// Test leg on the left side
Leg lLeg (false, 46, 48);


void setup() {
    Serial.begin(115200);
    Serial.println("S");
    int y = -80;
    for (int x = -50; x <= 50; x += 10) {
        rLeg.fakeMoveFoot(x, y);
    }
}


void loop() {
    delay(100);
}
