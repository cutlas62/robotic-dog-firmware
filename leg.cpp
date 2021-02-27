#include "leg.h"
#include <math.h>
#include <Arduino.h>

void Leg::moveFoot (int xCoor, int yCoor) {

}

// Same as moveFoot but just print the joint values
// instead of moving the feet, for testing purposes
void Leg::fakeMoveFoot (double x, double y) {
    double a = femurLength;
    double b = tibiaLength;
    double q2 = acos((x * x + y * y - a * a - b * b) / (2 * a * b));
    double q1 = atan2(y, x) - atan2((b * sin(q2)), (a + b * cos(q2)));
    Serial.print("[x,y,q1,q2] = [");
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.print(",");
    Serial.print(q1);
    Serial.print(",");
    Serial.print(q2);
    Serial.println("]");

    if (isnan(q1) || isnan(q2)){
        return;
    }

    uint16_t _q1 = constrainServoPoints(radToServoPoints(q1, -PI, 0, 217, 534));
    uint16_t _q2 = constrainServoPoints(radToServoPoints(q2, -PI/2, PI/2, 217, 534));
    Serial.print("_q1 = ");
    Serial.print(_q1);
    Serial.print(", ");
    Serial.print("_q2 = ");
    Serial.println(_q2);
}

uint16_t Leg::radToServoPoints (double radIn, double minIn, double maxIn, double minOut, double maxOut) {
    return ((radIn - minIn) * (maxOut - minOut) / (maxIn - minIn) + minOut);
}

uint16_t Leg::constrainServoPoints (uint16_t out) {
    if (out < SERVOMIN) {
        return SERVOMIN;
    }
    if (out > SERVOMAX) {
        return SERVOMAX;
    }
    return out;
}
