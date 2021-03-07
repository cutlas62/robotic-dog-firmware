#include "leg.h"

const uint8_t servoNumbers [4][2] = {{9, 8}, {14, 15}, {6, 7}, {1, 0}};


/*******************************************
    Constructors
 ******************************************/
Leg::Leg (enum legPosition pos, int hipOff, int kneeOff) {
    footX = 0;
    footY = 0;
    femurLength = FEMUR_LENGTH;
    tibiaLength = TIBIA_LENGTH;
    legPos = pos;
    hipServoN = servoNumbers[pos][0];
    kneeServoN = servoNumbers[pos][1];
    hipOffset = hipOff;
    kneeOffset = kneeOff;
}

Leg::Leg (int x, int y, enum legPosition pos, int hipOff, int kneeOff) {
    footX = x;
    footY = y;
    femurLength = FEMUR_LENGTH;
    tibiaLength = TIBIA_LENGTH;
    legPos = pos;
    hipServoN = servoNumbers[pos][0];
    kneeServoN = servoNumbers[pos][1];
    hipOffset = hipOff;
    kneeOffset = kneeOff;
}



/*******************************************
    Setters and Getters
 ******************************************/
int Leg::getHipOffset (void) {
    return hipOffset;
}

void Leg::setHipOffset(int offset) {
    hipOffset = offset;
}

int Leg::getKneeOffset (void) {
    return kneeOffset;
}

void Leg::setKneeOffset(int offset) {
    kneeOffset = offset;
}



/*******************************************
    Actuator Functions
 ******************************************/
void Leg::moveFoot (Adafruit_PWMServoDriver *pwm, double x, double y) {
    double a = femurLength;
    double b = tibiaLength;
    double q2 = acos((x * x + y * y - a * a - b * b) / (2 * a * b));
    double q1 = atan2(y, x) - atan2((b * sin(q2)), (a + b * cos(q2)));

    if (isnan(q1) || isnan(q2)) {
        return;
    }

    // Invert the X axis for the left side
    if ((legPos == FRONT_LEFT) || (legPos == REAR_LEFT)) {
        q1 = -PI - q1;
        q2 = PI - q2;
    }

    uint16_t _q1 = radToServoPoints(q1, -PI, 0, 217, 534);
    uint16_t _q2 = radToServoPoints(PI - q2, 0, PI, 217, 534);

    _q1 = constrainServoPoints(_q1 + hipOffset);
    _q2 = constrainServoPoints(_q2 + kneeOffset);

    pwm->setPWM(hipServoN, 0, _q1);
    pwm->setPWM(kneeServoN, 0, _q2);

    footX = x;
    footY = y;

}

// Same as moveFoot but just print the joint values
// instead of moving the feet, for testing purposes
/*
void Leg::fakeMoveFoot (double x, double y) {
    double a = femurLength;
    double b = tibiaLength;
    double q2 = acos((x * x + y * y - a * a - b * b) / (2 * a * b));
    double q1 = atan2(y, x) - atan2((b * sin(q2)), (a + b * cos(q2)));
    Serial.print(F("[x,y,q1,q2] = ["));
    Serial.print(x);
    Serial.print(F(","));
    Serial.print(y);
    Serial.print(F(","));
    Serial.print(q1);
    Serial.print(F(","));
    Serial.print(q2);
    Serial.println(F("]"));

    if (isnan(q1) || isnan(q2)) {
        return;
    }

    uint16_t _q1 = constrainServoPoints(radToServoPoints(q1, -PI, 0, 217, 534));
    uint16_t _q2 = constrainServoPoints(radToServoPoints(PI - q2, 0, PI, 217, 534));
    _q1 += hipOffset;
    _q2 += kneeOffset;
    Serial.print(F("_q1 = "));
    Serial.print(_q1);
    Serial.print(F(", _q2 = "));
    Serial.println(_q2);

    footX = x;
    footY = y;
}
*/

void Leg::homeLeg (Adafruit_PWMServoDriver *pwm) {
    pwm->setPWM(hipServoN, 0, SERVO_HOME + hipOffset);
    pwm->setPWM(kneeServoN, 0, SERVO_HOME + kneeOffset);
}

uint16_t Leg::radToServoPoints (double radIn, double minIn, double maxIn, double minOut, double maxOut) {
    return ((radIn - minIn) * (maxOut - minOut) / (maxIn - minIn) + minOut);
}

uint16_t Leg::constrainServoPoints (uint16_t out) {
    if (out < SERVO_MIN) {
        return SERVO_MIN;
    }
    if (out > SERVO_MAX) {
        return SERVO_MAX;
    }
    return out;
}
