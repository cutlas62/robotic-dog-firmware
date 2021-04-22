#include "leg.h"

const uint8_t servoNumbers [4][2] = {{9, 8}, {14, 15}, {6, 7}, {1, 0}};

/*******************************************
    Constructors
 ******************************************/
Leg::Leg (enum legPosition pos, int hipOff, int kneeOff) {
    footX = TIBIA_LENGTH;
    footY = -FEMUR_LENGTH;
    targetX = TIBIA_LENGTH;
    targetY = -FEMUR_LENGTH;
    stepSize = 1;
    legPos = pos;
    hipServoN = servoNumbers[pos][0];
    kneeServoN = servoNumbers[pos][1];
    hipOffset = hipOff;
    kneeOffset = kneeOff;

    q2_den = 2 * FEMUR_LENGTH * TIBIA_LENGTH;
    femur_l_sq = FEMUR_LENGTH * FEMUR_LENGTH;
    tibia_l_sq = TIBIA_LENGTH * TIBIA_LENGTH;
    diff_femur_tibia = femur_l_sq + tibia_l_sq;
}

Leg::Leg (double x, double y, enum legPosition pos, int hipOff, int kneeOff) {
    footX = x;
    footY = y;
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

void Leg::setTargetCoor(double x, double y) {
    targetX = x;
    targetY = y;
}

void Leg::setStepSize(double s) {
    stepSize = s;
}


/*******************************************
    Actuator Functions
 ******************************************/
uint8_t Leg::update (Adafruit_PWMServoDriver *pwm) {
    double diff;
    uint8_t ret = 0;

    // Check for x
    diff = footX - targetX;
    if (diff > 0) {
        footX -= (diff > stepSize) ? stepSize : diff;
        ret |= 0x01;
    } else if (diff < 0) {
        footX += (abs(diff) > stepSize) ? stepSize : abs(diff);
        ret |= 0x02;
    }

    // Check for y
    diff = footY - targetY;
    if (diff > 0) {
        footY -= (diff > stepSize) ? stepSize : diff;
        ret |= 0x04;
    } else if (diff < 0) {
        footY += (abs(diff) > stepSize) ? stepSize : abs(diff);
        ret |= 0x08;
    }

    // Move the foot to the next position
    if (ret != 0) {
        moveFoot(pwm, footX, footY);
    }
    return ret;
}


void Leg::moveFoot (Adafruit_PWMServoDriver *pwm, double x, double y) {
    double q2 = acos((x * x + y * y - diff_femur_tibia) / q2_den);
    double q1 = atan2(y, x) - atan2((TIBIA_LENGTH * sin(q2)), (FEMUR_LENGTH + TIBIA_LENGTH * cos(q2)));

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
