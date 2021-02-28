#ifndef LEG_H
#define LEG_H

#include "Arduino.h"
#include "stdint.h"
#include "math.h"
#include <Adafruit_PWMServoDriver.h>


#define SERVOMIN    150
#define SERVOMAX    600

enum legPosition {
    FRONT_RIGHT,
    FRONT_LEFT,
    REAR_RIGHT,
    REAR_LEFT
};

class Leg {
    private:
        // All dimensions are in mm
        int footX;
        int footY;
        int femurLength;
        int tibiaLength;
        enum legPosition legPos;
        uint8_t hipServoN;
        uint8_t kneeServoN;

        uint16_t radToServoPoints (double radIn, double minIn, double maxIn, double minOut, double maxOut);
        uint16_t constrainServoPoints (uint16_t out);

    public:

        Leg (enum legPosition pos, int fLength, int tLength, uint8_t hipN, uint8_t kneeN);
        Leg (int x, int y, enum legPosition pos, int fLength, int tLength, uint8_t hipN, uint8_t kneeN);

        void moveFoot (Adafruit_PWMServoDriver pwm, int x, int y);
        void fakeMoveFoot (double xCoor, double yCoor);
};

#endif
