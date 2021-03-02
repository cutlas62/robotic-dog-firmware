#ifndef LEG_H
#define LEG_H

#include "Arduino.h"
#include "stdint.h"
#include "math.h"
#include <Adafruit_PWMServoDriver.h>


#define SERVOMIN    150
#define SERVOMAX    600

#define FEMUR_LENGTH    46.0
#define TIBIA_LENGTH    48.0

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
        double femurLength;
        double tibiaLength;
        enum legPosition legPos;
        uint8_t hipServoN;
        uint8_t kneeServoN;
        int hipOffset;
        int kneeOffset;

        uint16_t radToServoPoints (double radIn, double minIn, double maxIn, double minOut, double maxOut);
        uint16_t constrainServoPoints (uint16_t out);

    public:

        Leg (enum legPosition pos, int hipOff, int kneeOff);
        Leg (int x, int y, enum legPosition pos, int hipOff, int kneeOff);

        int getHipOffset (void);
        void setHipOffset(int offset);
        int getKneeOffset (void);
        void setKneeOffset(int offset);

        void moveFoot (Adafruit_PWMServoDriver pwm, int x, int y);
        void fakeMoveFoot (double xCoor, double yCoor);
};

#endif
