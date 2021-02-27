#ifndef LEG_H
#define LEG_H

#include "stdint.h"

#define SERVOMIN    150
#define SERVOMAX    600

class Leg {
    private:
        // All dimensions are in mm
        int footX;
        int footY;
        int femurLength;
        int tibiaLength;
        bool isRight;	// 0 for left side legs, 1 for right side legs

        uint16_t radToServoPoints (double radIn, double minIn, double maxIn, double minOut, double maxOut);
        uint16_t constrainServoPoints (uint16_t out);

    public:

        Leg (bool rightSide, int fLength, int tLength) {
            footX = 0;
            footY = 0;
            femurLength = fLength;
            tibiaLength = tLength;
            isRight = rightSide;
        }

        Leg (int x, int y, bool rightSide, int fLength, int tLength) {
            footX = x;
            footY = y;
            femurLength = fLength;
            tibiaLength = tLength;
            isRight = rightSide;
        }

        void moveFoot (int xCoor, int yCoor);
        void fakeMoveFoot (double xCoor, double yCoor);
};

#endif
