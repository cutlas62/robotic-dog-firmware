#ifndef LEG_H
#define LEG_H

#include "Arduino.h"
#include "stdint.h"
#include "math.h"
#include <Adafruit_PWMServoDriver.h>


#define SERVO_MIN   150
#define SERVO_MAX   600
#define SERVO_HOME  ((SERVO_MIN + SERVO_MAX) / 2)

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
        double footX;
        double footY;
        double targetX;
        double targetY;
        double stepSize;
        enum legPosition legPos;
        uint8_t hipServoN;
        uint8_t kneeServoN;
        int hipOffset;
        int kneeOffset;

        // Constants to improve inverse kinematics computation time
        double q2_den;
        double femur_l_sq;
        double tibia_l_sq;
        double diff_femur_tibia;
        double femur_t_tibia;

        // Helper functions
        uint16_t radToServoPoints (double radIn, double minIn, double maxIn, double minOut, double maxOut);
        uint16_t constrainServoPoints (uint16_t out);

    public:

        Leg (enum legPosition pos, int hipOff, int kneeOff);
        Leg (double x, double y, enum legPosition pos, int hipOff, int kneeOff);

        int getHipOffset (void);
        void setHipOffset(int offset);
        int getKneeOffset (void);
        void setKneeOffset(int offset);
        void setTargetCoor (double x, double y);
        void setStepSize (double s);

        void moveFoot (Adafruit_PWMServoDriver *pwm, double x, double y);
        //void fakeMoveFoot (double xCoor, double yCoor);

        
        uint8_t update (Adafruit_PWMServoDriver *pwm);

        void homeLeg (Adafruit_PWMServoDriver *pwm);
};

#endif
