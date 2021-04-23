#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "leg.h"

#define INPUT_BUF_SIZE      50
#define MAX_INPUT_COMMANDS  5

Leg frLeg (FRONT_RIGHT, 8, -6);
Leg flLeg (FRONT_LEFT, 10, -10);
Leg rrLeg (REAR_RIGHT, -8, -13);
Leg rlLeg (REAR_LEFT, 3, -8);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/****************************************
    Function prototypes
 ****************************************/
void moveServos (void);
void moveFoot (void);
void squareTrajectory (void);
void bezierTrajectory (void);
void crawlGait (void);          // TBD
void walkGait (void);           // TBD
void runGait (void);            // TBD

void printHelp (void);
void homeAllServos (void);
uint32_t fac (uint32_t n);              // Factorial of a number
uint32_t bin (uint32_t n, uint32_t i);  // Newton's binomium

/****************************************
    Commands
 ****************************************/
typedef struct Cmd {
    char cmd[5];
    void (*funcptr)(void);
};
Cmd cmdMatrix [] {
    {"1\0", moveServos},
    {"2\0", moveFoot},
    {"3\0", squareTrajectory},
    {"4\0", bezierTrajectory},
    {"5\0", crawlGait},
    {"6\0", walkGait},
    {"7\0", runGait},
    {"help\0", printHelp},
    {"home\0", homeAllServos},
};

uint8_t nCmd = sizeof(cmdMatrix) / sizeof(Cmd);


/****************************************
    Setup
 ****************************************/
void setup() {
    Serial.begin(115200);
    Serial.setTimeout(10);

    pwm.begin();
    pwm.setPWMFreq(60);

    Serial.println(F("Starting now"));
    printHelp();
}

double points [][2] = {
    {TIBIA_LENGTH, -FEMUR_LENGTH},
    {TIBIA_LENGTH, 0},
    {TIBIA_LENGTH, -FEMUR_LENGTH},
    {TIBIA_LENGTH, -FEMUR_LENGTH - 20},
};
uint8_t nPoints = sizeof(points) / sizeof(points[0]);

/****************************************
    Loop
 ****************************************/
void loop() {
    checkSerialPort();
}


/****************************************
    Action functions
 ****************************************/
void moveServos (void) {
    Serial.println(F("moveServos"));

    uint16_t dc;
    uint16_t servo_min = 300;
    uint16_t servo_max = 500;
    for (uint8_t i = 0; i < 2; i++) {
        // Move the hip
        for (dc = SERVO_HOME; dc < servo_max; dc++) {
            pwm.setPWM(9, 0, dc);
            delay(5);
        }
        for (dc = servo_max; dc > servo_min; dc--) {
            pwm.setPWM(9, 0, dc);
            delay(5);
        }
        for (dc = servo_min; dc <= SERVO_HOME; dc++) {
            pwm.setPWM(9, 0, dc);
            delay(5);
        }

        // Move the knee
        for (dc = SERVO_HOME; dc < servo_max; dc++) {
            pwm.setPWM(8, 0, dc);
            delay(5);
        }
        for (dc = servo_max; dc > servo_min; dc--) {
            pwm.setPWM(8, 0, dc);
            delay(5);
        }
        for (dc = servo_min; dc <= SERVO_HOME; dc++) {
            pwm.setPWM(8, 0, dc);
            delay(5);
        }

        // Move both at the same time
        for (dc = SERVO_HOME; dc < (servo_max - 50); dc++) {
            pwm.setPWM(8, 0, ((servo_max - 50) - dc) + servo_min);
            pwm.setPWM(9, 0, dc);
            delay(5);
        }
        for (dc = (servo_max - 50); dc > servo_min; dc--) {
            pwm.setPWM(8, 0, ((servo_max - 50) - dc) + servo_min);
            pwm.setPWM(9, 0, dc);
            delay(5);
        }
        for (dc = servo_min; dc <= SERVO_HOME; dc++) {
            pwm.setPWM(8, 0, ((servo_max - 50) - dc) + servo_min);
            pwm.setPWM(9, 0, dc);
            delay(5);
        }
    }
}

void moveFoot (void) {
    Serial.println(F("moveFoot"));
    double x;
    double y;

    double points [9][2] = {
        {20, -60},
        {60, -60},
        {0, -60},
        {20, -60},
        {20, -85},
        {20, -25},
        {20, -85},
        {20, -60},
        {TIBIA_LENGTH, -FEMUR_LENGTH}
    };

    x = TIBIA_LENGTH;
    y = -FEMUR_LENGTH;

    for (uint8_t i = 0; i < 9; i++) {
        double targetX = points[i][0];
        double targetY = points[i][1];

        while (x != targetX || y != targetY) {
            // Update x
            if (x > targetX) {
                x--;
            } else if (x < targetX) {
                x++;
            }

            // Update y
            if (y > targetY) {
                y--;
            } else if (y < targetY) {
                y++;
            }

            // Move foot
            frLeg.moveFoot(&pwm, x, y);
            delay(20);
        }
    }
}

void squareTrajectory (void) {
    Serial.println(F("squareTrajectory"));
    double x;
    double y;

    double points [9][2] = {
        { -10, -50},
        { 50, -50},
        { 50, -80},
        { -10, -80},
        { -10, -50},
        { 50, -50},
        { 50, -80},
        { -10, -80},
        {TIBIA_LENGTH, -FEMUR_LENGTH}
    };

    x = TIBIA_LENGTH;
    y = -FEMUR_LENGTH;

    for (uint8_t i = 0; i < 9; i++) {
        double targetX = points[i][0];
        double targetY = points[i][1];

        while (x != targetX || y != targetY) {
            // Update x
            if (x > targetX) {
                x--;
            } else if (x < targetX) {
                x++;
            }

            // Update y
            if (y > targetY) {
                y--;
            } else if (y < targetY) {
                y++;
            }

            // Move foot
            frLeg.moveFoot(&pwm, x, y);
            delay(20);
        }

    }


}

void bezierTrajectory (void) {
    Serial.println(F("bezierTrajectory"));
    double x;
    double y;

    double points [][2] = {
        {20, -45},
        {80, -2},
        {80, -100},
        { -40, -100},
        { -40, -60},
        {20, -45},
    };

    // Slowly go back to first position
    x = TIBIA_LENGTH;
    y = -FEMUR_LENGTH;
    while (x != points[0][0] || y != points[0][1]) {
        // Update x
        if (x > points[0][0]) {
            x--;
        } else if (x < points[0][0]) {
            x++;
        }

        // Update y
        if (y > points[0][1]) {
            y--;
        } else if (y < points[0][1]) {
            y++;
        }

        // Move foot
        frLeg.moveFoot(&pwm, x, y);
        delay(15);
    }

    // Actual Bezier curve
    uint8_t nPoints = sizeof(points) / sizeof(points[0]);
    double n = nPoints - 1;
    for (uint8_t j = 0; j < 3; j++) {
        for (double t = 0; t <= 1; t += 0.005) {
            x = 0;
            y = 0;

            for (uint8_t i = 0; i < nPoints; i++) {
                x += bin(n, i) * pow(1 - t, n - i) * pow(t, i) * points[i][0];
                y += bin(n, i) * pow(1 - t, n - i) * pow(t, i) * points[i][1];
            }

            frLeg.moveFoot(&pwm, x, y);
            delay(5);
        }
    }

    // Slowly go back to home position
    x = points[0][0];
    y = points[0][1];
    while (x != TIBIA_LENGTH || y != -FEMUR_LENGTH) {
        // Update x
        if (x > TIBIA_LENGTH) {
            x--;
        } else if (x < TIBIA_LENGTH) {
            x++;
        }

        // Update y
        if (y > -FEMUR_LENGTH) {
            y--;
        } else if (y < -FEMUR_LENGTH) {
            y++;
        }

        // Move foot
        frLeg.moveFoot(&pwm, x, y);
        delay(15);
    }
    frLeg.moveFoot(&pwm, TIBIA_LENGTH, -FEMUR_LENGTH);
}

void crawlGait (void) {
    Serial.println(F("crawlGait"));
}

void walkGait (void) {
    Serial.println(F("walkGait"));
}

void runGait (void) {
    Serial.println(F("runGait"));
}


/****************************************
    Serial Port utilities
 ****************************************/
void checkSerialPort(void) {
    if (Serial.available() > 0) {
        // Read the incoming string and null-terminate it
        char inputBuf [INPUT_BUF_SIZE + 1];
        byte inNBytes = Serial.readBytes(inputBuf, INPUT_BUF_SIZE);
        inputBuf[inNBytes - 1] = 0;

        // Parse the incoming string
        int argc;
        char* argv [MAX_INPUT_COMMANDS];
        parseInputBuf(inputBuf, &argc, argv);

        // Decode the incoming command
        decodeInputCmd(argc, argv);
    }
}

void parseInputBuf (char* inBuf, int* argc, char* argv []) {
    *argc = 0;
    uint8_t lastTok = 0;
    uint8_t i = 0;
    while (inBuf[i] != NULL) {
        if (inBuf[i] == ' ') {
            argv[*argc] = &inBuf[lastTok];
            (*argc)++;
            lastTok = i + 1;
            inBuf[i] = NULL;

            if (*argc == MAX_INPUT_COMMANDS) {
                // Return here to avoid buffer overflow in argv
                return;
            }
        }
        i++;
    }

    // Check that the input wasn't empty
    if (i > 0) {
        argv[*argc] = &inBuf[lastTok];
        (*argc)++;
    }
}

int decodeInputCmd (int argc, char* argv []) {
    if (argc < 1) {
        return -1;
    }

    Serial.print(argv[0]);
    Serial.print(" -> ");
    uint8_t i;
    for (i = 0; i < nCmd; i++) {
        if (strcmp(argv[0], cmdMatrix[i].cmd) == 0) {
            (cmdMatrix[i].funcptr)();
            break;
        }
    }
    if (i == nCmd) {
        Serial.println(F("Command not found"));
    }
}

void printHelp (void) {
    Serial.println(F("******************"));
    Serial.println(F("*  1 - moveServos"));
    Serial.println(F("*  2 - moveFoot"));
    Serial.println(F("*  3 - squareTrajectory"));
    Serial.println(F("*  4 - bezierTrajectory"));
    Serial.println(F("*  5 - crawlGait"));
    Serial.println(F("*  6 - walkGait"));
    Serial.println(F("*  7 - runGait"));
    Serial.println(F("******************"));
}

/****************************************
    Helper functions
 ****************************************/
void homeAllServos() {
    Serial.println(F("Homing all the servos..."));
    frLeg.homeLeg(&pwm);
    flLeg.homeLeg(&pwm);
    rrLeg.homeLeg(&pwm);
    rlLeg.homeLeg(&pwm);
}

uint32_t fac (uint32_t n) {
    uint32_t ret = 1;
    while (n > 0) {
        ret *= n;
        n--;
    }
    return ret;
}

uint32_t bin (uint32_t n, uint32_t i) {
    return (fac(n) / (fac(i) * fac(n - i)));
}
