#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "leg.h"

#define INPUT_BUF_SIZE      50
#define MAX_INPUT_COMMANDS  5

Leg frLeg (FRONT_RIGHT, 8, -10);
Leg flLeg (FRONT_LEFT, 0, 0);
Leg rrLeg (REAR_RIGHT, 0, 0);
Leg rlLeg (REAR_LEFT, 0, 0);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(10);

    pwm.begin();
    pwm.setPWMFreq(60);

    Serial.println("Starting now");
}


void loop() {

    checkSerialPort();

    int y = -70;
    for (int x = -30; x <= 60; x += 1) {
        frLeg.moveFoot(pwm, x, y);
        flLeg.moveFoot(pwm, x, y);
        delay(8);
    }
    for (int x = 60; x >= -30; x -= 1) {
        frLeg.moveFoot(pwm, x, y);
        flLeg.moveFoot(pwm, x, y);
        delay(8);
    }

    //delay(100);
}

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
    if (i > 1) {
        argv[*argc] = &inBuf[lastTok];
        (*argc)++;
    }

}

int decodeInputCmd (int argc, char* argv []) {
    /*
    for (int i = 0; i < argc; i++){
        Serial.print("argv[");
        Serial.print(i);
        Serial.print("] -> ");
        Serial.println(argv[i]);
    }
    */
    if (argc < 1) {
        return -1;
    }

    Leg *targetLeg;

    if (strcmp(argv[0], "help") == 0) {
        printHelp();
    } else if (strcmp(argv[0], "fr") == 0) {
        targetLeg = &frLeg;
    } else if (strcmp(argv[0], "fl") == 0) {
        targetLeg = &flLeg;
    } else if (strcmp(argv[0], "rr") == 0) {
        targetLeg = &rrLeg;
    } else if (strcmp(argv[0], "rl") == 0) {
        targetLeg = &rlLeg;
    } else {
        return -2;
    }

    if (strcmp(argv[1], "get") == 0) {
        // Check that there are at least 1 more argument
        if (argc < 3) {
            return -3;
        }
        if (strcmp(argv[2], "kneeOffset") == 0) {
            Serial.println(targetLeg->getKneeOffset());
        } else if (strcmp(argv[2], "hipOffset") == 0) {
            Serial.println(targetLeg->getHipOffset());
        } else {
            return -5;
        }

    } else if (strcmp(argv[1], "set") == 0) {
        // Check that there are at least 2 more arguments
        if (argc < 4) {
            return -4;
        }

        // Get the target value
        int targetVal = atoi(argv[3]);

        if (strcmp(argv[2], "kneeOffset") == 0) {
            targetLeg->setKneeOffset(targetVal);
            Serial.println("Ok");
        } else if (strcmp(argv[2], "hipOffset") == 0) {
            targetLeg->setHipOffset(targetVal);
            Serial.println("Ok");
        } else {
            return -6;
        }
    } else {
        return -7;
    }
}

void printHelp (void) {
    Serial.println("WIP");
}
