#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "leg.h"

#define INPUT_BUF_SIZE  50

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
        delay(8);
    }
    for (int x = 60; x >= -30; x -= 1) {
        frLeg.moveFoot(pwm, x, y);
        delay(8);
    }

    delay(100);
}

void checkSerialPort(void) {
    if (Serial.available() > 0) {
        // Read the incoming string and null-terminate it
        char inputBuf [INPUT_BUF_SIZE + 1];
        byte inNBytes = Serial.readBytes(inputBuf, INPUT_BUF_SIZE);
        inputBuf[inNBytes] = 0;

        // Parse the incoming string
        int argc;
        char* argv [5];
        parseInputBuf(inputBuf, &argc, argv);

        Serial.print("argc = ");
        Serial.println(argc);
        for (int i = 0; i < argc; i++) {
            Serial.print(i);
            Serial.print(" -> ");
            Serial.println(argv[i]);
        }

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
        }
        i++;
    }

    // Check that the input wasn't empty
    if (i > 1) {
        argv[*argc] = &inBuf[lastTok];
        (*argc)++;
    }
}
