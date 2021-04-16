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
 *  Function prototypes
 ****************************************/
void moveServos (void);
void moveFoot (void);
void squareTrajectory (void);
void bezierTrajectory (void);
void crawlGait (void);
void walkGait (void);
void runGait (void);

/****************************************
 *  Commands
 ****************************************/
typedef struct Cmd {
    char cmd[2];
    void (*funcptr)(void);
};
Cmd cmdMatrix []{
    {"1\0", moveServos},
    {"2\0", moveFoot},
    {"3\0", squareTrajectory},
    {"4\0", bezierTrajectory},
    {"5\0", crawlGait},
    {"6\0", walkGait},
    {"7\0", runGait},
};

uint8_t nCmd = sizeof(cmdMatrix)/sizeof(Cmd);

/****************************************  
 *  Setup
 ****************************************/
void setup() {
    Serial.begin(115200);
    Serial.setTimeout(10);

    pwm.begin();
    pwm.setPWMFreq(60);

    Serial.println(F("Starting now"));
}

/****************************************  
 *  Loop
 ****************************************/
void loop() {
    checkSerialPort();
}

/****************************************  
 *  Action functions
 ****************************************/
 // TBD
void moveServos (void){
    Serial.println("moveServos");
}

// TBD
void moveFoot (void){
    Serial.println("moveFoot");
}

// TBD
void squareTrajectory (void){
    Serial.println("squareTrajectory");
}

// TBD
void bezierTrajectory (void){
    Serial.println("bezierTrajectory");
}

// TBD
void crawlGait (void){
    Serial.println("crawlGait");
}

// TBD
void walkGait (void){
    Serial.println("walkGait");
}

// TBD
void runGait (void){
    Serial.println("runGait");
}


/****************************************  
 *  Serial Port utilities
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
    
    for (int i = 0; i < nCmd; i++){
        if (strcmp(argv[0], cmdMatrix[i].cmd) == 0){
            (cmdMatrix[i].funcptr)();
        }
    }
}

void homeAllServos() {
    Serial.println(F("Homing all the servos..."));
    frLeg.homeLeg(&pwm);
    flLeg.homeLeg(&pwm);
    rrLeg.homeLeg(&pwm);
    rlLeg.homeLeg(&pwm);
}

void printHelp (void) {
    Serial.println(F("WIP"));
}
