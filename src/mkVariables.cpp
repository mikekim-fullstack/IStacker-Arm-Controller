#include <Arduino.h>
#include "../src/mkZoeRobotics_globalDataStruct.h"
#include "../src/mkZoeRobotics_define.h"

POSData posData[MAX_MOTOR_NUM]; // [X, R1, R2, Z]
SPEEDRampData speedData[MAX_MOTOR_NUM];
KIN_DATA kinData[MAX_MOTOR_NUM];
MOTORCHANEL motorCh[MAX_MOTOR_NUM];