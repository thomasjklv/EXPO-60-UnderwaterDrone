#ifndef CONTROL_H
#define CONTROL_H
#include "telemetry.h"

typedef enum{
    IDLE = 0,
    ATTACK = 1,
    RESURFACE = 2
}control_STATES;


void setAttitudeACT(drone_MAIN *drone);

#endif