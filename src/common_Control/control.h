#ifndef CONTROL_H
#define CONTROL_H

#include "telemetry.h"
#include "vehicle_config.h"

typedef enum
{
    IDLE = 0,
    ATTACK = 1,
    RESURFACE = 2
} control_STATES;

wrench4_t controller_compute_desired_wrench(const drone_MAIN *drone);
void control_update(drone_MAIN *drone,
                    vehicle_config_t *vehicle,
                    float dt_s);

#endif
