#ifndef VEHICLE_CONFIG_H
#define VEHICLE_CONFIG_H

#include "actuator_model.h"

void vehicle_config_init_default(vehicle_config_t *vehicle,
                                 srvSTR *yaw_left,
                                 srvSTR *yaw_right,
                                 srvSTR *pitch_left,
                                 srvSTR *pitch_right,
                                 srvSTR *roll_left,
                                 srvSTR *roll_right,
                                 motSTR *main_thruster);

#endif
