#ifndef ACTUATOR_SPECS_H
#define ACTUATOR_SPECS_H

#include <stdbool.h>
#include <stdint.h>

#include "common_Control/actuator_model.h"

typedef struct
{
    const char *name;

    actuator_type_t type;
    actuator_driver_type_t driver_type;

    bool enabled;
    bool inverted;

    uint8_t channel;

    float cmd_min;
    float cmd_max;
    float cmd_neutral;

    uint16_t pwm_min;
    uint16_t pwm_max;

    vector3 position_m;
    vector3 force_dir_body;
    vector3 direct_moment_per_cmd_body;

    float rate_limit_units_per_s;
    float deadzone;
    float efficiency;

    union
    {
        struct
        {
            float area_m2;
            float cl_alpha_per_rad;
            float deflection_rad_per_cmd;
            float min_speed_mps;
        } fin;

        struct
        {
            float thrust_gain_pos_N_per_cmd;
            float thrust_gain_neg_N_per_cmd;
        } thruster;
    } model;
} actuator_spec_t;

extern const actuator_spec_t g_actuator_specs[];
extern const uint8_t g_actuator_specs_count;

#endif
