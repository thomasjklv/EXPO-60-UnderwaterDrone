#ifndef ACTUATOR_MODEL_H
#define ACTUATOR_MODEL_H

#include <stdbool.h>
#include <stdint.h>

#include "actuators.h"
#include "control_types.h"
#include "transform.h"

typedef enum
{
    ACTUATOR_FIN = 0,
    ACTUATOR_THRUSTER = 1
} actuator_type_t;

typedef enum
{
    ACTUATOR_DRIVER_SERVO = 0,
    ACTUATOR_DRIVER_MOTOR = 1
} actuator_driver_type_t;

typedef struct
{
    actuator_type_t type;
    actuator_driver_type_t driver_type;
    const char *name;

    bool enabled;
    bool inverted;

    vector3 position_m;
    vector3 force_dir_body;
    vector3 direct_moment_per_cmd_body;

    float cmd_min;
    float cmd_max;
    float cmd_neutral;
    float last_command;

    float rate_limit_units_per_s;
    float deadzone;
    float efficiency;

    union
    {
        struct
        {
            float thrust_gain_pos_N_per_cmd;
            float thrust_gain_neg_N_per_cmd;
        } thruster;

        struct
        {
            float area_m2;
            float cl_alpha_per_rad;
            float deflection_rad_per_cmd;
            float min_speed_mps;
        } fin;
    } model;

    void *driver_ref;
} actuator_t;

typedef struct
{
    actuator_t actuators[CA_MAX_ACTUATORS];
    uint8_t actuator_count;

    float water_density_kg_m3;
    float damping_lambda;
    float authority_epsilon;
    float axis_weight[CA_AXIS_COUNT];
} vehicle_config_t;

float actuator_delta_min(const actuator_t *actuator);
float actuator_delta_max(const actuator_t *actuator);

void actuator_compute_effect_column(const vehicle_config_t *vehicle,
                                    const actuator_t *actuator,
                                    const allocator_runtime_t *runtime,
                                    float col[CA_AXIS_COUNT]);

wrench4_t actuator_compute_wrench_from_command(const vehicle_config_t *vehicle,
                                               const actuator_t *actuator,
                                               const allocator_runtime_t *runtime,
                                               float absolute_command);

void vehicle_apply_allocator_result(vehicle_config_t *vehicle,
                                    const allocator_result_t *result);

void vehicle_set_all_neutral(vehicle_config_t *vehicle);

#endif
