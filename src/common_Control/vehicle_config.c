#include "common_Control/vehicle_config.h"

#include <string.h>

#include "actuator_specs.h"
#include "generated_vehicle_settings.h"

static srvSTR g_servo_storage[CA_MAX_ACTUATORS];
static motSTR g_motor_storage[CA_MAX_ACTUATORS];

static void init_vehicle_defaults(vehicle_config_t *vehicle)
{
    if (vehicle == NULL) {
        return;
    }

    vehicle->water_density_kg_m3 = GENERATED_WATER_DENSITY_KG_M3;
    vehicle->damping_lambda = 0.01f;
    vehicle->authority_epsilon = 1e-5f;

    vehicle->axis_weight[CA_AXIS_SURGE] = GENERATED_AXIS_WEIGHT_SURGE;
    vehicle->axis_weight[CA_AXIS_ROLL]  = GENERATED_AXIS_WEIGHT_ROLL;
    vehicle->axis_weight[CA_AXIS_PITCH] = GENERATED_AXIS_WEIGHT_PITCH;
    vehicle->axis_weight[CA_AXIS_YAW]   = GENERATED_AXIS_WEIGHT_YAW;
}

static void init_servo_driver(srvSTR *servo, const actuator_spec_t *spec)
{
    if ((servo == NULL) || (spec == NULL)) {
        return;
    }

    memset(servo, 0, sizeof(*servo));

    servo->CHANNEL = spec->channel;
    servo->ANGLE = (int16_t)spec->cmd_neutral;
    servo->DFLT_ANGLE = (int16_t)spec->cmd_neutral;
    servo->MIN_ANGLE = (int16_t)spec->cmd_min;
    servo->MAX_ANGLE = (int16_t)spec->cmd_max;
    servo->MIN_PWM = spec->pwm_min;
    servo->MAX_PWM = spec->pwm_max;
}

static void init_motor_driver(motSTR *motor, const actuator_spec_t *spec)
{
    if ((motor == NULL) || (spec == NULL)) {
        return;
    }

    memset(motor, 0, sizeof(*motor));

    motor->CHANNEL = spec->channel;
    motor->DUTY = (int16_t)spec->cmd_neutral;
    motor->DFLT_DUTY = (int16_t)spec->cmd_neutral;
    motor->MIN_DUTY = (int16_t)spec->cmd_min;
    motor->MAX_DUTY = (int16_t)spec->cmd_max;
    motor->MIN_PWM = spec->pwm_min;
    motor->MAX_PWM = spec->pwm_max;
}

static void copy_common_fields(actuator_t *actuator, const actuator_spec_t *spec)
{
    if ((actuator == NULL) || (spec == NULL)) {
        return;
    }

    actuator->type = spec->type;
    actuator->driver_type = spec->driver_type;
    actuator->name = spec->name;

    actuator->enabled = spec->enabled;
    actuator->inverted = spec->inverted;

    actuator->position_m = spec->position_m;
    actuator->force_dir_body = spec->force_dir_body;
    actuator->direct_moment_per_cmd_body = spec->direct_moment_per_cmd_body;

    actuator->cmd_min = spec->cmd_min;
    actuator->cmd_max = spec->cmd_max;
    actuator->cmd_neutral = spec->cmd_neutral;
    actuator->last_command = spec->cmd_neutral;

    actuator->rate_limit_units_per_s = spec->rate_limit_units_per_s;
    actuator->deadzone = spec->deadzone;
    actuator->efficiency = spec->efficiency;
}

static void copy_model_fields(actuator_t *actuator, const actuator_spec_t *spec)
{
    if ((actuator == NULL) || (spec == NULL)) {
        return;
    }

    switch (spec->type)
    {
        case ACTUATOR_FIN:
            actuator->model.fin.area_m2 = spec->model.fin.area_m2;
            actuator->model.fin.cl_alpha_per_rad = spec->model.fin.cl_alpha_per_rad;
            actuator->model.fin.deflection_rad_per_cmd = spec->model.fin.deflection_rad_per_cmd;
            actuator->model.fin.min_speed_mps = spec->model.fin.min_speed_mps;
            break;

        case ACTUATOR_THRUSTER:
            actuator->model.thruster.thrust_gain_pos_N_per_cmd =
                spec->model.thruster.thrust_gain_pos_N_per_cmd;
            actuator->model.thruster.thrust_gain_neg_N_per_cmd =
                spec->model.thruster.thrust_gain_neg_N_per_cmd;
            break;

        default:
            break;
    }
}

static void attach_driver_storage(actuator_t *actuator,
                                  const actuator_spec_t *spec,
                                  uint8_t index)
{
    if ((actuator == NULL) || (spec == NULL) || (index >= CA_MAX_ACTUATORS)) {
        return;
    }

    switch (spec->driver_type)
    {
        case ACTUATOR_DRIVER_SERVO:
            init_servo_driver(&g_servo_storage[index], spec);
            actuator->driver_ref = &g_servo_storage[index];
            break;

        case ACTUATOR_DRIVER_MOTOR:
            init_motor_driver(&g_motor_storage[index], spec);
            actuator->driver_ref = &g_motor_storage[index];
            break;

        default:
            actuator->driver_ref = NULL;
            actuator->enabled = false;
            break;
    }
}

static void build_runtime_actuator(vehicle_config_t *vehicle,
                                   const actuator_spec_t *spec,
                                   uint8_t index)
{
    actuator_t *actuator;

    if ((vehicle == NULL) || (spec == NULL) || (index >= CA_MAX_ACTUATORS)) {
        return;
    }

    actuator = &vehicle->actuators[index];
    memset(actuator, 0, sizeof(*actuator));

    copy_common_fields(actuator, spec);
    copy_model_fields(actuator, spec);
    attach_driver_storage(actuator, spec, index);
}

void vehicle_config_init_default(vehicle_config_t *vehicle)
{
    uint8_t i;
    uint8_t count;

    if (vehicle == NULL) {
        return;
    }

    memset(vehicle, 0, sizeof(*vehicle));
    memset(g_servo_storage, 0, sizeof(g_servo_storage));
    memset(g_motor_storage, 0, sizeof(g_motor_storage));

    init_vehicle_defaults(vehicle);

    count = g_actuator_specs_count;
    if (count > CA_MAX_ACTUATORS) {
        count = CA_MAX_ACTUATORS;
    }

    for (i = 0; i < count; ++i) {
        build_runtime_actuator(vehicle, &g_actuator_specs[i], i);
    }

    vehicle->actuator_count = count;
}