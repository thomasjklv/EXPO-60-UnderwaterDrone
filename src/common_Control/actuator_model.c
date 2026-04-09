#include "common_Control/actuator_model.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float actuator_force_gain_per_delta_cmd(const vehicle_config_t *vehicle,
                                               const actuator_t *actuator,
                                               const allocator_runtime_t *runtime)
{
    float gain = 0.0f;

    if ((vehicle == NULL) || (actuator == NULL)) {
        return 0.0f;
    }

    if (!actuator->enabled) {
        return 0.0f;
    }

    switch (actuator->type)
    {
        case ACTUATOR_THRUSTER:
            gain = actuator->model.thruster.thrust_gain_pos_N_per_cmd;
            break;

        case ACTUATOR_FIN:
        {
            float speed = 0.0f;
            float q = 0.0f;

            if (runtime != NULL) {
                speed = fabsf(runtime->forward_speed_mps);
            }

            if (speed < actuator->model.fin.min_speed_mps) {
                return 0.0f;
            }

            q = 0.5f * vehicle->water_density_kg_m3 * speed * speed;

            gain = q
                 * actuator->model.fin.area_m2
                 * actuator->model.fin.cl_alpha_per_rad
                 * actuator->model.fin.deflection_rad_per_cmd;
            break;
        }

        default:
            gain = 0.0f;
            break;
    }

    gain *= actuator->efficiency;

    if (actuator->inverted) {
        gain = -gain;
    }

    return gain;
}

float actuator_delta_min(const actuator_t *actuator)
{
    if (actuator == NULL) {
        return 0.0f;
    }

    return actuator->cmd_min - actuator->cmd_neutral;
}

float actuator_delta_max(const actuator_t *actuator)
{
    if (actuator == NULL) {
        return 0.0f;
    }

    return actuator->cmd_max - actuator->cmd_neutral;
}

void actuator_compute_effect_column(const vehicle_config_t *vehicle,
                                    const actuator_t *actuator,
                                    const allocator_runtime_t *runtime,
                                    float col[CA_AXIS_COUNT])
{
    float gain = 0.0f;
    vector3 force_dir = vector3_zero();
    vector3 force_per_cmd = vector3_zero();
    vector3 moment_per_cmd = vector3_zero();

    if (col == NULL) {
        return;
    }

    memset(col, 0, sizeof(float) * CA_AXIS_COUNT);

    if ((vehicle == NULL) || (actuator == NULL)) {
        return;
    }

    gain = actuator_force_gain_per_delta_cmd(vehicle, actuator, runtime);
    if (fabsf(gain) < vehicle->authority_epsilon) {
        return;
    }

    force_dir = vector3_normalize(actuator->force_dir_body);
    force_per_cmd = vector3_scale(force_dir, gain);
    moment_per_cmd = vector3_cross(actuator->position_m, force_per_cmd);
    moment_per_cmd = vector3_add(moment_per_cmd, actuator->direct_moment_per_cmd_body);

    col[CA_AXIS_SURGE] = force_per_cmd.x;
    col[CA_AXIS_ROLL]  = moment_per_cmd.x;
    col[CA_AXIS_PITCH] = moment_per_cmd.y;
    col[CA_AXIS_YAW]   = moment_per_cmd.z;
}

wrench4_t actuator_compute_wrench_from_command(const vehicle_config_t *vehicle,
                                               const actuator_t *actuator,
                                               const allocator_runtime_t *runtime,
                                               float absolute_command)
{
    wrench4_t wrench = wrench4_zero();
    float col[CA_AXIS_COUNT];
    float delta_command = 0.0f;

    if ((vehicle == NULL) || (actuator == NULL)) {
        return wrench;
    }

    actuator_compute_effect_column(vehicle, actuator, runtime, col);
    delta_command = absolute_command - actuator->cmd_neutral;

    wrench.surge_N = col[CA_AXIS_SURGE] * delta_command;
    wrench.roll_Nm = col[CA_AXIS_ROLL]  * delta_command;
    wrench.pitch_Nm = col[CA_AXIS_PITCH] * delta_command;
    wrench.yaw_Nm = col[CA_AXIS_YAW] * delta_command;

    return wrench;
}

void vehicle_apply_allocator_result(vehicle_config_t *vehicle,
                                    const allocator_result_t *result)
{
    uint8_t i;

    if ((vehicle == NULL) || (result == NULL)) {
        return;
    }

    for (i = 0; i < vehicle->actuator_count; ++i)
    {
        actuator_t *actuator = &vehicle->actuators[i];
        float cmd = result->command[i];

        if (!actuator->enabled) {
            cmd = actuator->cmd_neutral;
        }

        cmd = clampf(cmd, actuator->cmd_min, actuator->cmd_max);
        actuator->last_command = cmd;

        switch (actuator->driver_type)
        {
            case ACTUATOR_DRIVER_SERVO:
                if (actuator->driver_ref != NULL) {
                    set_ServoAngle((srvSTR *)actuator->driver_ref, (int16_t)lroundf(cmd));
                }
                break;

            case ACTUATOR_DRIVER_MOTOR:
                if (actuator->driver_ref != NULL) {
                    set_MotorDuty((motSTR *)actuator->driver_ref, (int16_t)lroundf(cmd));
                }
                break;

            default:
                break;
        }
    }
}

void vehicle_set_all_neutral(vehicle_config_t *vehicle)
{
    allocator_result_t neutral_result;
    uint8_t i;

    if (vehicle == NULL) {
        return;
    }

    memset(&neutral_result, 0, sizeof(neutral_result));

    for (i = 0; i < vehicle->actuator_count; ++i) {
        neutral_result.command[i] = vehicle->actuators[i].cmd_neutral;
    }

    vehicle_apply_allocator_result(vehicle, &neutral_result);
}
