#include "common_Control/vehicle_config.h"

#include <string.h>

#define WATER_DENSITY_SEAWATER_KG_M3 1025.0f

static actuator_t make_fin(const char *name,
                           vector3 position_m,
                           vector3 force_dir_body,
                           srvSTR *servo)
{
    actuator_t actuator;

    memset(&actuator, 0, sizeof(actuator));

    actuator.type = ACTUATOR_FIN;
    actuator.driver_type = ACTUATOR_DRIVER_SERVO;
    actuator.name = name;
    actuator.enabled = true;
    actuator.inverted = false;

    actuator.position_m = position_m;
    actuator.force_dir_body = force_dir_body;
    actuator.direct_moment_per_cmd_body = vector3_zero();

    actuator.cmd_min = (float)servo->MIN_ANGLE;
    actuator.cmd_max = (float)servo->MAX_ANGLE;
    actuator.cmd_neutral = (float)servo->DFLT_ANGLE;
    actuator.last_command = actuator.cmd_neutral;

    actuator.rate_limit_units_per_s = 120.0f;
    actuator.deadzone = 0.30f;
    actuator.efficiency = 1.0f;

    actuator.model.fin.area_m2 = 0.0035f;
    actuator.model.fin.cl_alpha_per_rad = 3.80f;
    actuator.model.fin.deflection_rad_per_cmd = 0.01745329252f;
    actuator.model.fin.min_speed_mps = 0.20f;

    actuator.driver_ref = servo;

    return actuator;
}

static actuator_t make_thruster(const char *name,
                                vector3 position_m,
                                vector3 force_dir_body,
                                motSTR *motor)
{
    actuator_t actuator;

    memset(&actuator, 0, sizeof(actuator));

    actuator.type = ACTUATOR_THRUSTER;
    actuator.driver_type = ACTUATOR_DRIVER_MOTOR;
    actuator.name = name;
    actuator.enabled = true;
    actuator.inverted = false;

    actuator.position_m = position_m;
    actuator.force_dir_body = force_dir_body;
    actuator.direct_moment_per_cmd_body = vector3_zero();

    actuator.cmd_min = 0.0f;
    actuator.cmd_max = (float)motor->MAX_DUTY;
    actuator.cmd_neutral = 0.0f;
    actuator.last_command = 0.0f;

    actuator.rate_limit_units_per_s = 80.0f;
    actuator.deadzone = 0.0f;
    actuator.efficiency = 1.0f;

    actuator.model.thruster.thrust_gain_pos_N_per_cmd = 0.45f;
    actuator.model.thruster.thrust_gain_neg_N_per_cmd = 0.0f;

    actuator.driver_ref = motor;

    return actuator;
}

void vehicle_config_init_default(vehicle_config_t *vehicle,
                                 srvSTR *yaw_left,
                                 srvSTR *yaw_right,
                                 srvSTR *pitch_left,
                                 srvSTR *pitch_right,
                                 srvSTR *roll_left,
                                 srvSTR *roll_right,
                                 motSTR *main_thruster)
{
    if (vehicle == NULL) {
        return;
    }

    memset(vehicle, 0, sizeof(*vehicle));

    vehicle->water_density_kg_m3 = WATER_DENSITY_SEAWATER_KG_M3;
    vehicle->damping_lambda = 0.01f;
    vehicle->authority_epsilon = 1e-5f;

    vehicle->axis_weight[CA_AXIS_SURGE] = 1.0f;
    vehicle->axis_weight[CA_AXIS_ROLL]  = 1.5f;
    vehicle->axis_weight[CA_AXIS_PITCH] = 1.5f;
    vehicle->axis_weight[CA_AXIS_YAW]   = 2.0f;

    /*
      Coordinate system assumptions for this default config:
      x = forward
      y = starboard/right
      z = down

      Positive yaw fins produce lateral force in +y.
      Positive pitch fins produce vertical force in +z.
      Positive roll fin lift is also taken in +z, with roll coming from the y-offset.
    */

    vehicle->actuators[0] = make_fin("yaw_left",
                                     vector3_create(-0.45f, -0.12f, 0.00f),
                                     vector3_create(0.0f, 1.0f, 0.0f),
                                     yaw_left);

    vehicle->actuators[1] = make_fin("yaw_right",
                                     vector3_create(-0.45f,  0.12f, 0.00f),
                                     vector3_create(0.0f, 1.0f, 0.0f),
                                     yaw_right);

    vehicle->actuators[2] = make_fin("pitch_left",
                                     vector3_create(-0.45f, -0.10f, 0.00f),
                                     vector3_create(0.0f, 0.0f, 1.0f),
                                     pitch_left);

    vehicle->actuators[3] = make_fin("pitch_right",
                                     vector3_create(-0.45f,  0.10f, 0.00f),
                                     vector3_create(0.0f, 0.0f, 1.0f),
                                     pitch_right);

    vehicle->actuators[4] = make_fin("roll_left",
                                     vector3_create( 0.00f, -0.18f, 0.00f),
                                     vector3_create(0.0f, 0.0f, 1.0f),
                                     roll_left);

    vehicle->actuators[5] = make_fin("roll_right",
                                     vector3_create( 0.00f,  0.18f, 0.00f),
                                     vector3_create(0.0f, 0.0f, 1.0f),
                                     roll_right);

    vehicle->actuators[6] = make_thruster("main_thruster",
                                          vector3_create(-0.48f, 0.00f, 0.00f),
                                          vector3_create(1.0f, 0.0f, 0.0f),
                                          main_thruster);

    vehicle->actuator_count = 7;
}
