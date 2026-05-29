#include "actuator_specs.h"

const actuator_spec_t g_actuator_specs[] =
{
    {
        .name = "roll_Fin_L",
        .type = ACTUATOR_FIN,
        .driver_type = ACTUATOR_DRIVER_SERVO,
        .enabled = true,
        .inverted = false,
        .channel = 7,

        .cmd_min = -30.0f,
        .cmd_max = 30.0f,
        .cmd_neutral = 0.0f,

        .pwm_min = 1000,
        .pwm_max = 2000,

        .position_m = { 0.2f, -0.2f, 0.0f },
        .force_dir_body = { 0.0f, 0.0f, 1.0f },
        .direct_moment_per_cmd_body = { 0.0f, 0.0f, 0.0f },

        .rate_limit_units_per_s = 360.0f,
        .deadzone = 0.1f,
        .efficiency = 1.0f,

        /* generated id: roll_fin_l */
        .model.fin = {
            .area_m2 = 0.006f,
            .cl_alpha_per_rad = 3.8f,
            .deflection_rad_per_cmd = 0.017453f,
            .min_speed_mps = 0.2f
        }
    },

    {
        .name = "pitch_Fin_R",
        .type = ACTUATOR_FIN,
        .driver_type = ACTUATOR_DRIVER_SERVO,
        .enabled = true,
        .inverted = false,
        .channel = 8,

        .cmd_min = -30.0f,
        .cmd_max = 30.0f,
        .cmd_neutral = 0.0f,

        .pwm_min = 1000,
        .pwm_max = 2000,

        .position_m = { -40.0f, 0.0f, 0.0f },
        .force_dir_body = { 0.0f, 0.0f, 1.0f },
        .direct_moment_per_cmd_body = { 0.0f, 0.0f, 0.0f },

        .rate_limit_units_per_s = 360.0f,
        .deadzone = 0.1f,
        .efficiency = 1.0f,

        /* generated id: pitch_fin_r */
        .model.fin = {
            .area_m2 = 0.006f,
            .cl_alpha_per_rad = 3.8f,
            .deflection_rad_per_cmd = 0.017453f,
            .min_speed_mps = 0.2f
        }
    }
};

const uint8_t g_actuator_specs_count =
    (uint8_t)(sizeof(g_actuator_specs) / sizeof(g_actuator_specs[0]));
