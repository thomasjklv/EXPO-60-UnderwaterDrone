#include <stdio.h>
#include <string.h>

#include "common_Control/allocator.h"
#include "common_Control/vehicle_config.h"

static void print_result(const char *label,
                         const vehicle_config_t *vehicle,
                         const allocator_result_t *result)
{
    unsigned int i;

    printf("\n=== %s ===\n", label);
    printf("Feasible: %s\n", result->feasible ? "true" : "false");
    printf("Achieved: surge=%7.3f N  roll=%7.3f Nm  pitch=%7.3f Nm  yaw=%7.3f Nm\n",
           result->achieved.surge_N,
           result->achieved.roll_Nm,
           result->achieved.pitch_Nm,
           result->achieved.yaw_Nm);

    printf("Residual: surge=%7.3f N  roll=%7.3f Nm  pitch=%7.3f Nm  yaw=%7.3f Nm\n",
           result->residual.surge_N,
           result->residual.roll_Nm,
           result->residual.pitch_Nm,
           result->residual.yaw_Nm);

    for (i = 0; i < vehicle->actuator_count; ++i) {
        printf("  %-14s -> %7.3f\n",
               vehicle->actuators[i].name,
               result->command[i]);
    }
}

int main(void)
{
    srvSTR yaw_left =  { .CHANNEL = 1, .ANGLE = 0, .DFLT_ANGLE = 0, .MIN_ANGLE = -30, .MAX_ANGLE = 30, .MIN_PWM = 1000, .MAX_PWM = 2000 };
    srvSTR yaw_right = { .CHANNEL = 2, .ANGLE = 0, .DFLT_ANGLE = 0, .MIN_ANGLE = -30, .MAX_ANGLE = 30, .MIN_PWM = 1000, .MAX_PWM = 2000 };
    srvSTR pitch_left =  { .CHANNEL = 3, .ANGLE = 0, .DFLT_ANGLE = 0, .MIN_ANGLE = -30, .MAX_ANGLE = 30, .MIN_PWM = 1000, .MAX_PWM = 2000 };
    srvSTR pitch_right = { .CHANNEL = 4, .ANGLE = 0, .DFLT_ANGLE = 0, .MIN_ANGLE = -30, .MAX_ANGLE = 30, .MIN_PWM = 1000, .MAX_PWM = 2000 };
    srvSTR roll_left =  { .CHANNEL = 5, .ANGLE = 0, .DFLT_ANGLE = 0, .MIN_ANGLE = -30, .MAX_ANGLE = 30, .MIN_PWM = 1000, .MAX_PWM = 2000 };
    srvSTR roll_right = { .CHANNEL = 6, .ANGLE = 0, .DFLT_ANGLE = 0, .MIN_ANGLE = -30, .MAX_ANGLE = 30, .MIN_PWM = 1000, .MAX_PWM = 2000 };
    motSTR main_thruster = { .CHANNEL = 7, .DUTY = 0, .MAX_DUTY = 100 };

    vehicle_config_t vehicle;
    allocator_runtime_t runtime;
    allocator_result_t result;

    wrench4_t pure_surge = wrench4_create(20.0f, 0.0f, 0.0f, 0.0f);
    wrench4_t pure_yaw = wrench4_create(0.0f, 0.0f, 0.0f, 3.0f);
    wrench4_t pure_roll = wrench4_create(0.0f, 2.0f, 0.0f, 0.0f);
    wrench4_t combined = wrench4_create(15.0f, 1.0f, 1.0f, 2.0f);

    vehicle_config_init_default(&vehicle,
                                &yaw_left,
                                &yaw_right,
                                &pitch_left,
                                &pitch_right,
                                &roll_left,
                                &roll_right,
                                &main_thruster);

    runtime.dt_s = 1.0f;
    runtime.forward_speed_mps = 1.50f;

    memset(&result, 0, sizeof(result));

    allocator_solve(&vehicle, &runtime, &pure_surge, &result);
    print_result("Pure surge", &vehicle, &result);

    allocator_solve(&vehicle, &runtime, &pure_yaw, &result);
    print_result("Pure yaw", &vehicle, &result);

    allocator_solve(&vehicle, &runtime, &pure_roll, &result);
    print_result("Pure roll", &vehicle, &result);

    allocator_solve(&vehicle, &runtime, &combined, &result);
    print_result("Combined request", &vehicle, &result);

    return 0;
}
