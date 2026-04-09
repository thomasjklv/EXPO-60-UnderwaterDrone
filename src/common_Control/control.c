#include "common_Control/control.h"

#include <math.h>
#include <stddef.h>

#include "common_Control/allocator.h"
#include "config.h"

static float wrap_angle_deg(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

static float estimate_forward_speed_mps(const drone_MAIN *drone)
{
    if (drone == NULL) {
        return CONTROL_DEFAULT_FORWARD_SPEED_MPS;
    }

    if (fabsf(drone->ACTUALbodyAttitude4D.z) > 0.01f) {
        return fabsf(drone->ACTUALbodyAttitude4D.z);
    }

    return CONTROL_DEFAULT_FORWARD_SPEED_MPS;
}

wrench4_t controller_compute_desired_wrench(const drone_MAIN *drone)
{
    float roll_error_deg;
    float pitch_error_deg;
    float yaw_error_deg;
    float surge_force_N;

    if (drone == NULL) {
        return wrench4_zero();
    }

    roll_error_deg = drone->DESIREDbodyAttitude4D.r - drone->ACTUALbodyAttitude4D.r;
    pitch_error_deg = drone->DESIREDbodyAttitude4D.y - drone->ACTUALbodyAttitude4D.y;
    yaw_error_deg = wrap_angle_deg(drone->DESIREDbodyAttitude4D.x - drone->ACTUALbodyAttitude4D.x);

    surge_force_N = clampf(drone->DESIREDbodyAttitude4D.z,
                           -CONTROL_MAX_SURGE_FORCE_N,
                            CONTROL_MAX_SURGE_FORCE_N);

    return wrench4_create(
        surge_force_N,
        clampf(CONTROL_KP_ROLL_NM_PER_DEG * roll_error_deg,
               -CONTROL_MAX_ROLL_MOMENT_NM,
                CONTROL_MAX_ROLL_MOMENT_NM),
        clampf(CONTROL_KP_PITCH_NM_PER_DEG * pitch_error_deg,
               -CONTROL_MAX_PITCH_MOMENT_NM,
                CONTROL_MAX_PITCH_MOMENT_NM),
        clampf(CONTROL_KP_YAW_NM_PER_DEG * yaw_error_deg,
               -CONTROL_MAX_YAW_MOMENT_NM,
                CONTROL_MAX_YAW_MOMENT_NM)
    );
}

void control_update(drone_MAIN *drone,
                    vehicle_config_t *vehicle,
                    float dt_s)
{
    allocator_runtime_t runtime;
    allocator_result_t allocation;
    wrench4_t desired_wrench;
    float yaw_error_deg;
    float pitch_error_deg;
    float roll_error_deg;

    if ((drone == NULL) || (vehicle == NULL)) {
        return;
    }

    desired_wrench = controller_compute_desired_wrench(drone);

    runtime.dt_s = dt_s;
    runtime.forward_speed_mps = estimate_forward_speed_mps(drone);

    (void)allocator_solve(vehicle, &runtime, &desired_wrench, &allocation);
    vehicle_apply_allocator_result(vehicle, &allocation);

    yaw_error_deg = wrap_angle_deg(drone->DESIREDbodyAttitude4D.x - drone->ACTUALbodyAttitude4D.x);
    pitch_error_deg = drone->DESIREDbodyAttitude4D.y - drone->ACTUALbodyAttitude4D.y;
    roll_error_deg = drone->DESIREDbodyAttitude4D.r - drone->ACTUALbodyAttitude4D.r;

    drone->bodyAtt4D = bodyAttitude4D_create(
        yaw_error_deg,
        pitch_error_deg,
        desired_wrench.surge_N,
        roll_error_deg
    );
}
