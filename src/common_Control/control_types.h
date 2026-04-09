#ifndef CONTROL_TYPES_H
#define CONTROL_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#define CA_MAX_ACTUATORS 12
#define CA_AXIS_COUNT 4

typedef enum
{
    CA_AXIS_SURGE = 0,
    CA_AXIS_ROLL  = 1,
    CA_AXIS_PITCH = 2,
    CA_AXIS_YAW   = 3
} ca_axis_t;

typedef struct
{
    float surge_N;
    float roll_Nm;
    float pitch_Nm;
    float yaw_Nm;
} wrench4_t;

typedef struct
{
    float dt_s;
    float forward_speed_mps;
} allocator_runtime_t;

typedef struct
{
    float command[CA_MAX_ACTUATORS];
    wrench4_t achieved;
    wrench4_t residual;
    uint32_t saturated_mask;
    uint32_t disabled_mask;
    bool feasible;
} allocator_result_t;

static inline wrench4_t wrench4_create(float surge_N,
                                       float roll_Nm,
                                       float pitch_Nm,
                                       float yaw_Nm)
{
    wrench4_t w = { surge_N, roll_Nm, pitch_Nm, yaw_Nm };
    return w;
}

static inline wrench4_t wrench4_zero(void)
{
    wrench4_t w = { 0.0f, 0.0f, 0.0f, 0.0f };
    return w;
}

#endif
