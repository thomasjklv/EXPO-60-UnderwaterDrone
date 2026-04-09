#include "common_Control/allocator.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "config.h"

static void wrench_to_array(const wrench4_t *wrench, float out[CA_AXIS_COUNT])
{
    if ((wrench == NULL) || (out == NULL)) {
        return;
    }

    out[CA_AXIS_SURGE] = wrench->surge_N;
    out[CA_AXIS_ROLL]  = wrench->roll_Nm;
    out[CA_AXIS_PITCH] = wrench->pitch_Nm;
    out[CA_AXIS_YAW]   = wrench->yaw_Nm;
}

static wrench4_t array_to_wrench(const float in[CA_AXIS_COUNT])
{
    if (in == NULL) {
        return wrench4_zero();
    }

    return wrench4_create(in[CA_AXIS_SURGE],
                          in[CA_AXIS_ROLL],
                          in[CA_AXIS_PITCH],
                          in[CA_AXIS_YAW]);
}

void allocator_build_matrix(const vehicle_config_t *vehicle,
                            const allocator_runtime_t *runtime,
                            float B[CA_AXIS_COUNT][CA_MAX_ACTUATORS])
{
    uint8_t i;

    if ((vehicle == NULL) || (B == NULL)) {
        return;
    }

    memset(B, 0, sizeof(float) * CA_AXIS_COUNT * CA_MAX_ACTUATORS);

    for (i = 0; i < vehicle->actuator_count; ++i) {
        float col[CA_AXIS_COUNT];
        actuator_compute_effect_column(vehicle, &vehicle->actuators[i], runtime, col);

        B[CA_AXIS_SURGE][i] = col[CA_AXIS_SURGE];
        B[CA_AXIS_ROLL][i]  = col[CA_AXIS_ROLL];
        B[CA_AXIS_PITCH][i] = col[CA_AXIS_PITCH];
        B[CA_AXIS_YAW][i]   = col[CA_AXIS_YAW];
    }
}

static bool solve_linear_system(float A[CA_MAX_ACTUATORS][CA_MAX_ACTUATORS],
                                float b[CA_MAX_ACTUATORS],
                                float x[CA_MAX_ACTUATORS],
                                int n)
{
    int i;
    int j;
    int k;

    if ((A == NULL) || (b == NULL) || (x == NULL) || (n <= 0) || (n > CA_MAX_ACTUATORS)) {
        return false;
    }

    for (i = 0; i < n; ++i)
    {
        int pivot = i;
        float max_abs = fabsf(A[i][i]);

        for (j = i + 1; j < n; ++j)
        {
            float candidate = fabsf(A[j][i]);
            if (candidate > max_abs) {
                max_abs = candidate;
                pivot = j;
            }
        }

        if (max_abs < 1e-8f) {
            return false;
        }

        if (pivot != i)
        {
            for (k = 0; k < n; ++k)
            {
                float tmp = A[i][k];
                A[i][k] = A[pivot][k];
                A[pivot][k] = tmp;
            }

            {
                float tmp = b[i];
                b[i] = b[pivot];
                b[pivot] = tmp;
            }
        }

        for (j = i + 1; j < n; ++j)
        {
            float factor = A[j][i] / A[i][i];
            A[j][i] = 0.0f;

            for (k = i + 1; k < n; ++k) {
                A[j][k] -= factor * A[i][k];
            }

            b[j] -= factor * b[i];
        }
    }

    for (i = n - 1; i >= 0; --i)
    {
        float sum = b[i];

        for (j = i + 1; j < n; ++j) {
            sum -= A[i][j] * x[j];
        }

        x[i] = sum / A[i][i];
    }

    return true;
}

static void compute_fixed_contribution(const float B[CA_AXIS_COUNT][CA_MAX_ACTUATORS],
                                       const vehicle_config_t *vehicle,
                                       const float delta_cmd[CA_MAX_ACTUATORS],
                                       const bool fixed_mask[CA_MAX_ACTUATORS],
                                       float out_tau[CA_AXIS_COUNT])
{
    uint8_t axis;
    uint8_t i;

    memset(out_tau, 0, sizeof(float) * CA_AXIS_COUNT);

    if ((B == NULL) || (vehicle == NULL) || (delta_cmd == NULL) || (fixed_mask == NULL)) {
        return;
    }

    for (axis = 0; axis < CA_AXIS_COUNT; ++axis)
    {
        for (i = 0; i < vehicle->actuator_count; ++i)
        {
            if (fixed_mask[i]) {
                out_tau[axis] += B[axis][i] * delta_cmd[i];
            }
        }
    }
}

static int collect_free_indices(const vehicle_config_t *vehicle,
                                const bool fixed_mask[CA_MAX_ACTUATORS],
                                int free_indices[CA_MAX_ACTUATORS])
{
    int count = 0;
    uint8_t i;

    if ((vehicle == NULL) || (fixed_mask == NULL) || (free_indices == NULL)) {
        return 0;
    }

    for (i = 0; i < vehicle->actuator_count; ++i)
    {
        if (!fixed_mask[i]) {
            free_indices[count++] = (int)i;
        }
    }

    return count;
}

static bool solve_free_set(const vehicle_config_t *vehicle,
                           const float B[CA_AXIS_COUNT][CA_MAX_ACTUATORS],
                           const float tau_target[CA_AXIS_COUNT],
                           const int free_indices[CA_MAX_ACTUATORS],
                           int free_count,
                           float out_delta[CA_MAX_ACTUATORS])
{
    float A[CA_MAX_ACTUATORS][CA_MAX_ACTUATORS];
    float rhs[CA_MAX_ACTUATORS];
    int j;
    int k;
    int axis;

    if ((vehicle == NULL) || (B == NULL) || (tau_target == NULL) ||
        (free_indices == NULL) || (out_delta == NULL) || (free_count <= 0)) {
        return false;
    }

    memset(A, 0, sizeof(A));
    memset(rhs, 0, sizeof(rhs));
    memset(out_delta, 0, sizeof(float) * CA_MAX_ACTUATORS);

    for (j = 0; j < free_count; ++j)
    {
        int col_j = free_indices[j];

        for (axis = 0; axis < CA_AXIS_COUNT; ++axis)
        {
            float w = vehicle->axis_weight[axis];
            rhs[j] += w * B[axis][col_j] * tau_target[axis];
        }

        A[j][j] += vehicle->damping_lambda * vehicle->damping_lambda;

        for (k = 0; k < free_count; ++k)
        {
            int col_k = free_indices[k];

            for (axis = 0; axis < CA_AXIS_COUNT; ++axis)
            {
                float w = vehicle->axis_weight[axis];
                A[j][k] += w * B[axis][col_j] * B[axis][col_k];
            }
        }
    }

    return solve_linear_system(A, rhs, out_delta, free_count);
}

static void compute_achieved_wrench(const vehicle_config_t *vehicle,
                                    const allocator_runtime_t *runtime,
                                    const float absolute_cmd[CA_MAX_ACTUATORS],
                                    wrench4_t *out_wrench)
{
    uint8_t i;
    wrench4_t total = wrench4_zero();

    if ((vehicle == NULL) || (absolute_cmd == NULL) || (out_wrench == NULL)) {
        return;
    }

    for (i = 0; i < vehicle->actuator_count; ++i)
    {
        wrench4_t w = actuator_compute_wrench_from_command(vehicle,
                                                           &vehicle->actuators[i],
                                                           runtime,
                                                           absolute_cmd[i]);

        total.surge_N += w.surge_N;
        total.roll_Nm += w.roll_Nm;
        total.pitch_Nm += w.pitch_Nm;
        total.yaw_Nm += w.yaw_Nm;
    }

    *out_wrench = total;
}

bool allocator_solve(vehicle_config_t *vehicle,
                     const allocator_runtime_t *runtime,
                     const wrench4_t *desired,
                     allocator_result_t *result)
{
    float B[CA_AXIS_COUNT][CA_MAX_ACTUATORS];
    float desired_tau[CA_AXIS_COUNT];
    float fixed_tau[CA_AXIS_COUNT];
    float target_tau[CA_AXIS_COUNT];
    float delta_cmd[CA_MAX_ACTUATORS];
    bool fixed_mask[CA_MAX_ACTUATORS];
    int free_indices[CA_MAX_ACTUATORS];
    uint8_t i;
    int iteration;
    bool converged = false;

    if ((vehicle == NULL) || (runtime == NULL) || (desired == NULL) || (result == NULL)) {
        return false;
    }

    memset(result, 0, sizeof(*result));
    memset(delta_cmd, 0, sizeof(delta_cmd));
    memset(fixed_mask, 0, sizeof(fixed_mask));

    allocator_build_matrix(vehicle, runtime, B);
    wrench_to_array(desired, desired_tau);

    for (i = 0; i < vehicle->actuator_count; ++i)
    {
        if (!vehicle->actuators[i].enabled) {
            fixed_mask[i] = true;
            result->disabled_mask |= (1u << i);
            delta_cmd[i] = 0.0f;
            result->command[i] = vehicle->actuators[i].cmd_neutral;
        } else {
            result->command[i] = vehicle->actuators[i].last_command;
        }
    }

    for (iteration = 0; iteration < (int)vehicle->actuator_count + 1; ++iteration)
    {
        float free_solution[CA_MAX_ACTUATORS];
        int free_count;
        bool violation_found = false;

        compute_fixed_contribution(B, vehicle, delta_cmd, fixed_mask, fixed_tau);

        for (i = 0; i < CA_AXIS_COUNT; ++i) {
            target_tau[i] = desired_tau[i] - fixed_tau[i];
        }

        free_count = collect_free_indices(vehicle, fixed_mask, free_indices);

        if (free_count <= 0) {
            converged = true;
            break;
        }

        if (!solve_free_set(vehicle, B, target_tau, free_indices, free_count, free_solution)) {
            break;
        }

        for (i = 0; i < (uint8_t)free_count; ++i)
        {
            int idx = free_indices[i];
            float trial = free_solution[i];
            float dmin = actuator_delta_min(&vehicle->actuators[idx]);
            float dmax = actuator_delta_max(&vehicle->actuators[idx]);

            if (trial < dmin) {
                delta_cmd[idx] = dmin;
                fixed_mask[idx] = true;
                result->saturated_mask |= (1u << idx);
                violation_found = true;
            } else if (trial > dmax) {
                delta_cmd[idx] = dmax;
                fixed_mask[idx] = true;
                result->saturated_mask |= (1u << idx);
                violation_found = true;
            } else {
                delta_cmd[idx] = trial;
            }
        }

        if (!violation_found) {
            converged = true;
            break;
        }
    }

    for (i = 0; i < vehicle->actuator_count; ++i)
    {
        actuator_t *actuator = &vehicle->actuators[i];
        float absolute_cmd = actuator->cmd_neutral + delta_cmd[i];
        float max_step = actuator->rate_limit_units_per_s * runtime->dt_s;
        float delta_from_last;

        if (fabsf(delta_cmd[i]) < actuator->deadzone) {
            absolute_cmd = actuator->cmd_neutral;
        }

        if ((runtime->dt_s > 0.0f) && (actuator->rate_limit_units_per_s > 0.0f))
        {
            delta_from_last = absolute_cmd - actuator->last_command;
            delta_from_last = clampf(delta_from_last, -max_step, max_step);
            absolute_cmd = actuator->last_command + delta_from_last;
        }

        absolute_cmd = clampf(absolute_cmd, actuator->cmd_min, actuator->cmd_max);
        result->command[i] = absolute_cmd;
    }

    compute_achieved_wrench(vehicle, runtime, result->command, &result->achieved);

    result->residual.surge_N = desired->surge_N - result->achieved.surge_N;
    result->residual.roll_Nm = desired->roll_Nm - result->achieved.roll_Nm;
    result->residual.pitch_Nm = desired->pitch_Nm - result->achieved.pitch_Nm;
    result->residual.yaw_Nm = desired->yaw_Nm - result->achieved.yaw_Nm;

    result->feasible =
        (fabsf(result->residual.surge_N) <= ALLOCATOR_FEASIBILITY_EPSILON) &&
        (fabsf(result->residual.roll_Nm) <= ALLOCATOR_FEASIBILITY_EPSILON) &&
        (fabsf(result->residual.pitch_Nm) <= ALLOCATOR_FEASIBILITY_EPSILON) &&
        (fabsf(result->residual.yaw_Nm) <= ALLOCATOR_FEASIBILITY_EPSILON);

    if (!converged) {
        result->feasible = false;
    }

    if (!result->feasible) {
        float residual_arr[CA_AXIS_COUNT];
        residual_arr[CA_AXIS_SURGE] = result->residual.surge_N;
        residual_arr[CA_AXIS_ROLL]  = result->residual.roll_Nm;
        residual_arr[CA_AXIS_PITCH] = result->residual.pitch_Nm;
        residual_arr[CA_AXIS_YAW]   = result->residual.yaw_Nm;
        result->residual = array_to_wrench(residual_arr);
    }

    return converged;
}
