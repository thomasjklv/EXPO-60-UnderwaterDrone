#ifndef ALLOCATOR_H
#define ALLOCATOR_H

#include <stdbool.h>

#include "actuator_model.h"

bool allocator_solve(vehicle_config_t *vehicle,
                     const allocator_runtime_t *runtime,
                     const wrench4_t *desired,
                     allocator_result_t *result);

void allocator_build_matrix(const vehicle_config_t *vehicle,
                            const allocator_runtime_t *runtime,
                            float B[CA_AXIS_COUNT][CA_MAX_ACTUATORS]);

#endif
