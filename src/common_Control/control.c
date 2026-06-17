#include "common_Control/control.h"

#include <ctype.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

#include "common_Control/allocator.h"
#include "config.h"

typedef struct
{
    float kp;
    float ki;
    float kd;
} pid_gains_t;

typedef struct
{
    float integral;
    float prev_error;
    int has_prev;
} pid_state_t;

typedef struct
{
    pid_gains_t roll;
    pid_gains_t pitch;
    pid_gains_t yaw;

    float desired_yaw_deg;
    float desired_pitch_deg;
    float desired_roll_deg;
    float desired_surge_force_N;
} live_control_config_t;

static live_control_config_t g_live_cfg;
static int g_live_cfg_initialized = 0;
static time_t g_live_cfg_mtime = 0;
static double g_last_cfg_check_s = 0.0;

static pid_state_t g_roll_pid = {0};
static pid_state_t g_pitch_pid = {0};
static pid_state_t g_yaw_pid = {0};

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

static double monotonic_time_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static const char *runtime_config_path(void)
{
    const char *path = getenv("CONTROL_RUNTIME_CONFIG_PATH");
    if ((path != NULL) && (path[0] != '\0')) {
        return path;
    }
    return CONTROL_RUNTIME_CONFIG_PATH;
}

static void trim_inplace(char *s)
{
    size_t len;
    char *start;

    if (s == NULL) {
        return;
    }

    start = s;
    while ((*start != '\0') && isspace((unsigned char)*start)) {
        start++;
    }

    if (start != s) {
        memmove(s, start, strlen(start) + 1U);
    }

    len = strlen(s);
    while ((len > 0U) && isspace((unsigned char)s[len - 1U])) {
        s[len - 1U] = '\0';
        len--;
    }
}

static void set_default_live_config(live_control_config_t *cfg)
{
    if (cfg == NULL) {
        return;
    }

    memset(cfg, 0, sizeof(*cfg));

    cfg->roll.kp = CONTROL_KP_ROLL_NM_PER_DEG;
    cfg->roll.ki = CONTROL_KI_ROLL_NM_PER_DEG_S;
    cfg->roll.kd = CONTROL_KD_ROLL_NM_PER_DEG_PER_S;

    cfg->pitch.kp = CONTROL_KP_PITCH_NM_PER_DEG;
    cfg->pitch.ki = CONTROL_KI_PITCH_NM_PER_DEG_S;
    cfg->pitch.kd = CONTROL_KD_PITCH_NM_PER_DEG_PER_S;

    cfg->yaw.kp = CONTROL_KP_YAW_NM_PER_DEG;
    cfg->yaw.ki = CONTROL_KI_YAW_NM_PER_DEG_S;
    cfg->yaw.kd = CONTROL_KD_YAW_NM_PER_DEG_PER_S;

    cfg->desired_yaw_deg = CONTROL_DEFAULT_DESIRED_YAW_DEG;
    cfg->desired_pitch_deg = CONTROL_DEFAULT_DESIRED_PITCH_DEG;
    cfg->desired_roll_deg = CONTROL_DEFAULT_DESIRED_ROLL_DEG;
    cfg->desired_surge_force_N = CONTROL_DEFAULT_SURGE_FORCE_N;
}

static void reset_pid_state(pid_state_t *state)
{
    if (state == NULL) {
        return;
    }

    state->integral = 0.0f;
    state->prev_error = 0.0f;
    state->has_prev = 0;
}

static void reset_all_pids(void)
{
    reset_pid_state(&g_roll_pid);
    reset_pid_state(&g_pitch_pid);
    reset_pid_state(&g_yaw_pid);
}

static void apply_runtime_key_value(live_control_config_t *cfg,
                                    const char *key,
                                    const char *value)
{
    float v;

    if ((cfg == NULL) || (key == NULL) || (value == NULL)) {
        return;
    }

    v = strtof(value, NULL);

    if (strcmp(key, "roll_kp") == 0) cfg->roll.kp = v;
    else if (strcmp(key, "roll_ki") == 0) cfg->roll.ki = v;
    else if (strcmp(key, "roll_kd") == 0) cfg->roll.kd = v;
    else if (strcmp(key, "pitch_kp") == 0) cfg->pitch.kp = v;
    else if (strcmp(key, "pitch_ki") == 0) cfg->pitch.ki = v;
    else if (strcmp(key, "pitch_kd") == 0) cfg->pitch.kd = v;
    else if (strcmp(key, "yaw_kp") == 0) cfg->yaw.kp = v;
    else if (strcmp(key, "yaw_ki") == 0) cfg->yaw.ki = v;
    else if (strcmp(key, "yaw_kd") == 0) cfg->yaw.kd = v;
    else if (strcmp(key, "desired_roll_deg") == 0) cfg->desired_roll_deg = v;
    else if (strcmp(key, "desired_pitch_deg") == 0) cfg->desired_pitch_deg = v;
    else if (strcmp(key, "desired_yaw_deg") == 0) cfg->desired_yaw_deg = v;
    else if (strcmp(key, "desired_surge_force_N") == 0) cfg->desired_surge_force_N = v;
}

static void reload_live_config_if_needed(void)
{
    const char *path;
    struct stat st;
    FILE *fp;
    char line[256];
    live_control_config_t new_cfg;
    double now;

    if (!g_live_cfg_initialized) {
        set_default_live_config(&g_live_cfg);
        g_live_cfg_initialized = 1;
    }

    now = monotonic_time_s();
    if ((now - g_last_cfg_check_s) < CONTROL_RUNTIME_RELOAD_PERIOD_S) {
        return;
    }
    g_last_cfg_check_s = now;

    path = runtime_config_path();
    if ((path == NULL) || (path[0] == '\0')) {
        return;
    }

    if (stat(path, &st) != 0) {
        return;
    }

    if (st.st_mtime == g_live_cfg_mtime) {
        return;
    }

    set_default_live_config(&new_cfg);

    fp = fopen(path, "r");
    if (fp == NULL) {
        return;
    }

    while (fgets(line, sizeof(line), fp) != NULL) {
        char *eq;
        char *key;
        char *value;

        trim_inplace(line);
        if ((line[0] == '\0') || (line[0] == '#') || (line[0] == ';')) {
            continue;
        }

        eq = strchr(line, '=');
        if (eq == NULL) {
            continue;
        }

        *eq = '\0';
        key = line;
        value = eq + 1;

        trim_inplace(key);
        trim_inplace(value);

        apply_runtime_key_value(&new_cfg, key, value);
    }

    fclose(fp);

    g_live_cfg = new_cfg;
    g_live_cfg_mtime = st.st_mtime;
    reset_all_pids();

    printf("\n[control] live config reloaded: "
           "roll(%.3f, %.3f, %.3f) "
           "pitch(%.3f, %.3f, %.3f) "
           "yaw(%.3f, %.3f, %.3f) "
           "setpoints(yaw=%.2f, pitch=%.2f, roll=%.2f, surge=%.2f)\n",
           g_live_cfg.roll.kp, g_live_cfg.roll.ki, g_live_cfg.roll.kd,
           g_live_cfg.pitch.kp, g_live_cfg.pitch.ki, g_live_cfg.pitch.kd,
           g_live_cfg.yaw.kp, g_live_cfg.yaw.ki, g_live_cfg.yaw.kd,
           g_live_cfg.desired_yaw_deg,
           g_live_cfg.desired_pitch_deg,
           g_live_cfg.desired_roll_deg,
           g_live_cfg.desired_surge_force_N);
    fflush(stdout);
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

static float pid_step(pid_state_t *state,
                      pid_gains_t gains,
                      float error,
                      float dt_s,
                      float output_limit)
{
    float derivative = 0.0f;
    float integral_limit = CONTROL_PID_INTEGRAL_LIMIT;
    float output;

    if (state == NULL) {
        return 0.0f;
    }

    if (dt_s <= 0.0f) {
        dt_s = (float)CONTROL_LOOP_PERIOD_US * 1e-6f;
    }

    state->integral += error * dt_s;

    if (fabsf(gains.ki) > 1e-6f) {
        float ki_based_limit = output_limit / fabsf(gains.ki);
        if (ki_based_limit < integral_limit) {
            integral_limit = ki_based_limit;
        }
    }

    state->integral = clampf(state->integral, -integral_limit, integral_limit);

    if (state->has_prev) {
        derivative = (error - state->prev_error) / dt_s;
    } else {
        state->has_prev = 1;
    }

    state->prev_error = error;

    output = gains.kp * error
           + gains.ki * state->integral
           + gains.kd * derivative;

    return clampf(output, -output_limit, output_limit);
}

wrench4_t controller_compute_desired_wrench(const drone_MAIN *drone, float dt_s)
{
    float roll_error_deg;
    float pitch_error_deg;
    float yaw_error_deg;
    float surge_force_N;

    reload_live_config_if_needed();

    if (drone == NULL) {
        return wrench4_zero();
    }

    roll_error_deg = g_live_cfg.desired_roll_deg - drone->ACTUALbodyAttitude4D.r;
    pitch_error_deg = g_live_cfg.desired_pitch_deg - drone->ACTUALbodyAttitude4D.y;
    yaw_error_deg = wrap_angle_deg(g_live_cfg.desired_yaw_deg - drone->ACTUALbodyAttitude4D.x);

    surge_force_N = clampf(g_live_cfg.desired_surge_force_N,
                           -CONTROL_MAX_SURGE_FORCE_N,
                            CONTROL_MAX_SURGE_FORCE_N);

    return wrench4_create(
        surge_force_N,
        pid_step(&g_roll_pid, g_live_cfg.roll, roll_error_deg, dt_s, CONTROL_MAX_ROLL_MOMENT_NM),
        pid_step(&g_pitch_pid, g_live_cfg.pitch, pitch_error_deg, dt_s, CONTROL_MAX_PITCH_MOMENT_NM),
        pid_step(&g_yaw_pid, g_live_cfg.yaw, yaw_error_deg, dt_s, CONTROL_MAX_YAW_MOMENT_NM)
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

    reload_live_config_if_needed();

    drone->DESIREDbodyAttitude4D = bodyAttitude4D_create(
        g_live_cfg.desired_yaw_deg,
        g_live_cfg.desired_pitch_deg,
        g_live_cfg.desired_surge_force_N,
        g_live_cfg.desired_roll_deg
    );

    desired_wrench = controller_compute_desired_wrench(drone, dt_s);

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