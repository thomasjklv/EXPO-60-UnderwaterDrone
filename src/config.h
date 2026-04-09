#ifndef CONFIG_H
#define CONFIG_H

/* ===== DEBUG ===== */
#define LOG_FOLDER "logs"
#define LOG_RATE_HZ 10
#define LOG_AUTO_FLUSH 1
#define LOGGER_MAX_COLUMNS 32
#define LOGGER_MAX_NAME_LEN 64
#define LOGGER_MAX_VALUE_LEN 64
/*==================*/

/* ===== DRONE COM ===== */
#define LISTEN_PORT 14670
#define AUTOARM 1
#define sPrintTelemetry 0

#define ACTUATOR_HOST "127.0.0.1"
#define ACTUATOR_PORT 14550
/*==================*/

/* ===== DRONE Control ===== */
#define ROLL_CONT_DEADZONE 5.0f

#define CONTROL_DEFAULT_FORWARD_SPEED_MPS 1.50f

#define CONTROL_KP_ROLL_NM_PER_DEG  0.30f
#define CONTROL_KP_PITCH_NM_PER_DEG 0.30f
#define CONTROL_KP_YAW_NM_PER_DEG   0.22f

#define CONTROL_MAX_SURGE_FORCE_N   40.0f
#define CONTROL_MAX_ROLL_MOMENT_NM   6.0f
#define CONTROL_MAX_PITCH_MOMENT_NM  6.0f
#define CONTROL_MAX_YAW_MOMENT_NM    6.0f

#define ALLOCATOR_FEASIBILITY_EPSILON 0.75f
/*==================*/

#endif
