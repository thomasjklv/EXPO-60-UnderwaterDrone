#ifndef CONFIG_H
#define CONFIG_H

/* ===== DEBUG ===== */
#define LOG_FOLDER "../src/logs/"
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

/* ===== DRONE Controll ===== */
#define ROLL_CONT_DEADZONE 5.0f


#endif