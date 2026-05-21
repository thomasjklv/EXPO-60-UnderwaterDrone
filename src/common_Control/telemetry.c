#include "common_Control/telemetry.h"
#include "config.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <time.h>

#include "transform.h"
#include "common/mavlink.h"

#define MAVLINK_PORT 14670

#define SYS_ID 200
#define COMP_ID MAV_COMP_ID_ONBOARD_COMPUTER
#define TARGET_SYS 1
#define TARGET_COMP 1

static int rx_sock = -1;
static int tx_sock = -1;
static struct sockaddr_in tx_dest;
static bool tx_dest_ready = false;

static float roll_rad  = 0.0f;
static float pitch_rad = 0.0f;
static float yaw_rad   = 0.0f;

static vector3 gyro  = {0};
static vector3 accel = {0};
static vector3 comps = {0};
static gpsSTR gps    = {0};

static double last_attitude_rx_s = -1.0;
static double last_rate_request_s = -1.0;
static bool intervals_requested = false;

static double monotonic_time_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static void init_rx_socket(void)
{
    if (rx_sock >= 0) {
        return;
    }

    rx_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (rx_sock < 0) {
        perror("telemetry socket");
        return;
    }

    {
        struct sockaddr_in local_addr;
        memset(&local_addr, 0, sizeof(local_addr));

        local_addr.sin_family = AF_INET;
        local_addr.sin_port = htons(MAVLINK_PORT);
        local_addr.sin_addr.s_addr = htonl(INADDR_ANY);

        if (bind(rx_sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
            perror("telemetry bind");
        }
    }
}

static void init_tx_socket(void)
{
    if ((tx_sock >= 0) && tx_dest_ready) {
        return;
    }

    tx_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (tx_sock < 0) {
        perror("telemetry tx socket");
        return;
    }

    memset(&tx_dest, 0, sizeof(tx_dest));
    tx_dest.sin_family = AF_INET;
    tx_dest.sin_port = htons(ACTUATOR_PORT);

    if (inet_pton(AF_INET, ACTUATOR_HOST, &tx_dest.sin_addr) != 1) {
        perror("telemetry inet_pton");
        close(tx_sock);
        tx_sock = -1;
        return;
    }

    tx_dest_ready = true;
}

static void send_command_long(float param1,
                              float param2,
                              float param3,
                              float param4,
                              float param5,
                              float param6,
                              float param7,
                              uint16_t command)
{
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len;

    init_tx_socket();
    if ((tx_sock < 0) || !tx_dest_ready) {
        return;
    }

    mavlink_msg_command_long_pack(
        SYS_ID,
        COMP_ID,
        &msg,
        TARGET_SYS,
        TARGET_COMP,
        command,
        0,
        param1,
        param2,
        param3,
        param4,
        param5,
        param6,
        param7
    );

    len = mavlink_msg_to_send_buffer(buffer, &msg);

    if (sendto(tx_sock, buffer, len, 0,
               (struct sockaddr *)&tx_dest,
               sizeof(tx_dest)) < 0) {
        perror("telemetry sendto");
    }
}

static void request_message_interval(uint32_t message_id, int32_t interval_us)
{
    send_command_long((float)message_id,
                      (float)interval_us,
                      0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                      MAV_CMD_SET_MESSAGE_INTERVAL);
}

void telemetry_request_default_intervals(void)
{
    request_message_interval(MAVLINK_MSG_ID_ATTITUDE,
                             TELEMETRY_ATTITUDE_INTERVAL_US);
    request_message_interval(MAVLINK_MSG_ID_RAW_IMU,
                             TELEMETRY_RAW_IMU_INTERVAL_US);
    request_message_interval(MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                             TELEMETRY_GLOBAL_POSITION_INTERVAL_US);

    last_rate_request_s = monotonic_time_s();
    intervals_requested = true;
}

static void maybe_refresh_message_rates(void)
{
    double now = monotonic_time_s();

    if (!intervals_requested) {
        telemetry_request_default_intervals();
        return;
    }

    if ((now - last_rate_request_s) >= TELEMETRY_RATE_REQUEST_RETRY_S) {
        telemetry_request_default_intervals();
    }
}

void telemetry_poll(void)
{
    uint8_t buffer[2048];

    init_rx_socket();
    maybe_refresh_message_rates();

    if (rx_sock < 0) {
        return;
    }

    while (1)
    {
        mavlink_message_t msg;
        mavlink_status_t status;
        ssize_t n = recv(rx_sock, buffer, sizeof(buffer), MSG_DONTWAIT);

        if (n <= 0) {
            break;
        }

        for (ssize_t i = 0; i < n; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
                if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
                    mavlink_attitude_t att;
                    mavlink_msg_attitude_decode(&msg, &att);

                    roll_rad  = att.roll;
                    pitch_rad = att.pitch;
                    yaw_rad   = att.yaw;

                    gyro.x = att.rollspeed;
                    gyro.y = att.pitchspeed;
                    gyro.z = att.yawspeed;

                    last_attitude_rx_s = monotonic_time_s();
                }

                if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
                    mavlink_global_position_int_t pos;
                    mavlink_msg_global_position_int_decode(&msg, &pos);

                    gps.lat_deg = ((double)pos.lat) / 10000000.0;
                    gps.lon_deg = ((double)pos.lon) / 10000000.0;
                    gps.alt_m   = ((float)pos.relative_alt) / 1000.0f;
                }

                if (msg.msgid == MAVLINK_MSG_ID_RAW_IMU) {
                    mavlink_raw_imu_t imu;
                    mavlink_msg_raw_imu_decode(&msg, &imu);

                    accel.x = (float)imu.xacc;
                    accel.y = (float)imu.yacc;
                    accel.z = (float)imu.zacc;

                    comps.x = (float)imu.xmag;
                    comps.y = (float)imu.ymag;
                    comps.z = (float)imu.zmag;
                }
            }
        }
    }
}

bool telemetry_is_attitude_recent(float timeout_s)
{
    if (last_attitude_rx_s < 0.0) {
        return false;
    }

    return telemetry_get_attitude_age_s() <= (double)timeout_s;
}

double telemetry_get_attitude_age_s(void)
{
    if (last_attitude_rx_s < 0.0) {
        return 1e9;
    }

    return monotonic_time_s() - last_attitude_rx_s;
}

static void send_arm_command(float arm_value)
{
    send_command_long(arm_value, 0, 0, 0, 0, 0, 0,
                      MAV_CMD_COMPONENT_ARM_DISARM);
}

/* ================= PUBLIC FUNCTIONS ================= */

void armDrone(void)
{
    printf("ARM COMMAND SENT\n");
    send_arm_command(1.0f);
}

void disarmDrone(void)
{
    printf("DISARM COMMAND SENT\n");
    send_arm_command(0.0f);
}

vector3 get_GYRO_V3(void)
{
    return gyro;
}

vector3 get_ACCEL_V3(void)
{
    return accel;
}

vector3 get_COMPS_V3(void)
{
    return comps;
}

float get_YAW_HEADING(void)
{
    return rad_to_deg(yaw_rad);
}

float get_PITCH_HEADING(void)
{
    return rad_to_deg(pitch_rad);
}

gpsSTR get_GPS(void)
{
    return gps;
}

float rad_to_deg(float rad)
{
    return rad * (180.0f / 3.14159265f);
}

float get_ROLL_HEADING(void)
{
    return rad_to_deg(roll_rad);
}

bodyAttitude4D get_BODY_ATTI4D(void)
{
    return bodyAttitude4D_create(
        rad_to_deg(yaw_rad),
        rad_to_deg(pitch_rad),
        0.0f,
        rad_to_deg(roll_rad)
    );
}
