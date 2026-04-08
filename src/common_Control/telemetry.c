#include "common_Control/telemetry.h"
#include "config.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "transform.h"
#include "common/mavlink.h"

#define MAVLINK_PORT 14670

#define SYS_ID 200
#define COMP_ID MAV_COMP_ID_ONBOARD_COMPUTER
#define TARGET_SYS 1
#define TARGET_COMP 1

static int sock = -1;

static float roll_rad  = 0.0f;
static float pitch_rad = 0.0f;
static float yaw_rad   = 0.0f;

static vector3 gyro  = {0};
static vector3 accel = {0};
static vector3 comps = {0};
static gpsSTR gps    = {0};

static void init_socket(void)
{
    if (sock >= 0) {
        return;
    }

    sock = socket(AF_INET, SOCK_DGRAM, 0);

    struct sockaddr_in local_addr;
    memset(&local_addr, 0, sizeof(local_addr));

    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(MAVLINK_PORT);
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr));
}

static void update_mavlink(void)
{
    init_socket();

    uint8_t buffer[2048];
    ssize_t n = recv(sock, buffer, sizeof(buffer), MSG_DONTWAIT);

    if (n <= 0) {
        return;
    }

    mavlink_message_t msg;
    mavlink_status_t status;

    for (ssize_t i = 0; i < n; i++) {

        if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {

            /* ================= ATTITUDE ================= */

            if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {

                mavlink_attitude_t att;
                mavlink_msg_attitude_decode(&msg, &att);

                roll_rad  = att.roll;
                pitch_rad = att.pitch;
                yaw_rad   = att.yaw;

                gyro.x = att.rollspeed;
                gyro.y = att.pitchspeed;
                gyro.z = att.yawspeed;
            }

            /* ================= GPS ================= */

            if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {

                mavlink_global_position_int_t pos;
                mavlink_msg_global_position_int_decode(&msg, &pos);

                gps.lat_deg = ((double)pos.lat) / 10000000.0;
                gps.lon_deg = ((double)pos.lon) / 10000000.0;
                gps.alt_m   = ((float)pos.relative_alt) / 1000.0f;
            }

            /* ================= IMU ================= */

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

static void send_arm_command(float arm_value)
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);

    struct sockaddr_in dest;
    memset(&dest, 0, sizeof(dest));
    dest.sin_family = AF_INET;
    dest.sin_port = htons(ACTUATOR_PORT);
    inet_pton(AF_INET, ACTUATOR_HOST, &dest.sin_addr);

    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(
        SYS_ID,
        COMP_ID,
        &msg,
        TARGET_SYS,
        TARGET_COMP,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        arm_value,
        0, 0, 0, 0, 0, 0
    );

    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

    sendto(sock, buffer, len, 0,
           (struct sockaddr*)&dest,
           sizeof(dest));

    close(sock);
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
    update_mavlink();
    return gyro;
}

vector3 get_ACCEL_V3(void)
{
    update_mavlink();
    return accel;
}

vector3 get_COMPS_V3(void)
{
    update_mavlink();
    return comps;
}

float get_YAW_HEADING(void)
{
    update_mavlink();
    return rad_to_deg(yaw_rad);
}

float get_PITCH_HEADING(void)
{
    update_mavlink();
    return rad_to_deg(pitch_rad);
}

gpsSTR get_GPS(void)
{
    update_mavlink();
    return gps;
}

float rad_to_deg(float rad)
{
    return rad * (180.0f / 3.14159265f);
}

float get_ROLL_HEADING(void)
{
    update_mavlink();
    return rad_to_deg(roll_rad);
}

bodyAttitude4D get_BODY_ATTI4D(void)
{
    update_mavlink();

    return bodyAttitude4D_create(
        rad_to_deg(yaw_rad),    // x = yaw
        rad_to_deg(pitch_rad),  // y = pitch
        0.0f,                   // z = surge (nog geen bron in jouw code)
        rad_to_deg(roll_rad)    // r = roll
    );
}