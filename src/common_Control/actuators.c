#include "common_Control/actuators.h"
#include "config.h"

#ifndef ACTUATOR_HOST
#define ACTUATOR_HOST "127.0.0.1"
#endif

#ifndef ACTUATOR_PORT
#define ACTUATOR_PORT 14550
#endif

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

/* Manual MAVLink v1 packet settings */
#define MAVLINK_STX           0xFE
#define MSG_COMMAND_LONG      76
#define MAV_CMD_DO_SET_SERVO  183
#define CRC_EXTRA             152

static uint8_t seq = 0;
static int tx_sock = -1;
static struct sockaddr_in tx_addr;
static bool tx_addr_ready = false;
static uint16_t last_pwm_by_channel[256];
static bool last_pwm_valid_by_channel[256];

/* ==================== Internal Helpers ==================== */

static void init_tx_socket(void)
{
    if ((tx_sock >= 0) && tx_addr_ready) {
        return;
    }

    tx_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (tx_sock < 0) {
        perror("actuator socket");
        return;
    }

    memset(&tx_addr, 0, sizeof(tx_addr));
    tx_addr.sin_family = AF_INET;
    tx_addr.sin_port = htons(ACTUATOR_PORT);

    if (inet_pton(AF_INET, ACTUATOR_HOST, &tx_addr.sin_addr) != 1) {
        perror("actuator inet_pton");
        close(tx_sock);
        tx_sock = -1;
        return;
    }

    tx_addr_ready = true;
}

static uint16_t crc_acc(uint8_t d, uint16_t crc)
{
    uint8_t t = d ^ (uint8_t)(crc & 0xFF);
    t ^= (t << 4);
    return (uint16_t)((crc >> 8) ^ ((uint16_t)t << 8) ^ ((uint16_t)t << 3) ^ (t >> 4));
}

static int16_t clamp_Angle(const srvSTR *servo, int16_t angle)
{
    if (servo == NULL) {
        return 0;
    }

    if (angle > servo->MAX_ANGLE) {
        return servo->MAX_ANGLE;
    }

    if (angle < servo->MIN_ANGLE) {
        return servo->MIN_ANGLE;
    }

    return angle;
}

static int16_t clamp_Duty(const motSTR *motor, int16_t duty)
{
    if (motor == NULL) {
        return 0;
    }

    if (duty < motor->MIN_DUTY) {
        return motor->MIN_DUTY;
    }

    if (duty > motor->MAX_DUTY) {
        return motor->MAX_DUTY;
    }

    return duty;
}

uint16_t servo_AngleToPwm(const srvSTR *servo)
{
    if (servo == NULL) {
        return 1500;
    }

    {
        int16_t angle = clamp_Angle(servo, servo->ANGLE);
        int16_t angleRange = servo->MAX_ANGLE - servo->MIN_ANGLE;
        int16_t pwmRange   = servo->MAX_PWM   - servo->MIN_PWM;

        if ((angleRange <= 0) || (pwmRange <= 0)) {
            return 1500;
        }

        {
            int32_t pwm = servo->MIN_PWM +
                          ((int32_t)(angle - servo->MIN_ANGLE) * pwmRange) / angleRange;

            if (pwm < servo->MIN_PWM) {
                pwm = servo->MIN_PWM;
            }

            if (pwm > servo->MAX_PWM) {
                pwm = servo->MAX_PWM;
            }

            return (uint16_t)pwm;
        }
    }
}

uint16_t motor_DutyToPwm(const motSTR *motor)
{
    int32_t duty_range;
    int32_t pwm_range;
    int32_t duty_shifted;
    int32_t pwm;

    if (motor == NULL) {
        return 1500;
    }

    duty_range = (int32_t)motor->MAX_DUTY - (int32_t)motor->MIN_DUTY;
    pwm_range  = (int32_t)motor->MAX_PWM  - (int32_t)motor->MIN_PWM;

    if ((duty_range <= 0) || (pwm_range <= 0)) {
        return motor->MIN_PWM;
    }

    duty_shifted = (int32_t)clamp_Duty(motor, motor->DUTY) - (int32_t)motor->MIN_DUTY;

    pwm = (int32_t)motor->MIN_PWM + (duty_shifted * pwm_range) / duty_range;

    if (pwm < (int32_t)motor->MIN_PWM) {
        pwm = motor->MIN_PWM;
    }

    if (pwm > (int32_t)motor->MAX_PWM) {
        pwm = motor->MAX_PWM;
    }

    return (uint16_t)pwm;
}

static void send_channel_raw(uint8_t channel, uint16_t pwm)
{
    uint8_t payload[33];
    uint8_t pkt[41];
    uint16_t crc;
    uint16_t cmd = MAV_CMD_DO_SET_SERVO;
    float p1 = (float)channel;
    float p2 = (float)pwm;

    init_tx_socket();
    if ((tx_sock < 0) || !tx_addr_ready) {
        return;
    }

    if (last_pwm_valid_by_channel[channel] && (last_pwm_by_channel[channel] == pwm)) {
        return;
    }

    memset(payload, 0, sizeof(payload));
    memset(pkt, 0, sizeof(pkt));

    memcpy(payload + 0,  &p1,  4);
    memcpy(payload + 4,  &p2,  4);
    memcpy(payload + 28, &cmd, 2);
    payload[30] = 1;
    payload[31] = 0;
    payload[32] = 0;

    pkt[0] = MAVLINK_STX;
    pkt[1] = 33;
    pkt[2] = seq++;
    pkt[3] = 255;
    pkt[4] = 0;
    pkt[5] = MSG_COMMAND_LONG;

    memcpy(pkt + 6, payload, 33);

    crc = 0xFFFF;
    for (int i = 1; i <= 38; i++) {
        crc = crc_acc(pkt[i], crc);
    }
    crc = crc_acc(CRC_EXTRA, crc);

    pkt[39] = (uint8_t)(crc & 0xFF);
    pkt[40] = (uint8_t)(crc >> 8);

    if (sendto(tx_sock, pkt, sizeof(pkt), 0,
               (struct sockaddr *)&tx_addr, sizeof(tx_addr)) < 0) {
        perror("actuator sendto");
        return;
    }

    last_pwm_by_channel[channel] = pwm;
    last_pwm_valid_by_channel[channel] = true;
}

/* ==================== Public Functions ==================== */

void set_ServoAngle(srvSTR *servo, int16_t angle)
{
    uint16_t pwm;

    if (servo == NULL) {
        return;
    }

    servo->ANGLE = clamp_Angle(servo, angle);
    pwm = servo_AngleToPwm(servo);
    send_channel_raw(servo->CHANNEL, pwm);
}

void set_MotorDuty(motSTR *motor, int16_t newDUTY)
{
    uint16_t pwm;

    if (motor == NULL) {
        return;
    }

    motor->DUTY = clamp_Duty(motor, newDUTY);
    pwm = motor_DutyToPwm(motor);
    send_channel_raw(motor->CHANNEL, pwm);
}

void reset_ServoAngle(srvSTR *servo)
{
    if (servo == NULL) {
        return;
    }

    set_ServoAngle(servo, servo->DFLT_ANGLE);
}
