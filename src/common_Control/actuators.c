#include "common_Control/actuators.h"
#include "config.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

/* Manual MAVLink v1 packet settings */
#define MAVLINK_STX           0xFE
#define MSG_COMMAND_LONG      76
#define MAV_CMD_DO_SET_SERVO  183
#define CRC_EXTRA             152

static uint8_t seq = 0;

/* ==================== Internal Helpers ==================== */

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

    if (duty < 0) {
        return 0;
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

    int16_t angle = clamp_Angle(servo, servo->ANGLE);
    int16_t angleRange = servo->MAX_ANGLE - servo->MIN_ANGLE;
    int16_t pwmRange   = servo->MAX_PWM   - servo->MIN_PWM;

    if (angleRange <= 0) {
        return 1500;
    }

    if (pwmRange <= 0) {
        return 1500;
    }

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

uint16_t motor_DutyToPwm(const motSTR *motor)
{
    if (motor == NULL) {
        return 1000;
    }

    int16_t duty = clamp_Duty(motor, motor->DUTY);
    int16_t maxDuty = motor->MAX_DUTY;

    if (maxDuty <= 0) {
        return 1000;
    }

    uint32_t pwm = 1000U + ((uint32_t)duty * 1000U) / (uint32_t)maxDuty;

    if (pwm < 1000U) {
        pwm = 1000U;
    }

    if (pwm > 2000U) {
        pwm = 2000U;
    }

    return (uint16_t)pwm;
}

static void send_channel_raw(uint8_t channel, float pwm)
{
    uint8_t payload[33];
    uint8_t pkt[41];
    uint16_t crc;
    uint16_t cmd = MAV_CMD_DO_SET_SERVO;
    float p1 = (float)channel;

    memset(payload, 0, sizeof(payload));
    memset(pkt, 0, sizeof(pkt));

    memcpy(payload + 0,  &p1,  4);
    memcpy(payload + 4,  &pwm, 4);
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

    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) {
        perror("socket");
        return;
    }

    struct sockaddr_in a;
    memset(&a, 0, sizeof(a));
    a.sin_family = AF_INET;
    a.sin_port = htons(ACTUATOR_PORT);

    if (inet_pton(AF_INET, ACTUATOR_HOST, &a.sin_addr) != 1) {
        perror("inet_pton");
        close(s);
        return;
    }

    if (sendto(s, pkt, sizeof(pkt), 0, (struct sockaddr *)&a, sizeof(a)) < 0) {
        perror("sendto");
    }

    close(s);
}

/* ==================== Public Functions ==================== */

void set_ServoAngle(srvSTR *servo, int16_t angle)
{
    if (servo == NULL) {
        return;
    }

    servo->ANGLE = clamp_Angle(servo, angle);

    uint16_t pwm = servo_AngleToPwm(servo);
    send_channel_raw(servo->CHANNEL, (float)pwm);
}

void set_MotorDuty(motSTR *motor, int16_t newDUTY)
{
    if (motor == NULL) {
        return;
    }

    motor->DUTY = clamp_Duty(motor, newDUTY);

    uint16_t pwm = motor_DutyToPwm(motor);
    send_channel_raw(motor->CHANNEL, (float)pwm);
}

void reset_ServoAngle(srvSTR *servo)
{
    if (servo == NULL) {
        return;
    }

    set_ServoAngle(servo, servo->DFLT_ANGLE);
}
