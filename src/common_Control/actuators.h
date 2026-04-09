#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <stdint.h>

/* =========== Structs =========== */
typedef struct
{
    uint8_t  CHANNEL;
    int16_t  ANGLE;
    int16_t  DFLT_ANGLE;
    int16_t  MIN_ANGLE;
    int16_t  MAX_ANGLE;
    uint16_t MIN_PWM;
    uint16_t MAX_PWM;
} srvSTR;

typedef struct
{
    uint8_t CHANNEL;
    int16_t DUTY;
    int16_t MAX_DUTY;
} motSTR;
/* =============================== */

/* ========== Functions ========== */
void set_ServoAngle(srvSTR *servo, int16_t angle);
void set_MotorDuty(motSTR *motor, int16_t newDUTY);
void reset_ServoAngle(srvSTR *servo);
uint16_t servo_AngleToPwm(const srvSTR *servo);
uint16_t motor_DutyToPwm(const motSTR *motor);
/* =============================== */

#endif
