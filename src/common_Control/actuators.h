#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <stdint.h>






/* =========== Structs =========== */
typedef struct
{
    int16_t MAX_ANGLE;
    int16_t MIN_ANGLE;
    int16_t DFLT_ANGLE;

    int16_t ANGLE;

    uint8_t CHANNEL;

} srvSTR;



/* =============================== */


/* ========== Functions ========== */
void set_ServoAngle(srvSTR *servo, int16_t angle);
void reset_ServoAngle(srvSTR *servo);
uint16_t servo_AngleToPwm(const srvSTR *servo);
/* =============================== */
#endif