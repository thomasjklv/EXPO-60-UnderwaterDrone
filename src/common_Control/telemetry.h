#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include "transform.h"

typedef struct 
{
    bodyAttitude4D bodyAtt4D;
    bodyAttitude4D DESIREDbodyAttitude4D;
    bodyAttitude4D ACTUALbodyAttitude4D;

    bool ARMED;
    vector3 gyro_RAD;
    vector3 comps_RAD;
    vector3 accel_V3;
    float compsYAW;
    float compsPITCH;
    gpsSTR gps;
} drone_MAIN;

/* ================= DRONE ================= */
void armDrone(void);
void disarmDrone(void);
bodyAttitude4D get_BODY_ATTI4D(void);
/* ================= GYRO ================= */
vector3 get_GYRO_V3(void);

/* ================= COMPASS ================= */
vector3 get_COMPS_V3(void);
float get_YAW_HEADING(void);
float get_PITCH_HEADING(void);
float get_ROLL_HEADING(void);
/* ================= Accel ================= */
vector3 get_ACCEL_V3(void);

/* ================= CONVERSIONS ================= */
float rad_to_deg(float rad);    

/* ================= GPS ================= */
gpsSTR get_GPS(void);


#endif