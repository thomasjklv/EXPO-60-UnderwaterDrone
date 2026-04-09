/*
===============================================================================
  Project 60 – Underwater Drone Defence
===============================================================================
*/

#pragma region Includes
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <pthread.h>
#include <math.h>
#include <signal.h>
#include <time.h>
#include "common/mavlink.h"
#include "math.h"

#include "config.h"

#include "common_Control/actuators.h"
#include "common_Control/telemetry.h"
#include "common_Control/transform.h"
#include "common_Control/control.h"
#include "common_Control/vehicle_config.h"
#include "common_Control/actuator_model.h"
#include "Debug/logger.h"

#define LISTEN_PORT 14670

volatile drone_MAIN TOP_DRONE = {.ARMED = 0};
pthread_t t1, t2;

vehicle_config_t g_vehicle;

#pragma region actuator list

srvSTR yawFinLeft = {
    .CHANNEL = 1,
    .ANGLE = 0,
    .DFLT_ANGLE = 0,
    .MIN_ANGLE = -30,
    .MAX_ANGLE = 30,
    .MIN_PWM = 1000,
    .MAX_PWM = 2000
};

srvSTR yawFinRight = {
    .CHANNEL = 2,
    .ANGLE = 0,
    .DFLT_ANGLE = 0,
    .MIN_ANGLE = -30,
    .MAX_ANGLE = 30,
    .MIN_PWM = 1000,
    .MAX_PWM = 2000
};

srvSTR pitchFinLeft = {
    .CHANNEL = 3,
    .ANGLE = 0,
    .DFLT_ANGLE = 0,
    .MIN_ANGLE = -30,
    .MAX_ANGLE = 30,
    .MIN_PWM = 1000,
    .MAX_PWM = 2000
};

srvSTR pitchFinRight = {
    .CHANNEL = 4,
    .ANGLE = 0,
    .DFLT_ANGLE = 0,
    .MIN_ANGLE = -30,
    .MAX_ANGLE = 30,
    .MIN_PWM = 1000,
    .MAX_PWM = 2000
};

srvSTR rollFinLeft = {
    .CHANNEL = 5,
    .ANGLE = 0,
    .DFLT_ANGLE = 0,
    .MIN_ANGLE = -30,
    .MAX_ANGLE = 30,
    .MIN_PWM = 1000,
    .MAX_PWM = 2000
};

srvSTR rollFinRight = {
    .CHANNEL = 8,
    .ANGLE = 0,
    .DFLT_ANGLE = 0,
    .MIN_ANGLE = -30,
    .MAX_ANGLE = 30,
    .MIN_PWM = 1000,
    .MAX_PWM = 2000
};

motSTR mainThruster = {
    .MAX_DUTY = 100,
    .DUTY = 0,
    .CHANNEL = 7
};
#pragma endregion

void EXIT_TASK(int sig)
{
    printf("\nEXIT\n");
    vehicle_set_all_neutral(&g_vehicle);
    disarmDrone();
    logger_close();
    pthread_cancel(t1);
    pthread_cancel(t2);

    sleep(1);
    exit(sig);
}

double get_time_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

#pragma region Telemetry Thread
void *thread_1_Telemetry(void *arg)
{
    (void)arg;

    while (1)
    {
        TOP_DRONE.gyro_RAD = get_GYRO_V3();
        TOP_DRONE.comps_RAD = get_COMPS_V3();
        TOP_DRONE.compsYAW = get_YAW_HEADING();
        TOP_DRONE.compsPITCH = get_PITCH_HEADING();
        TOP_DRONE.accel_V3 = get_ACCEL_V3();
        TOP_DRONE.gps = get_GPS();
        TOP_DRONE.ACTUALbodyAttitude4D = get_BODY_ATTI4D();

        {
            double t = get_time_s();

            logger_begin_row();
            logger_set_double("Tijd", t);
            logger_set_float("Yaw_deg", TOP_DRONE.compsYAW);
            logger_set_float("Pitch_deg", TOP_DRONE.compsPITCH);

            logger_set_float("Gyro_X", TOP_DRONE.gyro_RAD.x);
            logger_set_float("Gyro_Y", TOP_DRONE.gyro_RAD.y);
            logger_set_float("Gyro_Z", TOP_DRONE.gyro_RAD.z);

            logger_set_float("Compass_X", TOP_DRONE.comps_RAD.x);
            logger_set_float("Compass_Y", TOP_DRONE.comps_RAD.y);
            logger_set_float("Compass_Z", TOP_DRONE.comps_RAD.z);

            logger_set_float("Accel_X", TOP_DRONE.accel_V3.x);
            logger_set_float("Accel_Y", TOP_DRONE.accel_V3.y);
            logger_set_float("Accel_Z", TOP_DRONE.accel_V3.z);
            logger_end_row();
        }

        if (sPrintTelemetry)
        {
            printf("\rYaw:%6.2f deg  Pitch:%6.2f deg  Roll:%6.2f deg",
                   get_YAW_HEADING(),
                   get_PITCH_HEADING(),
                   get_ROLL_HEADING());
            fflush(stdout);
        }

        usleep(10000);
    }

    return NULL;
}
#pragma endregion

#pragma region Control Thread
void *thread_2_Control(void *arg)
{
    control_STATES State = ATTACK;
    double last_time = get_time_s();

    (void)arg;

    TOP_DRONE.DESIREDbodyAttitude4D = bodyAttitude4D_create(
        0.0f,   /* desired yaw deg   */
        0.0f,   /* desired pitch deg */
        20.0f,  /* desired surge force request in N */
        0.0f    /* desired roll deg  */
    );

    while (1)
    {
        double now = get_time_s();
        float dt_s = (float)(now - last_time);
        last_time = now;

        usleep(10000);

        switch (State)
        {
            case IDLE:
                vehicle_set_all_neutral(&g_vehicle);
                break;

            case ATTACK:
                control_update((drone_MAIN *)&TOP_DRONE, &g_vehicle, dt_s);
                break;

            case RESURFACE:
                vehicle_set_all_neutral(&g_vehicle);
                break;

            default:
                State = IDLE;
                break;
        }
    }

    return NULL;
}
#pragma endregion

#pragma region Main
int main(void)
{
    logger_init();

    vehicle_config_init_default(&g_vehicle,
                                &yawFinLeft,
                                &yawFinRight,
                                &pitchFinLeft,
                                &pitchFinRight,
                                &rollFinLeft,
                                &rollFinRight,
                                &mainThruster);

    signal(SIGINT, EXIT_TASK);
    disarmDrone();

    if (AUTOARM)
    {
        printf("WARNING: DRONE WILL AUTO ARM IN 5s...\n");
        sleep(5);
        armDrone();
        TOP_DRONE.ARMED = true;
        printf("DRONE ARMED\n");
        logger_begin_row();
        logger_set_double("Tijd", get_time_s());
        logger_set_string("ARMSTATUS", "ARMED");
        logger_end_row();
    }

    while (!TOP_DRONE.ARMED)
    {
        usleep(100);
    }

    pthread_create(&t1, NULL, thread_1_Telemetry, NULL);
    pthread_create(&t2, NULL, thread_2_Control, NULL);

    pthread_join(t1, NULL);
    pthread_join(t2, NULL);
    return 0;
}
#pragma endregion
