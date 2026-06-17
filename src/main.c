/*
===============================================================================
  Project 60 – Underwater Drone Defence
===============================================================================
*/

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>

#include "config.h"

#include "common_Control/telemetry.h"
#include "common_Control/control.h"
#include "common_Control/vehicle_config.h"
#include "Debug/logger.h"

volatile drone_MAIN TOP_DRONE = { .ARMED = false };
pthread_t t1, t2;

vehicle_config_t g_vehicle;

static void EXIT_TASK(int sig)//TODO Disarm werkt niet meer 
{
    printf("\nEXIT\n");

    vehicle_set_all_neutral(&g_vehicle);
    disarmDrone();

    if (ENABLELOGGER) {
        logger_close();
    }

    pthread_cancel(t1);
    pthread_cancel(t2);

    sleep(1);
    exit(sig);
}

static double get_time_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static void *thread_1_Telemetry(void *arg)
{
    (void)arg;

    while (1)
    {
        telemetry_poll();

        TOP_DRONE.gyro_RAD = get_GYRO_V3();
        TOP_DRONE.comps_RAD = get_COMPS_V3();
        TOP_DRONE.compsYAW = get_YAW_HEADING();
        TOP_DRONE.compsPITCH = get_PITCH_HEADING();
        TOP_DRONE.accel_V3 = get_ACCEL_V3();
        TOP_DRONE.gps = get_GPS();
        TOP_DRONE.ACTUALbodyAttitude4D = get_BODY_ATTI4D();

        if (ENABLELOGGER)
        {
            logger_begin_row();
            logger_set_double("Tijd", get_time_s());
            logger_set_float("Yaw_deg", TOP_DRONE.compsYAW);
            logger_set_float("Pitch_deg", TOP_DRONE.compsPITCH);
            logger_set_float("Roll_deg", TOP_DRONE.ACTUALbodyAttitude4D.r);
            logger_set_float("Gyro_X", TOP_DRONE.gyro_RAD.x);
            logger_set_float("Gyro_Y", TOP_DRONE.gyro_RAD.y);
            logger_set_float("Gyro_Z", TOP_DRONE.gyro_RAD.z);
            logger_end_row();
        }

        if (sPrintTelemetry)
        {
            printf("\rYaw:%6.2f deg  Pitch:%6.2f deg  Roll:%6.2f deg  AttAge:%5.3f s",
                   TOP_DRONE.compsYAW,
                   TOP_DRONE.compsPITCH,
                   TOP_DRONE.ACTUALbodyAttitude4D.r,
                   telemetry_get_attitude_age_s());
            fflush(stdout);
        }

        usleep(TELEMETRY_LOOP_PERIOD_US);
    }

    return NULL;
}

static void *thread_2_Control(void *arg)
{
    control_STATES State = ATTACK;
    double last_time = get_time_s();

    (void)arg;

    TOP_DRONE.DESIREDbodyAttitude4D = bodyAttitude4D_create(
        0.0f,
        0.0f,
        20.0f,
        0.0f
    );

    while (1)
    {
        double now = get_time_s();
        float dt_s = (float)(now - last_time);
        last_time = now;

        if ((dt_s <= 0.0f) || (dt_s > 0.5f)) {
            dt_s = (float)CONTROL_LOOP_PERIOD_US * 1e-6f;
        }

        switch (State)
        {
            case IDLE:
                vehicle_set_all_neutral(&g_vehicle);
                break;

            case ATTACK:
                if (telemetry_is_attitude_recent(TELEMETRY_ATTITUDE_TIMEOUT_S)) {
                    control_update((drone_MAIN *)&TOP_DRONE, &g_vehicle, dt_s);
                } else {
                    vehicle_set_all_neutral(&g_vehicle);
                }
                break;

            case RESURFACE:
                vehicle_set_all_neutral(&g_vehicle);
                break;

            default:
                State = IDLE;
                break;
        }

        usleep(CONTROL_LOOP_PERIOD_US);
    }

    return NULL;
}

int main(void)
{
    if (ENABLELOGGER) {
        logger_init();
    }

    vehicle_config_init_default(&g_vehicle);

    signal(SIGINT, EXIT_TASK);

    disarmDrone();
    telemetry_request_default_intervals();

    if (AUTOARM)
    {
        printf("WARNING: DRONE WILL AUTO ARM IN 5s...\n");
        sleep(5);
        armDrone();
        TOP_DRONE.ARMED = true;
        printf("DRONE ARMED\n");

        if (ENABLELOGGER)
        {
            logger_begin_row();
            logger_set_double("Tijd", get_time_s());
            logger_set_string("ARMSTATUS", "ARMED");
            logger_end_row();
        }
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