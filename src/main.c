/*
===============================================================================
  ████████╗ ██████╗ ██████╗ ██████╗ ███████╗██████╗  ██████╗
  ╚══██╔══╝██╔═══██╗██╔══██╗██╔══██╗██╔════╝██╔══██╗██╔═══██╗
     ██║   ██║   ██║██████╔╝██████╔╝█████╗  ██║  ██║██║   ██║
     ██║   ██║   ██║██╔══██╗██╔═══╝ ██╔══╝  ██║  ██║██║   ██║
     ██║   ╚██████╔╝██║  ██║██║     ███████╗██████╔╝╚██████╔╝
     ╚═╝    ╚═════╝ ╚═╝  ╚═╝╚═╝     ╚══════╝╚═════╝  ╚═════╝

===============================================================================
  Project 60 – Underwater Drone Defence
  Expo Engineering Project

  Platform:
    • Raspberry Pi 3 Model B
    • Pixhawk 4 (MAVLink controlled)

  Description:
    Autonomous / semi-autonomous underwater defence drone control software.
    Handles sensor acquisition, MAVLink communication, actuator control,
    and real-time system feedback.

  Configuration:
    Adjust system parameters and hardware settings in:
      config.h

  Authors:
    Thomas van Bakel  – thomas.vanbakel@student.fontys.nl
    Lucas de Leur     – l.deleur@student.fontys.nl

  Institution:
    Fontys University of Applied Sciences

===============================================================================
*/

#pragma region Includes
// Include all Lib includes here Ex:<stdio.h>
/*============ Pi-Lib Files ============*/
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

// Include the config file here
/*======== Config settings File ========*/
#include "config.h"

// Include Custom modules here Ex:"Your Module.h"
/*=========== Custom Modules ===========*/
#include "common_Control/actuators.h"
#include "common_Control/telemetry.h"
#include "common_Control/transform.h"
#include "common_Control/control.h"
#include "Debug/logger.h"

// Define this script only Defines Ex: #define value true
/*=========== Script Defines ===========*/
#define LISTEN_PORT 14670

/*=========== DRONE CONTROL ===========*/
volatile drone_MAIN TOP_DRONE = {.ARMED = 0};

#pragma region actuator list
/*=============== SERVOS ==============*/
extern srvSTR SERVO_TEST = {
    .MAX_ANGLE = 90,
    .MIN_ANGLE = 0,
    .DFLT_ANGLE = 45,
    .ANGLE = 90,
    .CHANNEL = 8};
#pragma endregion

void EXIT_TASK(int sig)
{
    printf("\nEXIT\n");
    disarmDrone();
    logger_close();
    exit(sig);
}

double get_time_s()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

#pragma region Telemetry Thread
// MAIN TELEMETRY THREAD: This Thread is made for reading and setting Telemetry Values
void *thread_1_Telemetry(void *arg)
{
    while (1)
    {

        TOP_DRONE.gyro_RAD = get_GYRO_V3();
        TOP_DRONE.comps_RAD = get_COMPS_V3();
        TOP_DRONE.compsYAW = get_YAW_HEADING();
        TOP_DRONE.compsPITCH = get_PITCH_HEADING();
        TOP_DRONE.accel_V3 = get_ACCEL_V3();
        TOP_DRONE.gps = get_GPS();
        TOP_DRONE.ACTUALbodyAttitude4D = get_BODY_ATTI4D();

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

        if (sPrintTelemetry)
        {
            printf("\rYaw:%6.2f deg  Pitch:%6.2f deg  |  Gyro X:%6.3f Y:%6.3f Z:%6.3f  |  Accel X:%6.2f  Accel Y:%6.2f  Accel Z:%6.2f",
                   get_YAW_HEADING(),
                   get_PITCH_HEADING(),
                   TOP_DRONE.gyro_RAD.x, TOP_DRONE.gyro_RAD.y, TOP_DRONE.gyro_RAD.z,
                   TOP_DRONE.accel_V3.x, TOP_DRONE.accel_V3.y, TOP_DRONE.accel_V3.z);
            fflush(stdout);
        }
    }
    return NULL;
}
#pragma endregion

#pragma region Control Thread
// MAIN CONTROL THREAD: This Thread is made for controlling the drones actuators
void *thread_2_Control(void *arg)
{
    control_STATES State = ATTACK;
    TOP_DRONE.DESIREDbodyAttitude4D = bodyAttitude4D_create(0.0f, 0.0f, 0.0f, 0.0f);
    float roll_ERROR;
    while (1)
    {

        switch (State)
        {
        case IDLE:
            /* TODO: In IDLE, the drone should wait for a target GPS signal with a direction and velocity,
               as well as a launch signal. When received, it should perform the detachment process
               and then transition to the ATTACK state. */// IDLE -> ATTACK
            break;

        case ATTACK:
        {
            /* TODO: In ATTACK, the drone should maintain a desired depth, keep its roll angle level,
               and hold the correct heading toward the target. After a certain time or when an error
               threshold is reached, it should resurface to check its GPS position and any target updates.
               In ATTACK mode, the drone should also use the nose sensor to look for the target and if found go
               towards it. 
               */// ATTACK <-> RESURFACE s


            float roll_ERROR = TOP_DRONE.DESIREDbodyAttitude4D.r - TOP_DRONE.ACTUALbodyAttitude4D.r;
            if (fabsf(roll_ERROR) >= ROLL_CONT_DEADZONE){
                TOP_DRONE.bodyAtt4D.r = TOP_DRONE.DESIREDbodyAttitude4D.r - TOP_DRONE.ACTUALbodyAttitude4D.r;
                printf("RollControll: %6.2f\n",TOP_DRONE.DESIREDbodyAttitude4D.r - TOP_DRONE.ACTUALbodyAttitude4D.r);
            }

          

 
            break;
        }
        case RESURFACE:
            /* TODO: In RESURFACE, the drone should surface to verify that its GPS position and heading
               are still on course. The drone may also receive updated information about its target. */
            break;

        default:
            State = IDLE; // Force fallback to IDLE
            break;

            


        }
    }
    return NULL;
}
#pragma endregion

#pragma region Sensor Thread
// MAIN SENSOR THREAD: This Thread is made for using and proceccing the Sensors
void *thread_3_Sensors(void *arg)
{
    while (1)
    {
        // TODO: SensorHead code
    }
    return NULL;
}
#pragma endregion

#pragma region Main
int main(void)
{
    logger_init();

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
        // TODO: GetARM Signal Function met true/false return
        // DroneARM() ? TOP_DRONE.ARMED  = true : TOP_DRONE.ARMED  = false;
        usleep(100);
    }

    pthread_t t1, t2;

    pthread_create(&t1, NULL, thread_1_Telemetry, NULL);
    pthread_create(&t2, NULL, thread_2_Control, NULL);
   // pthread_create(&t3 NULL, thread_3_Sensors, NULL);

    pthread_join(t1, NULL);
    pthread_join(t2, NULL);
    //pthread_join(t3, NULL);
    return 0;
}
#pragma endregion