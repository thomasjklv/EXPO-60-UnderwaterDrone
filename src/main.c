/*
===============================================================================
  Project 60 – Underwater Drone Defence
  Simpele bediening voor 2 thrusters met pijltjestoetsen

  Pijltje omhoog = beide thrusters harder
  Pijltje omlaag = beide thrusters zachter
  Spatie / s     = direct stop
  q              = stoppen
===============================================================================
*/

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
#include <stdlib.h>
#include <termios.h>

#include "common/mavlink.h"
#include "config.h"
#include "common_Control/actuators.h"
#include "common_Control/telemetry.h"

#define THRUSTER_STEP_DUTY 5
#define THRUSTER_MIN_DUTY  0
#define THRUSTER_MAX_DUTY  100

static struct termios oude_terminal;
static int terminal_aangepast = 0;
static int huidige_duty = 0;

motSTR mainThruster = {
    .MAX_DUTY = 100,
    .DUTY = 0,
    .CHANNEL = 8
};

motSTR mainThruster_2 = {
    .MAX_DUTY = 100,
    .DUTY = 0,
    .CHANNEL = 7
};

static int begrens(int waarde, int minimum, int maximum)
{
    if (waarde < minimum) {
        return minimum;
    }
    if (waarde > maximum) {
        return maximum;
    }
    return waarde;
}

static void zet_thrusters(int nieuwe_duty)
{
    huidige_duty = begrens(nieuwe_duty, THRUSTER_MIN_DUTY, THRUSTER_MAX_DUTY);

    set_MotorDuty(&mainThruster, huidige_duty);
    set_MotorDuty(&mainThruster_2, huidige_duty);

    printf("\rThrusters: %3d%%   ", huidige_duty);
    fflush(stdout);
}

static void herstel_terminal(void)
{
    if (terminal_aangepast) {
        tcsetattr(STDIN_FILENO, TCSANOW, &oude_terminal);
        terminal_aangepast = 0;
    }
}

static void setup_terminal(void)
{
    struct termios nieuwe_terminal;

    tcgetattr(STDIN_FILENO, &oude_terminal);
    nieuwe_terminal = oude_terminal;

    /* Geen Enter nodig en toetsen worden niet op het scherm geprint. */
    nieuwe_terminal.c_lflag &= ~(ICANON | ECHO);

    tcsetattr(STDIN_FILENO, TCSANOW, &nieuwe_terminal);
    terminal_aangepast = 1;

    atexit(herstel_terminal);
}

static void stop_programma(int sig)
{
    (void)sig;

    zet_thrusters(0);
    disarmDrone();
    herstel_terminal();

    printf("\nGestopt. Thrusters uit en drone disarmed.\n");
    exit(0);
}

int main(void)
{
    int toets;

    signal(SIGINT, stop_programma);
    setup_terminal();

    disarmDrone();
    zet_thrusters(0);

    if (AUTOARM) {
        printf("\nWARNING: DRONE WILL AUTO ARM IN 5s...\n");
        sleep(5);
        armDrone();
        printf("DRONE ARMED\n");
    }

    printf("Gebruik pijltje omhoog/omlaag. Spatie of s = stop. q = afsluiten.\n");

    while (1) {
        toets = getchar();

        /* Pijltjestoetsen sturen 3 tekens: ESC [ A/B */
        if (toets == 27) {
            int toets2 = getchar();
            int toets3 = getchar();

            if (toets2 == '[' && toets3 == 'A') {
                zet_thrusters(huidige_duty + THRUSTER_STEP_DUTY);
            } else if (toets2 == '[' && toets3 == 'B') {
                zet_thrusters(huidige_duty - THRUSTER_STEP_DUTY);
            }
        } else if (toets == ' ' || toets == 's' || toets == 'S') {
            zet_thrusters(0);
        } else if (toets == 'q' || toets == 'Q') {
            stop_programma(0);
        }
    }

    return 0;
}
