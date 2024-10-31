#include "alarm.h"

#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>


// Control variables
volatile int alarmEnabled = FALSE;
volatile int alarmCount = 0;


void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
}

void setAlarm(int timeout)
{
    // Set alarm function handler
    struct sigaction act = { 0 };
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1) {
        perror("sigaction");
        exit(EXIT_FAILURE);
    }

    
    if (alarmEnabled == FALSE)
    {
        alarm(timeout); // Set alarm to be triggered to a specific timeout
        alarmEnabled = TRUE;
    }
    
    printf("Ending program\n");
}

void resetAlarm()
{
    alarmEnabled = FALSE;
    alarmCount = 0;
    alarm(0);
}
