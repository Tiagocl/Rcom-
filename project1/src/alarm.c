#include "alarm.h"

#include <unistd.h>
#include <signal.h>
#include <stdio.h>

volatile int alarmEnabled;
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
    (void)signal(SIGALRM, alarmHandler);

    
    if (alarmEnabled == FALSE)
    {
        alarm(timeout); // Set alarm to be triggered to a specific timeout
        alarmEnabled = TRUE;
    }
    
    printf("Ending program\n");
}