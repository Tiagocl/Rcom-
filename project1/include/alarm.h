#ifndef _ALARM_H_
#define _ALARM_H_

#define FALSE 0
#define TRUE 1

extern volatile int alarmEnabled;
extern volatile int alarmCount;

// Alarm function handler
void alarmHandler(int signal);

void setAlarm(int timeout);

#endif // _ALARM_H_
