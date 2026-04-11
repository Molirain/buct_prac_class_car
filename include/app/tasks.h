#ifndef APP_TASKS_H
#define APP_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

void AppTaskMotor(void *argument);
void AppTaskSensor(void *argument);
void AppTaskCtrl(void *argument);
void AppTaskComm(void *argument);

#ifdef __cplusplus
}
#endif

#endif // APP_TASKS_H
