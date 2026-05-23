#ifndef __TASKS.H__

#include "FreeRTOS.h"
#include "task.h"
#include "MPU6050.h"
#include "stdio.h"
#include "string.h"
#include "main.h"

void StartTask02(void *argument);
void StartTask03(void *argument);
void VOFA_Send_Data(&float pitch, &float roll, &float yaw);
#endif /* __TASKS.H__ */
