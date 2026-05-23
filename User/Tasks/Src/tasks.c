#include "../Inc/tasks.h"
#include "MPU6050.h"
#include "main.h"
#include "string.h"
#include "stdio.h"
#include "usart.h"
#include "../../Utils/Inc/justfloat.h"

// Local copies of sensor angles
static float pitch = 0.0f;
static float roll  = 0.0f;
static float yaw   = 0.0f;

void VOFA_Send_Data(float *pitch, float *roll, float *yaw)
{
  char data[80];
  char tmp[24];
  int p = 0;
  const char *pre1 = "Pitch: ";
  const char *pre2 = ", Roll: ";
  const char *pre3 = ", Yaw: ";
  memcpy(data + p, pre1, strlen(pre1)); p += strlen(pre1);
  jf_ftoa(*pitch, tmp, 2);
  memcpy(data + p, tmp, strlen(tmp)); p += strlen(tmp);
  memcpy(data + p, pre2, strlen(pre2)); p += strlen(pre2);
  jf_ftoa(*roll, tmp, 2);
  memcpy(data + p, tmp, strlen(tmp)); p += strlen(tmp);
  memcpy(data + p, pre3, strlen(pre3)); p += strlen(pre3);
  jf_ftoa(*yaw, tmp, 2);
  memcpy(data + p, tmp, strlen(tmp)); p += strlen(tmp);
  memcpy(data + p, "\r\n", 2); p += 2;
  HAL_UART_Transmit(&huart1, (uint8_t*)data, p, HAL_MAX_DELAY);
}

void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 10;
  MPU6050_DMP_init();
  for(;;)
  {
    MPU6050_DMP_Get_Date(&pitch, &roll, &yaw);
    VOFA_Send_Data(&pitch, &roll, &yaw);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartTask02 */
}