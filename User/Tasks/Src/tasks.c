#include "../Inc/tasks.h"
#include "MPU6050.h"
#include "main.h"
#include "string.h"
#include "stdio.h"
#include "usart.h"
#include "Motor.h"
#include "Remote_Control.h"
#include "../../Utils/Inc/justfloat.h"

// Local copies of sensor angles
static float pitch = 0.0f;
static float roll  = 0.0f;
static float yaw   = 0.0f;

#define MOTOR_FEEDBACK_PRINT_COUNT 8U

static uint8_t g_vofa_uart4_busy = 0U;
static uint8_t g_vofa_uart4_tx[512];

static void VOFA_AppendFloat(char *data, int *position, float value)
{
  char tmp[24];

  jf_ftoa(value, tmp, 2);
  memcpy(data + *position, tmp, strlen(tmp));
  *position += (int)strlen(tmp);
}

void VOFA_Send_Data(float *pitch, float *roll, float *yaw)
{
  int p = 0;

  /* 如果正在发送，直接返回 */
  if (g_vofa_uart4_busy != 0U)
  {
    return;
  }

  /* 如果传入姿态指针则先输出姿态三元组 */
  if (pitch != NULL && roll != NULL && yaw != NULL)
  {
    VOFA_AppendFloat((char *)g_vofa_uart4_tx, &p, *pitch);
    g_vofa_uart4_tx[p++] = ',';
    VOFA_AppendFloat((char *)g_vofa_uart4_tx, &p, *roll);
    g_vofa_uart4_tx[p++] = ',';
    VOFA_AppendFloat((char *)g_vofa_uart4_tx, &p, *yaw);
    g_vofa_uart4_tx[p++] = ',';
  }

  for (uint8_t driver_id = 1U; driver_id <= MOTOR_FEEDBACK_PRINT_COUNT; ++driver_id)
  {
    const MotorFeedback_t *feedback = Motor_GetFeedback(driver_id);

    if (driver_id > 1U)
    {
      g_vofa_uart4_tx[p++] = ',';
    }

    if (feedback == NULL)
    {
      VOFA_AppendFloat((char *)g_vofa_uart4_tx, &p, 0.0f);
      g_vofa_uart4_tx[p++] = ',';
      VOFA_AppendFloat((char *)g_vofa_uart4_tx, &p, 0.0f);
      g_vofa_uart4_tx[p++] = ',';
      VOFA_AppendFloat((char *)g_vofa_uart4_tx, &p, 0.0f);
      g_vofa_uart4_tx[p++] = ',';
      VOFA_AppendFloat((char *)g_vofa_uart4_tx, &p, 0.0f);
      continue;
    }

    VOFA_AppendFloat((char *)g_vofa_uart4_tx, &p, (float)feedback->mechanical_angle);
    g_vofa_uart4_tx[p++] = ',';
    VOFA_AppendFloat((char *)g_vofa_uart4_tx, &p, (float)feedback->speed);
    g_vofa_uart4_tx[p++] = ',';
    VOFA_AppendFloat((char *)g_vofa_uart4_tx, &p, (float)feedback->torque_current);
    g_vofa_uart4_tx[p++] = ',';
    VOFA_AppendFloat((char *)g_vofa_uart4_tx, &p, (float)feedback->motor_temperature);
  }

  memcpy(g_vofa_uart4_tx + p, "\r\n", 2);
  p += 2;

  g_vofa_uart4_busy = 1U;
  if (HAL_UART_Transmit_DMA(&huart4, g_vofa_uart4_tx, (uint16_t)p) != HAL_OK)
  {
    g_vofa_uart4_busy = 0U;
  }
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

void StartTask03(void *argument)
{
  Motor_Init();
  RemoteControl_Init();

  TickType_t last_feedback_print_tick = xTaskGetTickCount();

  for(;;)
  {
    int16_t motor2_command = 0;
    int16_t motor4_command = 0;

    (void)RemoteControl_GetMotorCommands(&motor2_command, &motor4_command);

    CAN1_Send_Motor_Command(0, motor2_command, 0, motor4_command);

    if ((xTaskGetTickCount() - last_feedback_print_tick) >= pdMS_TO_TICKS(50))
    {
      VOFA_Send_Data(NULL, NULL, NULL);
      last_feedback_print_tick = xTaskGetTickCount();
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart4)
  {
    g_vofa_uart4_busy = 0U;
  }
}
