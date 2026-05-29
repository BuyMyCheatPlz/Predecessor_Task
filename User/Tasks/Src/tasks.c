#include "../Inc/tasks.h"
#include "MPU6050.h"
#include "main.h"
#include "string.h"
#include "usart.h"
#include "Motor.h"
#include "Remote_Control.h"

// Local copies of sensor angles
static float pitch = 0.0f;
static float roll  = 0.0f;
static float yaw   = 0.0f;

#define MOTOR_CMD_LIMIT 2500
#define MOTOR_CMD_LIMIT 2500

static volatile uint8_t g_vofa_uart4_busy = 0U;
static uint8_t g_vofa_uart4_tx[512];

static int16_t Task_ClampMotorCommand(int16_t cmd)
{
  if (cmd > MOTOR_CMD_LIMIT)
  {
    return MOTOR_CMD_LIMIT;
  }
  if (cmd < -MOTOR_CMD_LIMIT)
  {
    return -MOTOR_CMD_LIMIT;
  }
  return cmd;
}

// Motor2 anti-creep compensation removed

static int VOFA_ReserveBytes(int *position, int required)
{
  if ((*position + required) >= (int)sizeof(g_vofa_uart4_tx))
  {
    return 0;
  }

  return 1;
}

static void VOFA_AppendFloatBinary(int *position, float value)
{
  size_t copy_size = sizeof(float);

  if (VOFA_ReserveBytes(position, (int)copy_size) == 0)
  {
    return;
  }

  memcpy(g_vofa_uart4_tx + *position, &value, copy_size);
  *position += (int)copy_size;
}

void VOFA_Send_Data(float *pitch, float *roll, float *yaw)
{
  int p = 0;
  uint8_t driver_id;

  taskENTER_CRITICAL();
  if (g_vofa_uart4_busy != 0U)
  {
    taskEXIT_CRITICAL();
    return;
  }
  g_vofa_uart4_busy = 1U;
  taskEXIT_CRITICAL();

  /* 如果传入姿态指针则先输出姿态三元组 */
  if (pitch != NULL && roll != NULL && yaw != NULL)
  {
    VOFA_AppendFloatBinary(&p, *pitch);
    VOFA_AppendFloatBinary(&p, *roll);
    VOFA_AppendFloatBinary(&p, *yaw);
  }

  for (driver_id = 1U; driver_id <= 8U; ++driver_id)
  {
    const MotorFeedback_t *feedback = Motor_GetFeedback(driver_id);

    if (feedback == NULL)
    {
      VOFA_AppendFloatBinary(&p, 0.0f);
      VOFA_AppendFloatBinary(&p, 0.0f);
      continue;
    }

    VOFA_AppendFloatBinary(&p, (float)feedback->mechanical_angle);
    VOFA_AppendFloatBinary(&p, (float)feedback->speed);
  }

  if (VOFA_ReserveBytes(&p, 4) == 0)
  {
    g_vofa_uart4_busy = 0U;
    return;
  }

  /* VOFA JustFloat frame tail: 0x00 0x00 0x80 0x7F */
  g_vofa_uart4_tx[p++] = 0x00U;
  g_vofa_uart4_tx[p++] = 0x00U;
  g_vofa_uart4_tx[p++] = 0x80U;
  g_vofa_uart4_tx[p++] = 0x7FU;

  if (HAL_UART_Transmit(&huart4, g_vofa_uart4_tx, (uint16_t)p, 100) != HAL_OK)
  {
    g_vofa_uart4_busy = 0U;
  }
  else
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
  while (MPU6050_DMP_init() != 0)
  {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  for(;;)
  {
    if (MPU6050_DMP_Get_Date(&pitch, &roll, &yaw) == 0)
    {
      VOFA_Send_Data(&pitch, &roll, &yaw);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartTask02 */
}

void StartTask03(void *argument)
{
  Motor_Init();
  RemoteControl_Init();

  for(;;)
  {
    int16_t motor2_command = 0;
    int16_t motor4_command = 0;

    (void)RemoteControl_GetMotorCommands(&motor2_command, &motor4_command);

    /* Apply direct subtraction compensation for motor2 (0x03EF == 1007) */
    motor2_command = (int16_t)(motor2_command - (int16_t)0x03EF);

    motor2_command = Task_ClampMotorCommand(motor2_command);
    motor4_command = Task_ClampMotorCommand(motor4_command);

    CAN1_Send_Motor_Command(0, motor2_command, 0, motor4_command);

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
