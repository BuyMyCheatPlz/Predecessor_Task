#include "Remote_Control.h"

#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define SBUS_FRAME_LENGTH 25U
#define SBUS_START_BYTE 0x0FU
#define SBUS_END_BYTE 0x00U
#define SBUS_CHANNEL_CENTER 992
#define SBUS_CHANNEL_MIN 172
#define SBUS_CHANNEL_MAX 1811
#define SBUS_CHANNEL_DEADBAND 20
#define MOTOR_COMMAND_MAX 25000
#define REMOTE_CONTROL_TIMEOUT_MS 50U

/* SBUS flag byte (index 23) bits: bit2 = frame lost, bit3 = failsafe (per SBUS spec) */
#define SBUS_FLAG_BYTE_INDEX 23U
#define SBUS_FLAG_FRAME_LOST 0x04U
#define SBUS_FLAG_FAILSAFE 0x08U

static uint8_t g_sbus_frame[SBUS_FRAME_LENGTH];
static volatile int16_t g_motor2_command = 0;
static volatile int16_t g_motor4_command = 0;
static volatile uint8_t g_command_valid = 0U;
static volatile TickType_t g_last_frame_tick = 0U;

static uint16_t RemoteControl_ReadChannel(const uint8_t *frame, uint8_t channel)
{
  uint32_t bit_offset = (uint32_t)channel * 11U;
  uint8_t byte_index = (uint8_t)(1U + (bit_offset / 8U));
  uint8_t bit_index = (uint8_t)(bit_offset % 8U);
  uint32_t raw;

  raw = (uint32_t)frame[byte_index];
  raw |= ((uint32_t)frame[byte_index + 1U] << 8U);
  raw |= ((uint32_t)frame[byte_index + 2U] << 16U);

  return (uint16_t)((raw >> bit_index) & 0x07FFU);
}

static int16_t RemoteControl_ChannelToCommand(uint16_t channel_value)
{
  int32_t delta = (int32_t)channel_value - SBUS_CHANNEL_CENTER;

  if ((delta > -SBUS_CHANNEL_DEADBAND) && (delta < SBUS_CHANNEL_DEADBAND))
  {
    return 0;
  }

  if (delta > 0)
  {
    delta = (delta * MOTOR_COMMAND_MAX) / (SBUS_CHANNEL_MAX - SBUS_CHANNEL_CENTER);
  }
  else
  {
    delta = (delta * MOTOR_COMMAND_MAX) / (SBUS_CHANNEL_CENTER - SBUS_CHANNEL_MIN);
  }

  if (delta > MOTOR_COMMAND_MAX)
  {
    delta = MOTOR_COMMAND_MAX;
  }
  else if (delta < -MOTOR_COMMAND_MAX)
  {
    delta = -MOTOR_COMMAND_MAX;
  }

  return (int16_t)delta;
}

static void RemoteControl_UpdateCommandsFromFrame(const uint8_t *frame)
{
  if ((frame[0] != SBUS_START_BYTE) || (frame[SBUS_FRAME_LENGTH - 1U] != SBUS_END_BYTE))
  {
    return;
  }

  /* Check SBUS flags for frame lost / failsafe. If failsafe, mark commands invalid and force zero outputs. */
  {
    uint8_t flags = frame[SBUS_FLAG_BYTE_INDEX];
    if ((flags & SBUS_FLAG_FAILSAFE) != 0U)
    {
      g_motor2_command = 0;
      g_motor4_command = 0;
      g_command_valid = 0U;
      g_last_frame_tick = xTaskGetTickCountFromISR();
      return;
    }

    if ((flags & SBUS_FLAG_FRAME_LOST) != 0U)
    {
      /* Treat frame-lost as invalid as well */
      g_motor2_command = 0;
      g_motor4_command = 0;
      g_command_valid = 0U;
      g_last_frame_tick = xTaskGetTickCountFromISR();
      return;
    }
  }

  g_motor2_command = RemoteControl_ChannelToCommand(RemoteControl_ReadChannel(frame, 0U));
  g_motor4_command = RemoteControl_ChannelToCommand(RemoteControl_ReadChannel(frame, 2U));
  g_command_valid = 1U;
  g_last_frame_tick = xTaskGetTickCountFromISR();
}

static void RemoteControl_ArmReceiveDma(void)
{
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, g_sbus_frame, SBUS_FRAME_LENGTH) != HAL_OK)
  {
    Error_Handler();
  }

  if (huart2.hdmarx != NULL)
  {
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
  }
}

void RemoteControl_Init(void)
{
  RemoteControl_Reset();
  g_motor2_command = 0;
  g_motor4_command = 0;
  g_command_valid = 0U;
  g_last_frame_tick = 0U;

  RemoteControl_ArmReceiveDma();
}

bool RemoteControl_GetMotorCommands(int16_t *motor2_command, int16_t *motor4_command)
{
  TickType_t now_tick;

  if ((motor2_command == NULL) || (motor4_command == NULL))
  {
    return false;
  }

  now_tick = xTaskGetTickCount();

  taskENTER_CRITICAL();
  if ((g_command_valid == 0U) ||
      (g_last_frame_tick == 0U) ||
      ((now_tick - g_last_frame_tick) > pdMS_TO_TICKS(REMOTE_CONTROL_TIMEOUT_MS)))
  {
    *motor2_command = 0;
    *motor4_command = 0;
    taskEXIT_CRITICAL();
    return false;
  }

  *motor2_command = g_motor2_command;
  *motor4_command = g_motor4_command;
  taskEXIT_CRITICAL();

  return true;
}

void RemoteControl_Reset(void)
{
  g_motor2_command = 0;
  g_motor4_command = 0;
  g_command_valid = 0U;
  g_last_frame_tick = 0U;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart != &huart2)
  {
    return;
  }

  if (Size == SBUS_FRAME_LENGTH)
  {
    RemoteControl_UpdateCommandsFromFrame(g_sbus_frame);
  }

  RemoteControl_ArmReceiveDma();
}


