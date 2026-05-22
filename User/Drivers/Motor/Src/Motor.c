#include "Motor.h"
#include "can.h"

static MotorFeedback_t g_motor_feedback[MOTOR_FEEDBACK_MAX_DRIVERS];
static MotorFeedback_t g_last_motor_feedback;

static int16_t Motor_ClampVoltage(int16_t voltage)
{
  if (voltage > 25000)
  {
    return 25000;
  }
  if (voltage < -25000)
  {
    return -25000;
  }
  return voltage;
}

static void Motor_PackInt16BE(uint8_t *buf, uint8_t index, int16_t value)
{
  uint16_t raw = (uint16_t)value;
  buf[index] = (uint8_t)(raw >> 8);
  buf[index + 1U] = (uint8_t)(raw & 0xFFU);
}

static uint16_t Motor_ReadUInt16BE(const uint8_t *data, uint8_t index)
{
  return (uint16_t)(((uint16_t)data[index] << 8) | (uint16_t)data[index + 1U]);
}

static int16_t Motor_ReadInt16BE(const uint8_t *data, uint8_t index)
{
  return (int16_t)(((uint16_t)data[index] << 8) | (uint16_t)data[index + 1U]);
}

static void Motor_UpdateFeedback(const CAN_RxHeaderTypeDef *rx_header, const uint8_t *rx_data)
{
  MotorFeedback_t feedback;
  uint32_t driver_id;

  if ((rx_header->IDE != CAN_ID_STD) || (rx_header->RTR != CAN_RTR_DATA) || (rx_header->DLC < 7U))
  {
    return;
  }

  if (rx_header->StdId < 0x205U)
  {
    return;
  }

  driver_id = (uint32_t)(rx_header->StdId - 0x204U);
  if ((driver_id == 0U) || (driver_id >= MOTOR_FEEDBACK_MAX_DRIVERS))
  {
    return;
  }

  feedback.driver_id = (uint8_t)driver_id;
  feedback.mechanical_angle = Motor_ReadUInt16BE(rx_data, 0U);
  feedback.speed = Motor_ReadInt16BE(rx_data, 2U);
  feedback.torque_current = Motor_ReadInt16BE(rx_data, 4U);
  feedback.motor_temperature = rx_data[6];
  feedback.can_id = rx_header->StdId;
  feedback.valid = 1U;

  g_motor_feedback[driver_id] = feedback;
  g_last_motor_feedback = feedback;
}

static void Motor_ReadOneCanMessage(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
  {
    Error_Handler();
  }

  Motor_UpdateFeedback(&rx_header, rx_data);
}

static void Motor_SendGroupCommand(CAN_HandleTypeDef *hcan, uint16_t std_id,
                                   int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint32_t tx_mailbox;
  uint8_t tx_data[8] = {0};

  tx_header.StdId = std_id;
  tx_header.ExtId = 0;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8;
  tx_header.TransmitGlobalTime = DISABLE;

  Motor_PackInt16BE(tx_data, 0U, Motor_ClampVoltage(m1));
  Motor_PackInt16BE(tx_data, 2U, Motor_ClampVoltage(m2));
  Motor_PackInt16BE(tx_data, 4U, Motor_ClampVoltage(m3));
  Motor_PackInt16BE(tx_data, 6U, Motor_ClampVoltage(m4));

  if (HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox) != HAL_OK)
  {
    Error_Handler();
  }
}

void Motor_Init(void)
{
  CAN_FilterTypeDef filter = {0};

  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000U;
  filter.FilterIdLow = 0x0000U;
  filter.FilterMaskIdHigh = 0x0000U;
  filter.FilterMaskIdLow = 0x0000U;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
}

void CAN_Read_Motor_Command(void)
{
  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0U)
  {
    Motor_ReadOneCanMessage(&hcan1);
  }
}

const MotorFeedback_t *Motor_GetFeedback(uint8_t driver_id)
{
  if ((driver_id == 0U) || (driver_id >= MOTOR_FEEDBACK_MAX_DRIVERS))
  {
    return NULL;
  }

  if (g_motor_feedback[driver_id].valid == 0U)
  {
    return NULL;
  }

  return &g_motor_feedback[driver_id];
}

const MotorFeedback_t *Motor_GetLastFeedback(void)
{
  if (g_last_motor_feedback.valid == 0U)
  {
    return NULL;
  }

  return &g_last_motor_feedback;
}

void CAN1_Send_Motor_Command(int16_t motor1_voltage, int16_t motor2_voltage, int16_t motor3_voltage, int16_t motor4_voltage)
{
  Motor_SendGroupCommand(&hcan1, 0x1FFU, motor1_voltage, motor2_voltage, motor3_voltage, motor4_voltage);
}

void CAN2_Send_Motor_Command(int16_t motor5_voltage, int16_t motor6_voltage, int16_t motor7_voltage, int16_t motor8_voltage)
{
  Motor_SendGroupCommand(&hcan1, 0x2FFU, motor5_voltage, motor6_voltage, motor7_voltage, motor8_voltage);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan != &hcan1)
  {
    return;
  }

  Motor_ReadOneCanMessage(hcan);
}
