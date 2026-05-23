#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"
#include "stdio.h"
#include "string.h"

void Motor_Init(void);
void CAN_Read_Motor_Command(void);
void CAN1_Send_Motor_Command(uint8_t motor1_voltage, uint8_t motor2_voltage, uint8_t motor3_voltage, uint8_t motor4_voltage);
void CAN2_Send_Motor_Command(uint8_t motor5_voltage, uint8_t motor6_voltage, uint8_t motor7_voltage);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

typedef struct

#endif /* __MOTOR_H__ */