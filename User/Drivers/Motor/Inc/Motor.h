#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"
#include "stdio.h"
#include "string.h"
#include <stdint.h>

#define MOTOR_FEEDBACK_MAX_DRIVERS 33U

typedef struct
{
	uint8_t driver_id;
	uint16_t mechanical_angle;
	int16_t speed;
	int16_t torque_current;
	uint8_t motor_temperature;
	uint32_t can_id;
	uint8_t valid;
} MotorFeedback_t;

void Motor_Init(void);
void CAN_Read_Motor_Command(void);
void CAN1_Send_Motor_Command(int16_t motor1_voltage, int16_t motor2_voltage, int16_t motor3_voltage, int16_t motor4_voltage);
void CAN2_Send_Motor_Command(int16_t motor5_voltage, int16_t motor6_voltage, int16_t motor7_voltage, int16_t motor8_voltage);
const MotorFeedback_t *Motor_GetFeedback(uint8_t driver_id);
const MotorFeedback_t *Motor_GetLastFeedback(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

#endif /* __MOTOR_H__ */
