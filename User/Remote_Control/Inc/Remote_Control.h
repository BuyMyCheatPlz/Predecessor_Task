#ifndef __REMOTE_CONTROL_H__
#define __REMOTE_CONTROL_H__

#include <stdbool.h>
#include <stdint.h>
#include "usart.h"

typedef struct
{
  int16_t motor2_command;
  int16_t motor4_command;
  uint8_t valid;
} RemoteControlCommand_t;

void RemoteControl_Init(void);
bool RemoteControl_GetMotorCommands(int16_t *motor2_command, int16_t *motor4_command);
void RemoteControl_Reset(void);

#endif /* __REMOTE_CONTROL_H__ */
