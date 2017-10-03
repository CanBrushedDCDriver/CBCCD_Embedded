/*
 * uCANnetCommands.c
 *
 *  Created on: 12 lis 2016
 *      Author: Vostro1440
 */
#include "mcuCANCommands.h"
#include "../drv8825/drv8825.h"
#include "../drv8801/drv8801.h"
#include "../motorController/motorController.h"

//--------------------------- CONTROL FRAMES RX BY DRIVER ------------------------------------
typedef struct tCANStepperCMD1 {
	union {
		struct {
			union {
				struct {
					uint32_t stepSize:8;
					uint32_t dir :8;
					uint32_t unused:16;
				};
				uint32_t byte;
			} flags;
			uint32_t stepCount;
		};
		uint8_t data[8];
	};
} CANStepperCMD1;

typedef struct tCANBrushCMD1 {
	union {
		struct {
			struct {
				uint32_t pwm: 16;
				uint32_t breakingOn : 1;
				uint32_t dir : 1;
				uint32_t speedControl: 1;
				uint32_t positionControl: 1;
				uint32_t unused: 12;
			} directControl;
			struct {
				uint16_t position;
				uint16_t speed;
			} regulatorControl;
		};
		uint8_t data[8];
	};
} CANBrushedCMD1;

void switchToBoot_cmd_handler(CanRxMsgTypeDef* CanRxBuffer)
{
	if (CanRxBuffer->Data[0] == 0x11)
	{
		RebootToBootloader();
	}
}

void stepper_step_cmd_handler(CanRxMsgTypeDef* CanRxBuffer)
{
#ifdef DRV8825
	CANStepperCMD1 *cmd = (CANStepperCMD1*)CanRxBuffer->Data;
	drv8825_SetDir(cmd->flags.dir);
	drv8825_SetMicrostepsCount(cmd->flags.stepSize);
	drv8825_SetStepCount(cmd->stepCount);
#endif
#ifdef DRV8801
	CANBrushedCMD1 *cmd = (CANBrushedCMD1*)CanRxBuffer->Data;
	if ((cmd->directControl.speedControl == 0) && (cmd->directControl.positionControl == 0))
	{
		drv8801_SetDir(cmd->directControl.dir);
		drv8801_SetPWM(cmd->directControl.pwm);

		motorController.flags.SpeedControler = 0;
		motorController.flags.PositionControler = 0;
	} else
	{
		motorController.flags.SpeedControler = cmd->directControl.speedControl;
		motorController.flags.PositionControler = cmd->directControl.positionControl;
		motorController.setPosition = cmd->regulatorControl.position;
		motorController.setSpeed = cmd->regulatorControl.speed;
	}
#endif
}

//--------------------------- TX FRAMES ------------------------------------
