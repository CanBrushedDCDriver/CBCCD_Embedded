/*
 * uCANnetCommands.h
 *
 *  Created on: 12 lis 2016
 *      Author: Vostro1440
 */

#ifndef UCANNET_MCUCANCOMMANDS_H_
#define UCANNET_MCUCANCOMMANDS_H_

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_can.h"
#include "../MotorDefinitions.h"

#define STEPPER_STEP_CMD__ID 64
void stepper_step_cmd_handler(CanRxMsgTypeDef* CanRxBuffer);
void switchToBoot_cmd_handler(CanRxMsgTypeDef* CanRxBuffer);

typedef struct tCANStatusFrame1 {
	union {
		struct {
			union {
				struct {
					uint16_t Speed;
					uint16_t Position;
				};
				uint32_t whole;
			} sensors;

#ifdef DRV8825
			union {
				struct {
					uint32_t nowStepping:1;
					uint32_t StepCount:31;
				};
				uint32_t whole;
			} stepper;
#endif
#ifdef DRV8801
		union {
			struct {
				uint32_t state:3; // running, braking
				uint32_t dir : 1;
				uint32_t pwmValue:16;
			};
			uint32_t whole;
		} brushed;
#endif

		};
		uint8_t data[8];
	};
} CANStatusFrame1;



#endif /* UCANNET_MCUCANCOMMANDS_H_ */
