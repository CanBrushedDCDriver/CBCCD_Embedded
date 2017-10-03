/*
 * controllers.h
 *
 *  Created on: 10.01.2017
 *      Author: PLLUJUR1
 */

#include "stdint.h"

#ifndef MOTORCONTROLLER_MOTORCONTROLLER_H_
#define MOTORCONTROLLER_MOTORCONTROLLER_H_

void motorController_Init();
void motorController_Handle();

typedef struct _tMotorControlFlags
{
	uint32_t SpeedControler : 1;
	uint32_t PositionControler : 1;
} tMotorControlFlags;

typedef struct _tMotorController
{
	tMotorControlFlags flags;
	uint16_t setSpeed;
	uint16_t setPosition;
	volatile uint32_t currentPosition;
	volatile uint32_t currentSpeed;
} tMotorController;

extern tMotorController motorController;

#endif /* MOTORCONTROLLER_MOTORCONTROLLER_H_ */
