/*
 * drv8801.h
 *
 *  Created on: Jan 2, 2017
 *      Author: Vostro1440
 */

#ifndef DRV8801_H_
#define DRV8801_H_

#include "stdint.h"
#include "../MotorDefinitions.h"

#ifdef DRV8801

void drv8801_Init();
void drv8801_SetDir(uint8_t u8StepDir);
void drv8801_SetPWM(uint32_t u32PWMValue);

#endif

#endif /* DRV8801_H_ */
