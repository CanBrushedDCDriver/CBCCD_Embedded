#ifndef DRV8825_DRV8825_H_
#define DRV8825_DRV8825_H_

#include "stdint.h"
#include "../MotorDefinitions.h"

#ifdef DRV8825

uint8_t drv8825_NowStepping();
uint32_t drv8825_GetTotalSteps();
void drv8825_SetStepCount(uint32_t vStepCount);
void drv8825_StepSize(uint8_t Size);
void drv8825_Reset(uint8_t Reset);
void drv8825_EnableDevice(uint8_t Enable);
void drv8825_SetSleep(uint8_t Sleep);
uint8_t drv8825_PerformStep();
void drv8825_SetMicrostepsCount(tMicrostepsCount microsteps); // step divider
void drv8825_SetDir(uint8_t u8StepDir);

#endif

#endif /* DRV8825_DRV8825_H_ */
