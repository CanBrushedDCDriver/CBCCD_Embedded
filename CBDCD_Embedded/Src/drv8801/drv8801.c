/*
 * drv8801.c
 *
 *  Created on: Jan 2, 2017
 *      Author: Vostro1440
 */

#include "drv8801.h"
#include "main.h"
#include "stm32f0xx_hal.h"

#ifdef DRV8801

static uint8_t drv8801_dir;
static uint32_t drv8801_PWMValue;
static uint32_t drv8801_MaxPower = 60;

void drv8801_Init()
{

}
void drv8801_SetDir(uint8_t u8StepDir)
{
	drv8801_dir = u8StepDir;
	HAL_GPIO_WritePin(DRV_DIR_GPIO_Port, DRV_DIR_Pin ,u8StepDir);
}

extern TIM_HandleTypeDef htim17;
void drv8801_SetPWM(uint32_t u32PWMValue)
{
	if (u32PWMValue > drv8801_MaxPower) u32PWMValue = drv8801_MaxPower;
	drv8801_PWMValue  = u32PWMValue;
	__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,u32PWMValue);
}


#endif
