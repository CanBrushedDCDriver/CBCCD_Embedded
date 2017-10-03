#include "drv8825.h"

#include "main.h"
#include "stm32f0xx_hal.h"

#ifdef DRV8825

static uint8_t drv8825_microsteps = 0;
static uint32_t StepCount = 0;
static uint32_t totalStepsCount = 0;
static uint8_t drv8825_dir;

void drv8825_Reset(uint8_t Reset)
{
	HAL_GPIO_WritePin(DRV_nRESET_GPIO_Port, DRV_nRESET_Pin ,Reset);
}

void drv8825_EnableDevice(uint8_t Enable)
{
	HAL_GPIO_WritePin(DRV_nENABLE_GPIO_Port, DRV_nENABLE_Pin ,Enable);
}

void drv8825_SetSleep(uint8_t Sleep)
{
	HAL_GPIO_WritePin(DRV_nSLEEP_GPIO_Port, DRV_nSLEEP_Pin ,Sleep);
}


void drv8825_SetStepCount(uint32_t vStepCount)
{
	StepCount = vStepCount;
}

uint32_t drv8825_GetTotalSteps()
{
	return totalStepsCount;
}
uint8_t drv8825_NowStepping()
{
	return (StepCount != 0);
}

uint8_t drv8825_PerformStep()
{
	volatile uint8_t i = 0;
	if (StepCount != 0)
	{
		StepCount --;
		// step is performed at Rising egde
		HAL_GPIO_WritePin(DRV_STEP_GPIO_Port, DRV_STEP_Pin ,1);
		for (i = 0; i != 5; i ++);

		HAL_GPIO_WritePin(DRV_STEP_GPIO_Port, DRV_STEP_Pin ,0);

		totalStepsCount += (32 >> drv8825_microsteps);
		return 1;
	}
	return 0;
}


void drv8825_SetMicrostepsCount(tMicrostepsCount microsteps)
{
	drv8825_microsteps = microsteps;
	HAL_GPIO_WritePin(DRV_MS1_GPIO_Port, DRV_MS1_Pin ,microsteps&0xFE);
	HAL_GPIO_WritePin(DRV_MS2_GPIO_Port, DRV_MS2_Pin ,(microsteps>1)&0xFE);
	HAL_GPIO_WritePin(DRV_MS3_GPIO_Port, DRV_MS3_Pin ,(microsteps>2)&0xFE);
}


void drv8825_SetDir(uint8_t u8StepDir)
{
	drv8825_dir = u8StepDir;
	HAL_GPIO_WritePin(DRV_DIR_GPIO_Port, DRV_DIR_Pin ,u8StepDir);
}


#endif

