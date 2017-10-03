#include "motorController.h"
#include "string.h"
#include "stdlib.h"
#include "../MotorDefinitions.h"
#include "../drv8801/drv8801.h"

void motorController_EnablePositionController();
void motorController_EnableSpeedController();
void motorController_DisableControllers();

tMotorController motorController;

void motorController_Init()
{
	memset(&motorController,0,sizeof(motorController));
}

#define HISTEREZIS 5
#define P_GAIN 1

void motorController_Handle()
{
#ifdef DRV8801
	if (motorController.flags.PositionControler)
	{
//		volatile uint32_t tmpCurrentPos = ;
		int32_t deltaPos = (motorController.currentPosition - (int32_t)motorController.setPosition);
		uint32_t absDelataPos = abs(deltaPos);
		if (absDelataPos > HISTEREZIS)
		{
			if (deltaPos > 0)
			{
				drv8801_SetDir(1);
			} else
			{
				drv8801_SetDir(0);
			}
			drv8801_SetPWM(absDelataPos / P_GAIN);
		}
	}
#endif
}

