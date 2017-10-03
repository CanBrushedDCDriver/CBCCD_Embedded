#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_can.h"
#include "uCANnet.h"
#include "mcuCAN.h"
#include "mcuCANCommands.h"
#include "string.h"
#include "uCANNetDefinitions.h"
#include "../motorController/motorController.h"
#include "../drv8825/drv8825.h"

uint8_t deviceGroup = 0;

uint8_t uCANGetDeviceId(void)
{
	static uint8_t uCANid = 0;
	if (uCANid == 0)
	{
		uint8_t *tmp = (uint8_t*)0x1FFFF7AC;
		for (int i = 0; i != 12; i++)
		{
			uCANid += (tmp)[i];
		}

	}
	return uCANid;
}

HAL_StatusTypeDef uCANnetSetFilters()
{

	uCANnetID id, mask;

//	 unicast frame
	mask.whole = 0xFFFFFFFF;
	mask.group = 0; //don't care
	mask.unused = 0;
	mask.frame_type = 0;

	id.type = MOTOR_DRIVER_ID;
	id.id = uCANGetDeviceId();
	id.group = deviceGroup;
	id.mcast = 0;

	AddFilterCAN(id, mask);

	// multicast frame
//	mask.whole = 0xFFFFFFFF;
//	mask.frame_type = 0; // don't care
//	mask.group = 0;
//	mask.type = 0;
//	mask.unused = 0;
//	mask.mcast = 1;
//
//	id.mcast = 0;
//
//	AddFilterCAN(id, mask);
	return HAL_OK;
}

void uCANnetCANInit(void)
{
	mcuCANInit();
}

extern uint8_t deviceGroup;
extern uint32_t drv_pos;
void uCANnetStatusFrameTx(void)
{
	CANStatusFrame1 status;
	memset(&status,0,sizeof(status.data));

	uCANnetID id;
	id.id = uCANGetDeviceId();
	id.frame_type = MOTOR_CONTROL_FRAME_ID;
	id.type = MOTOR_DRIVER_ID;
	id.group = deviceGroup;
	id.mcast = 0;

#ifdef DRV8825
	status.stepper.StepCount = drv8825_GetTotalSteps();
	status.stepper.nowStepping = drv8825_NowStepping();
#endif

	status.sensors.Position = motorController.currentPosition;
	status.sensors.Speed = motorController.currentSpeed;

	mcuCANnetFrameTx(id.whole, 8, status.data);
}


// ---- FRAME RECPTION ----
#define ARRAYSIZE_uCANnetRxHandle 2
uCANnetRxHandle uCANnetRxHandlesArray[ARRAYSIZE_uCANnetRxHandle] = {
		{STEPPER_STEP_CMD__ID, stepper_step_cmd_handler},
		{13, switchToBoot_cmd_handler}
};

// called by CANRX Interrupt, checks if this CAN frame_type is handled by device
void uCANnetFrameRx(CanRxMsgTypeDef* CanRxBuffer)
{
	uint32_t i;
	volatile uCANnetID id;
	id.whole = CanRxBuffer->ExtId;

	for (i = 0; i != ARRAYSIZE_uCANnetRxHandle; i++)
	{
		if (id.frame_type == uCANnetRxHandlesArray[i].frameType)
		{
			uCANnetRxHandlesArray[i].frameTypeRxHandle(CanRxBuffer);
			return;
		}
	}
}

void uCANnetSetGroup(void)
{

}

