#include "mcuCAN.h"
#include "stm32f0xx_hal_can.h"
#include "uCANnet.h"
#include "uCANNetDefinitions.h"
#include "mcuCANCommands.h"
#include "string.h"
#include "../drv8825/drv8825.h"

volatile tcanRx canRxFlags;
CanTxMsgTypeDef CanTxBuffer;
CanRxMsgTypeDef CanRxBuffer;

tCANfilter slcanFillIdRegister32(tCANFilterFlagsId fl, uint32_t id)
{
	tCANfilter f;
	f.h.reg = 0;
	f.l.reg = 0;

	f.l.f32.RTR = fl.bRTR1;
	f.l.f32.IDE = fl.bExtedned1;
	if (fl.bExtedned1)
	{
		f.l.f32.EXID4_0 = id;
		f.l.f32.EXID12_5 = id >> 5;
		f.h.f32.EXID17_13 = id >> 13;
	} else {
		f.h.f32.STID2_0 = id;
		f.h.f32.STID10_3 = id >> 3;
	}
	return f;
}

extern CAN_HandleTypeDef hcan;

void mcuCANInit()
{
	  hcan.pTxMsg = &CanTxBuffer;
	  hcan.pRxMsg = &CanRxBuffer;
	  canRxFlags.flags.byte = 0;
}

void mcuCANnetFrameTx(uint32_t id, uint8_t len, uint8_t* data)
{
	memcpy(hcan.pTxMsg->Data,data, len);
    hcan.pTxMsg->RTR = 0;
	hcan.pTxMsg->IDE = CAN_ID_EXT;
	hcan.pTxMsg->ExtId = id;
    hcan.pTxMsg->DLC = len;
	HAL_CAN_Transmit(&hcan, 1000);
}

HAL_StatusTypeDef AddFilterCAN(uCANnetID id, uCANnetID mask)
{
	static uint8_t FiltersCount = 0;
	CAN_FilterConfTypeDef sFilterConfig;
	tCANFilterFlagsId fl;
	fl.reg = 0;
	fl.bExtedned1 = 1;

	sFilterConfig.FilterNumber = FiltersCount;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

	tCANfilter f = slcanFillIdRegister32(fl,id.whole);
	sFilterConfig.FilterIdHigh = f.h.reg;
	sFilterConfig.FilterIdLow = f.l.reg;

	f = slcanFillIdRegister32(fl,mask.whole);
	sFilterConfig.FilterMaskIdHigh = f.h.reg;
	sFilterConfig.FilterMaskIdLow = f.l.reg;

	sFilterConfig.FilterFIFOAssignment = 0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.BankNumber = FiltersCount;

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	{
		return HAL_ERROR;
	}
	else
	{
		FiltersCount ++;
		return HAL_OK;
	}
}
