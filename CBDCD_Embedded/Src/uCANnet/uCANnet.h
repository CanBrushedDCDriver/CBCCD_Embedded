/*
 * uCANnet.h
 *
 *  Created on: 11 lis 2016
 *      Author: Vostro1440
 */
#ifndef UCANNET_UCANNET_H_
#define UCANNET_UCANNET_H_

#include "main.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_can.h"

typedef union {
		struct{
			uint32_t frame_type : 8; // [0-254] frame type, type specific
			uint32_t type: 8; //[0 - 254] device type
			uint32_t id : 8;   // [0-254] unique device id in network for this device type
			uint32_t mcast : 1; // [0-1] multicast frame for sending one frames to all devices of the same type in group
			uint32_t group : 3; // [0-8] group number
			uint32_t unused : 1;
		};
		uint32_t whole; //29bits extended
	} uCANnetID;

typedef struct
{
	uint8_t frameType;
	void (*frameTypeRxHandle)(CanRxMsgTypeDef* CanRxBuffer);
}uCANnetRxHandle;

HAL_StatusTypeDef uCANnetSetFilters();
void uCANnetFrameRx(CanRxMsgTypeDef* CanRxBuffer);
void uCANnetStatusFrameTx(void);
void uCANnetSetGroup(void);
void uCANnetCANInit(void);
uint8_t uCANGetDeviceId(void);

#endif /* UCANNET_UCANNET_H_ */
