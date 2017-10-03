/*
 * mcutypedef.h
 *
 *  Created on: 11 lis 2016
 *      Author: Vostro1440
 */
#ifndef UCANNET_MCUCAN_H_
#define UCANNET_MCUCAN_H_

#include "main.h"
#include "stm32f0xx_hal.h"
#include "uCANnet.h"

typedef struct {
		uint16_t  EXID17_15 : 3;
		uint16_t  RTR : 1;
		uint16_t  IDE : 1;
		uint16_t  STID2_0 : 3;

		uint16_t  STID10_3 : 8;
}tCANreg16;

typedef union{
	struct {
		uint32_t bRTR1 : 1;
		uint32_t bExtedned1 : 1;
		uint32_t bRTR2 : 1;
		uint32_t bExtedned2 : 1;
	};
	uint32_t reg;
} tCANFilterFlagsId;

typedef struct {
	union {
		struct {
			uint16_t EXID17_13 : 5;
			uint16_t STID2_0   : 3;
			uint16_t STID10_3  : 8;
		} f32;
		uint16_t reg;
		tCANreg16 f16;
	}h;
	union {
		struct {
			uint16_t reserved : 1;
			uint16_t RTR : 1;
			uint16_t IDE : 1;
			uint16_t EXID4_0   : 5;
			uint16_t EXID12_5  : 8;
		} f32;
		uint16_t reg;
		tCANreg16 f16;
	}l;
}tCANfilter;

typedef struct tcanRxFlags {
	union {
		struct {
			uint8_t fifo1 :1;
			uint8_t fifo2 :1;
		};
		uint8_t byte;
	} flags;
	uint8_t activefifo;
} tcanRx;

HAL_StatusTypeDef AddFilterCAN(uCANnetID id, uCANnetID mask);
void mcuCANnetFrameTx(uint32_t id, uint8_t len, uint8_t* data);
void mcuCANInit(void);

#endif /* UCANNET_MCUCAN_H_ */
