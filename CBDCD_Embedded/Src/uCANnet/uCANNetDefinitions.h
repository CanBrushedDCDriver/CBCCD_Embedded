/*
 * uCANNetDefinitions.h
 *
 *  Created on: Dec 19, 2016
 *      Author: Vostro1440
 */

#ifndef UCANNETDEFINITIONS_H_
#define UCANNETDEFINITIONS_H_

#include "../MotorDefinitions.h"

// ------------ device types ------------------
#ifdef DRV8825
#define MOTOR_DRIVER_ID 5
#endif

#ifdef DRV8801
#define MOTOR_DRIVER_ID 6
#endif

// ----------------- frames types ------------------
#define MOTOR_CONTROL_FRAME_ID 13

#endif /* UCANNETDEFINITIONS_H_ */
