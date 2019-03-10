/*
 * rocket.h
 *
 *  Created on: 2019/03/10
 *      Author: feunoir
 */

#ifndef ROCKET_H_
#define ROCKET_H_

#include "stm32l4xx_hal.h"

typedef enum {
	ROCKET_FALSE = 0x00,
	ROCKET_LAUNCHED = 0x01,
	ROCKET_ALLOWEDDEPLOY = 0x02,
	ROCKET_REACHEDAPOGEE = 0x04,
	ROCKET_DEPLOYTIMERELAPSED = 0x08,
	ROCKET_ABLETODEPLOY = 0x10
} Rocket_Status_t;

typedef struct {
	uint32_t tim;
	uint32_t press;
	int16_t temp;
} EnvData_Container_t;

typedef struct {
	EnvData_Container_t envdata[32];
} EnvData_DataSet_t;

typedef struct {
	EnvData_DataSet_t dataset[4];
	uint8_t rocket_status; // Rocket_Status_t�Q��
	uint8_t sdwrite_queue;
} Rocket_Info_t;

void Rocket_Init(Rocket_Info_t info);

void Rocket_InitStatus(Rocket_Info_t info);

void Rocket_ResetQueue(Rocket_Info_t info);
void Rocket_AddQueue(Rocket_Info_t info);

void Rocket_UpdateStatus_Launched(Rocket_Info_t info);
void Rocket_UpdateStatus_AllowDeploy(Rocket_Info_t info);
void Rocket_UpdateStatus_ReachedApogee(Rocket_Info_t info);
void Rocket_UpdateStatus_DeployTimerElapsed(Rocket_Info_t info);

void Rocket_Evaluate_AbleToDeploy(Rocket_Info_t info);

Rocket_Status_t Rocket_isLaunched(Rocket_Info_t info);
Rocket_Status_t Rocket_isAllowedDeploy(Rocket_Info_t info);
Rocket_Status_t Rocket_isReachedApogee(Rocket_Info_t info);
Rocket_Status_t Rocket_isDeployTimerElapsed(Rocket_Info_t info);
Rocket_Status_t Rocket_isAbleToDeploy(Rocket_Info_t info);

uint8_t Rocket_ReadStatus(Rocket_Info_t info, Rocket_Status_t selector);
void Rocket_SetStatus(Rocket_Info_t info, Rocket_Status_t selector);
void Rocket_ResetStatus(Rocket_Info_t info, Rocket_Status_t selector);

void Rocket_EnvData_ShiftDataSet(Rocket_Info_t info);
void Rocket_EnvData_AddNewDataSet(Rocket_Info_t info, EnvData_DataSet_t data);

#endif /* ROCKET_H_ */