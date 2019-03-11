/*
 * rocket.c
 *
 *  Created on: 2019/03/10
 *      Author: feunoir
 */
#include "rocket.h"

void Rocket_Init(Rocket_Info_t info) {
	Rocket_InitStatus(info);
	Rocket_ResetQueue(info);
}

void Rocket_InitStatus(Rocket_Info_t info) {
	info.rocket_status = 0x00;
}

uint8_t Rocket_GetQueue(Rocket_Info_t info) {
	return info.sdwrite_queue;
}

void Rocket_ResetQueue(Rocket_Info_t info) {
	info.sdwrite_queue = 0;
}

void Rocket_AddQueue(Rocket_Info_t info) {
	if(info.sdwrite_queue >= 4) {
		info.sdwrite_queue = 4;
	} else {
		info.sdwrite_queue++;
	}
}

void Rocket_UpdateStatus_Launched(Rocket_Info_t info) {
	Rocket_SetStatus(info, ROCKET_LAUNCHED);
}
void Rocket_UpdateStatus_AllowDeploy(Rocket_Info_t info) {
	Rocket_SetStatus(info, ROCKET_ALLOWEDDEPLOY);
}
void Rocket_UpdateStatus_ReachedApogee(Rocket_Info_t info) {
	if(Rocket_isAllowedDeploy(info) == ROCKET_ALLOWEDDEPLOY) {
		Rocket_SetStatus(info, ROCKET_REACHEDAPOGEE);
	}
}
void Rocket_UpdateStatus_DeployTimerElapsed(Rocket_Info_t info) {
	Rocket_SetStatus(info, ROCKET_DEPLOYTIMERELAPSED);
}

void Rocket_Evaluate_AbleToDeploy(Rocket_Info_t info) {
	uint8_t isabletoinitiate =
			(Rocket_isLaunched(info) == ROCKET_LAUNCHED) &
			(Rocket_isAllowedDeploy(info) == ROCKET_ALLOWEDDEPLOY) &
			(
				(Rocket_isReachedApogee(info) == ROCKET_REACHEDAPOGEE) |
				(Rocket_isDeployTimerElapsed(info) == ROCKET_DEPLOYTIMERELAPSED)
			);
	if(isabletoinitiate) {
		Rocket_SetStatus(info, ROCKET_ABLETODEPLOY);
	} else {
		Rocket_ResetStatus(info, ROCKET_ABLETODEPLOY);
	}

}

Rocket_Status_t Rocket_isLaunched(Rocket_Info_t info) {
	return Rocket_ReadStatus(info, ROCKET_LAUNCHED);
}

Rocket_Status_t Rocket_isAllowedDeploy(Rocket_Info_t info) {
	return Rocket_ReadStatus(info, ROCKET_ALLOWEDDEPLOY);
}

Rocket_Status_t Rocket_isReachedApogee(Rocket_Info_t info) {
	return Rocket_ReadStatus(info, ROCKET_ALLOWEDDEPLOY);
}

Rocket_Status_t Rocket_isDeployTimerElapsed(Rocket_Info_t info) {
	return Rocket_ReadStatus(info, ROCKET_DEPLOYTIMERELAPSED);
}

Rocket_Status_t Rocket_isAbleToDeploy(Rocket_Info_t info) {
	return Rocket_ReadStatus(info, ROCKET_ABLETODEPLOY);
}

uint8_t Rocket_ReadStatus(Rocket_Info_t info, Rocket_Status_t selector) {
	return info.rocket_status & selector;
}

void Rocket_SetStatus(Rocket_Info_t info, Rocket_Status_t selector) {
	info.rocket_status |= selector;
}

void Rocket_ResetStatus(Rocket_Info_t info, Rocket_Status_t selector) {
	info.rocket_status &= ~selector;
}

void Rocket_EnvData_ShiftDataSet(Rocket_Info_t info) {
	uint8_t i;
	for(i = 0; i <= 2; i++) {
		info.dataset[3-i] = info.dataset[3-(i+1)];
	}
}

void Rocket_EnvData_AddNewDataSet(Rocket_Info_t info, EnvData_DataSet_t data) {
	info.dataset[0] = data;
}
