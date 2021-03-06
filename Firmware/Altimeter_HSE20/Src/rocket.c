/*
 * rocket.c
 *
 *  Created on: 2019/03/10
 *      Author: feunoir
 */
#include "rocket.h"

void Rocket_Init(Rocket_Info_t* info) {
	Rocket_InitStatus(info);
	Rocket_ResetQueue(info);
}

void Rocket_InitStatus(Rocket_Info_t* info) {
	info->rocket_status = 0x00;
}

uint8_t Rocket_GetStatus(Rocket_Info_t* info) {
	return info->rocket_status;
}

uint8_t Rocket_GetQueue(Rocket_Info_t* info) {
	return info->sdwrite_queue;
}

void Rocket_ResetQueue(Rocket_Info_t* info) {
	info->sdwrite_queue = 0;
}

void Rocket_AddQueue(Rocket_Info_t* info) {
	if(info->sdwrite_queue >= 4) {
		info->sdwrite_queue = 4;
	} else {
		info->sdwrite_queue++;
	}
}

void Rocket_UpdateStatus_Launched(Rocket_Info_t* info) {
	Rocket_SetStatus(info, ROCKET_LAUNCHED);
}
void Rocket_UpdateStatus_AllowDeploy(Rocket_Info_t* info) {
	Rocket_SetStatus(info, ROCKET_ALLOWEDDEPLOY);
}
void Rocket_UpdateStatus_ReachedApogee(Rocket_Info_t* info) {
	if(Rocket_isAllowedDeploy(info) == ROCKET_ALLOWEDDEPLOY) {
		Rocket_SetStatus(info, ROCKET_REACHEDAPOGEE);
	}
}
void Rocket_UpdateStatus_DeployTimerElapsed(Rocket_Info_t* info) {
	Rocket_SetStatus(info, ROCKET_DEPLOYTIMERELAPSED);
}
void Rocket_UpdateStatus_ReachedThresholdAlt(Rocket_Info_t* info) {
	if(Rocket_isAbleToDeploy_1stStage(info) == ROCKET_ABLETODEPLOY_1STSTAGE) {
		Rocket_SetStatus(info, ROCKET_REACHEDTHRESHOLDALT);
	}
}

//	niwa
void Rocket_Evaluate_ReachedApogee(Rocket_Info_t* info){
	float sum = 0,old_sum=0;
	float average, old_average;
	for(uint8_t i=0;i<32;i++){
		sum += info->dataset[0].envdata[i].press/4096.0;
		old_sum += info->dataset[1].envdata[i].press/4096.0;
	}
	average = sum/32.0;
	old_average = old_sum/32.0;
	if(average > old_average){
		Rocket_UpdateStatus_ReachedApogee(info);
	}
}
void Rocket_Evaluate_ReachedThresholdAlt(Rocket_Info_t* info){
	float sum = 0;
	float ground_press;
	float height;
	for(uint8_t i=0;i<32;i++){
		sum += info->ground.envdata[i].press/4096.0;
	}
	ground_press = sum/32.0;
	height = (ground_press - info->dataset[0].envdata[31].press/4096.0)*8.28;
	if(height < 200.0){
		Rocket_UpdateStatus_ReachedThresholdAlt(info);
	}
}
//	/niwa

void Rocket_Evaluate_AbleToDeploy_1stStage(Rocket_Info_t* info) {
	uint8_t isabletoinitiate =
			(Rocket_isLaunched(info) == ROCKET_LAUNCHED) &
			(Rocket_isAllowedDeploy(info) == ROCKET_ALLOWEDDEPLOY) &
			(
				(Rocket_isReachedApogee(info) == ROCKET_REACHEDAPOGEE) |
				(Rocket_isDeployTimerElapsed(info) == ROCKET_DEPLOYTIMERELAPSED)
			);
	if(isabletoinitiate) {
		Rocket_SetStatus(info, ROCKET_ABLETODEPLOY_1STSTAGE);
	} else {
		Rocket_ResetStatus(info, ROCKET_ABLETODEPLOY_1STSTAGE);
	}

}

void Rocket_Evaluate_AbleToDeploy_2ndStage(Rocket_Info_t* info) {
	uint8_t isabletoinitiate =
			(Rocket_isAbleToDeploy_1stStage(info) == ROCKET_ABLETODEPLOY_1STSTAGE) &
			(Rocket_isReachedThresholdAlt(info) == ROCKET_REACHEDTHRESHOLDALT);
	if(isabletoinitiate) {
		Rocket_SetStatus(info, ROCKET_ABLETODEPLOY_2NDSTAGE);
	} else {
		Rocket_ResetStatus(info, ROCKET_ABLETODEPLOY_2NDSTAGE);
	}
}

Rocket_Status_t Rocket_isLaunched(Rocket_Info_t* info) {
	return Rocket_ReadStatus(info, ROCKET_LAUNCHED);
}

Rocket_Status_t Rocket_isAllowedDeploy(Rocket_Info_t* info) {
	return Rocket_ReadStatus(info, ROCKET_ALLOWEDDEPLOY);
}

Rocket_Status_t Rocket_isReachedApogee(Rocket_Info_t* info) {
	return Rocket_ReadStatus(info, ROCKET_ALLOWEDDEPLOY);
}

Rocket_Status_t Rocket_isDeployTimerElapsed(Rocket_Info_t* info) {
	return Rocket_ReadStatus(info, ROCKET_DEPLOYTIMERELAPSED);
}

Rocket_Status_t Rocket_isAbleToDeploy_1stStage(Rocket_Info_t* info) {
	return Rocket_ReadStatus(info, ROCKET_ABLETODEPLOY_1STSTAGE);
}
Rocket_Status_t Rocket_isReachedThresholdAlt(Rocket_Info_t* info) {
	return Rocket_ReadStatus(info, ROCKET_REACHEDTHRESHOLDALT);
}
Rocket_Status_t Rocket_isAbleToDeploy_2ndStage(Rocket_Info_t* info) {
	return Rocket_ReadStatus(info, ROCKET_ABLETODEPLOY_2NDSTAGE);
}

uint8_t Rocket_ReadStatus(Rocket_Info_t* info, Rocket_Status_t selector) {
	return info->rocket_status & selector;
}

void Rocket_SetStatus(Rocket_Info_t* info, Rocket_Status_t selector) {
	info->rocket_status |= selector;
}

void Rocket_ResetStatus(Rocket_Info_t* info, Rocket_Status_t selector) {
	info->rocket_status &= ~selector;
}

void Rocket_EnvData_ShiftDataSet(Rocket_Info_t* info) {
	uint8_t i;
	for(i = 0; i <= 2; i++) {
		info->dataset[3-i] = info->dataset[3-(i+1)];
	}
}

void Rocket_EnvData_AddNewDataSet(Rocket_Info_t* info, EnvData_DataSet_t data) {
	info->dataset[0] = data;
}

EnvData_DataSet_t Rocket_EnvData_GetEnvDataSet(Rocket_Info_t* info, uint8_t oldness) {
	return info->dataset[oldness];
}

void Rocket_EnvData_SetGroundEnvDataSet(Rocket_Info_t* info, EnvData_DataSet_t data) {
	info->ground = data;
}
EnvData_DataSet_t Rocket_EnvData_GetGroundEnvDataSet(Rocket_Info_t* info) {
	return info->ground;
}


