/*
 * secboard.c
 *
 *  Created on: May 24, 2024
 *      Author: knn
 */
#include "main.h"
#include "secboard.h"
#include "tim.h"

void eq_band(param_sgtl_t* param_son, int* freq, float* gain){
	HAL_GPIO_WritePin(band0_GPIO_Port,band0_Pin,1);
	HAL_GPIO_WritePin(band1_GPIO_Port,band1_Pin,1);
	HAL_GPIO_WritePin(band2_GPIO_Port,band2_Pin,1);
	HAL_GPIO_WritePin(band3_GPIO_Port,band3_Pin,1);
	switch(param_son->bandmod){
	case 0 :
		HAL_GPIO_WritePin(band0_GPIO_Port,band0_Pin,0);
		*freq = param_son->band0_freq ;
		*gain = param_son->band0_gain;
		break;
	case 1 :
		HAL_GPIO_WritePin(band1_GPIO_Port,band1_Pin,0);
		*freq = param_son->band1_freq ;
		*gain = param_son->band1_gain;
		break;

	case 2:
		HAL_GPIO_WritePin(band2_GPIO_Port,band2_Pin,0);
		*freq = param_son->band2_freq ;
		*gain = param_son->band2_gain;
		break;

	case 3:
		HAL_GPIO_WritePin(band3_GPIO_Port,band3_Pin,0);
		*freq = param_son->band3_freq ;
		*gain = param_son->band3_gain;
		break;

	default :
		Error_Handler();
		break;
	}
}

void colormotion(TIM_HandleTypeDef* htim3,int freq,float gain){
	__HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_1, 1-(1/20000*(freq - 19000)));
	__HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_2, (1/20000)*(freq - 19000));
	__HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_3, 1 - (1/24*(gain + 12)));
	__HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_4, 1/24*(gain + 12));
}


