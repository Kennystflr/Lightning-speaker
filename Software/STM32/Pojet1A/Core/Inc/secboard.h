/*
 * secboard.h
 *
 *  Created on: May 24, 2024
 *      Author: knn
 */

#ifndef INC_SECBOARD_H_
#define INC_SECBOARD_H_
#include "filter.h"

void eq_band(param_sgtl_t * param_son,int* freq,float* gain);
void colormotion(TIM_HandleTypeDef* htim3,int freq,float gain);

#endif /* INC_SECBOARD_H_ */
