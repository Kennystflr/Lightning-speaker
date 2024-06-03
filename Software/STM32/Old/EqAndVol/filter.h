/*
 * mywork.h
 *
 *  Created on: Mar 14, 2024
 *      Author: knn
 */

#ifndef INC_FILTER_H_
#define INC_FILTER_H_

#include "sgtl5000.h"



void calc_eq_coefs(int freq,float gain_dB,int coefs[4]);
void volume(uint16_t* var,h_sgtl5000_t * sgtl);
void set_filter(h_sgtl5000_t * sgtl,int freq, float gain_dB);
void set_coef(h_sgtl5000_t * sgtl,int coef,int i);




#endif /* INC_FILTER_H_ */
