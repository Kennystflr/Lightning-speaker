/*
 * filter.h
 *
 *  Created on: Mar 14, 2024
 *      Author: knn
 */

#ifndef INC_FILTER_H_
#define INC_FILTER_H_

#include "sgtl5000.h"

extern ADC_HandleTypeDef hadc1;

typedef struct param_sgtl_struct
{
    int volume;
    int bandmod;
    int band0_freq;
    float band0_gain;
    int band1_freq;
    float band1_gain;
    int band2_freq;
    float band2_gain;
    int band3_freq;
    float band3_gain;
} param_sgtl_t;


void init_valeur_default(param_sgtl_t* param_son);
void modif_freq(param_sgtl_t *,int signalA, int signalB);
void modif_gain(param_sgtl_t *,int signalA, int singalB);
void ChangementEtat(param_sgtl_t*,h_sgtl5000_t *);
void calc_eq_coefs(int freq,float gain_dB,int * coefs);
void volume(param_sgtl_t * var,h_sgtl5000_t * sgtl,uint16_t  in12bits);
void set_filter(h_sgtl5000_t * sgtl,int freq, float gain_dB);


#endif /* INC_FILTER_H_ */
