
/*
 * mywork.c
 *
 *  Created on: Mar 14, 2024
 *      Author: knn
 */
#ifndef SAMPLE_RATE
#define SAMPLE_RATE 48000
#endif



#include <stdio.h>
#include <math.h>
#include "main.h"
#include <filter.h>


extern ADC_HandleTypeDef hadc1;

sgtl5000_registers_t MSB_coef[] =
{
		SGTL5000_DAP_COEF_WR_B0_MSB,
		SGTL5000_DAP_COEF_WR_B1_MSB,
		SGTL5000_DAP_COEF_WR_B2_MSB,
		SGTL5000_DAP_COEF_WR_A1_MSB,
		SGTL5000_DAP_COEF_WR_A2_MSB
};

sgtl5000_registers_t LSB_coef[] = {
		SGTL5000_DAP_COEF_WR_B0_LSB,
		SGTL5000_DAP_COEF_WR_B1_LSB,
		SGTL5000_DAP_COEF_WR_B2_LSB,
		SGTL5000_DAP_COEF_WR_A1_LSB,
		SGTL5000_DAP_COEF_WR_A2_LSB};


void calc_eq_coefs(int freq, float gain_dB, int coef[4]){

	float Q = 1.5 ;
	float omega = (float) 2*M_PI*freq/SAMPLE_RATE;
	float alpha =(float) sin(omega)/(2*Q);
	float A = pow(10,gain_dB/40);

	float b_0 = 1 + alpha * A;
	float b_1 = -2 * cos(omega);
	float b_2 = 1 - alpha * A;
	float a_0 = 1 + (alpha / A);
	float a_1 = -2 * cos(omega);
	float a_2 = 1 - (alpha / A);


	coef[0] = (int) (b_0/a_0 + 0.499);
	coef[1] = (int) (b_1/a_0 + 0.499);
	coef[2] = (int) (b_2/a_0  + 0.499);
	coef[3] = (int) (a_1/a_0  + 0.499);
	coef[4] = (int) (a_2/a_0  + 0.499);

}


void volume(uint16_t* var,h_sgtl5000_t * sgtl){
	uint16_t value_pot;
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&value_pot,1);
	HAL_Delay(1);
	HAL_ADC_Stop_DMA(&hadc1);
	*var =(uint32_t) ((value_pot - 200)*0.6);
	sgtl5000_i2c_write_register(sgtl,SGTL5000_DAP_MAIN_CHAN, *var);
}

void set_filter(h_sgtl5000_t * sgtl,int freq, float gain_dB){
	int coefs[4];
	calc_eq_coefs(freq,gain_dB,coefs);
	for (int i=0;i<5;i++){
		set_coef(sgtl,coefs[i],i);
	}
}

void set_coef(h_sgtl5000_t * sgtl, int coef, int i){
	sgtl5000_i2c_write_register(sgtl,LSB_coef[i], coef & 0xF);//Recupération des 4 derniers bits sur 20 du coefficient
	sgtl5000_i2c_write_register(sgtl,MSB_coef[i], ((coef &0xFFFF0)>>2));//Recupération des 16 premiers bits sur 20 du coefficient
}
