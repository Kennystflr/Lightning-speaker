
/*
 * mywork.c
 *
 *  Created on: Mar 14, 2024
 *      Author: knn
 */
#ifndef SAMPLE_RATE
#define SAMPLE_RATE 48000
#endif
#define GAIN_INCREMENT 0.5
#define ABS_GAIN_MAX 12
#define FREQ_MAX 19000
#define FREQ_MIN 40


#include <stdio.h>
#include <math.h>
#include "main.h"
#include <filter.h>


void calc_eq_coefs(int freq, float gain_dB, int * coef){

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
	coef[2] = (int) (b_2/a_0 + 0.499);
	coef[3] = (int) (a_1/a_0 + 0.499);
	coef[4] = (int) (a_2/a_0 + 0.499);

}


void volume(param_sgtl_t * param_son, h_sgtl5000_t  * sgtl, uint16_t in12bits){
	int nombrevaleurs=(int)(0.6*4096/128);
	for(int multiple=0; multiple<128; multiple++){
		if(in12bits>nombrevaleurs*multiple-1 && in12bits < nombrevaleurs * multiple + nombrevaleurs){
			param_son->volume=multiple;
		}
	}
	sgtl5000_i2c_write_register(sgtl,SGTL5000_DAP_MAIN_CHAN,(uint32_t) param_son->volume);
}

void set_filter(h_sgtl5000_t * sgtl,int freq, float gain_dB){
	int coefs[5];
	calc_eq_coefs(freq,gain_dB,coefs);
	//set b0
	sgtl5000_i2c_write_register(sgtl,SGTL5000_DAP_COEF_WR_B0_LSB, coefs[0] & 0xF);//Recupération des 4 derniers bits sur 20 du coefficient
	sgtl5000_i2c_write_register(sgtl,SGTL5000_DAP_COEF_WR_B0_MSB, ((coefs[0] &0xFFFF0)>>2));//Recupération des 16 premiers bits sur 20 du coefficient
	//set b1
	sgtl5000_i2c_write_register(sgtl,SGTL5000_DAP_COEF_WR_B1_LSB, coefs[1] & 0xF);//Recupération des 4 derniers bits sur 20 du coefficient
	sgtl5000_i2c_write_register(sgtl,SGTL5000_DAP_COEF_WR_B1_MSB, ((coefs[1] &0xFFFF0)>>2));//Recupération des 16 premiers bits sur 20 du coefficient
	//set b2
	sgtl5000_i2c_write_register(sgtl,SGTL5000_DAP_COEF_WR_B2_LSB, coefs[2] & 0xF);//Recupération des 4 derniers bits sur 20 du coefficient
	sgtl5000_i2c_write_register(sgtl,SGTL5000_DAP_COEF_WR_B2_MSB, ((coefs[2] &0xFFFF0)>>2));//Recupération des 16 premiers bits sur 20 du coefficient
	//set a1
	sgtl5000_i2c_write_register(sgtl,SGTL5000_DAP_COEF_WR_A1_LSB, coefs[3] & 0xF);//Recupération des 4 derniers bits sur 20 du coefficient
	sgtl5000_i2c_write_register(sgtl,SGTL5000_DAP_COEF_WR_A1_MSB, ((coefs[3] &0xFFFF0)>>2));//Recupération des 16 premiers bits sur 20 du coefficient
	//set A2
	sgtl5000_i2c_write_register(sgtl,SGTL5000_DAP_COEF_WR_A2_LSB, coefs[4] & 0xF);//Recupération des 4 derniers bits sur 20 du coefficient
	sgtl5000_i2c_write_register(sgtl,SGTL5000_DAP_COEF_WR_A2_MSB, ((coefs[4] &0xFFFF0)>>2));//Recupération des 16 premiers bits sur 20 du coefficient
}


void init_valeur_default(param_sgtl_t* param_son){
	param_son->band0_freq=450;
	param_son->band1_freq=1900;
	param_son->band2_freq=5500;
	param_son->band3_freq=12000;
	param_son->band0_gain=0;
	param_son->band1_gain=0;
	param_son->band2_gain=0;
	param_son->band3_gain=0;
	param_son->volume=0;
	param_son->bandmod=0;
}

void modif_freq(param_sgtl_t * param_son, int signalA, int signalB){

	int difffrequence=0;

	switch(param_son->bandmod){

	case 0 :
		difffrequence=700/50;
		if((param_son->band0_freq>=FREQ_MIN) & (param_son->band0_freq<=FREQ_MAX)){
			if(signalA==1){ //on suppose que si B est en retard, on tourne ds le sens horaire
				if(signalB==0){
					param_son->band0_freq += difffrequence;
				}
				else {
					param_son->band0_freq -= difffrequence;
				}
			}
			break;
		}
	case 1 :
		if((param_son->band1_freq>=FREQ_MIN) & (param_son->band1_freq<=FREQ_MAX)){
			difffrequence=2200/50;
			if(signalA==1){ //on suppose que si B est en retard, on tourne ds le sens horaire
				if(signalB==0){
					param_son->band1_freq += difffrequence;
				}
				else {
					param_son->band1_freq -= difffrequence;
				}
			}
			break;
		}
	case 2 :
		if((param_son->band2_freq>=FREQ_MIN) & (param_son->band2_freq<=FREQ_MAX)){
			difffrequence=5000/50;
			if(signalA==1){ //on suppose que si B est en retard, on tourne ds le sens horaire
				if(signalB==0){
					param_son->band2_freq += difffrequence;
				}
				else {
					param_son->band2_freq -= difffrequence;
				}
			}
			break;
		}
	default :
		if((param_son->band3_freq>=FREQ_MIN) & (param_son->band3_freq<=FREQ_MAX)){
			difffrequence=8000/50;
			if(signalA==1){ //on suppose que si B est en retard, on tourne ds le sens horaire
				if(signalB==0){
					param_son->band3_freq += difffrequence;
				}
				else {
					param_son->band3_freq -= difffrequence;
				}
			}
			break;
		}
	}
}

void modif_gain(param_sgtl_t * param_son, int signalA, int signalB){

	switch(param_son->bandmod){

	case 0 :
		if((param_son->band0_gain>=-ABS_GAIN_MAX) & (param_son->band0_gain<=ABS_GAIN_MAX)){
			if(signalA==1){ //on suppose que si B est en retard, on tourne ds le sens horaire
				if(signalB==0){
					param_son->band0_gain += GAIN_INCREMENT;
				}
				else {
					param_son->band0_gain -= GAIN_INCREMENT;
				}
			}
			break;
		}
	case 1 :
		if((param_son->band1_gain>=-ABS_GAIN_MAX) & (param_son->band1_gain<=ABS_GAIN_MAX)){
			if(signalA==1){ //on suppose que si B est en retard, on tourne ds le sens horaire
				if(signalB==0){
					param_son->band1_gain += GAIN_INCREMENT;
				}
				else {
					param_son->band1_gain -= GAIN_INCREMENT;
				}
			}
			break;
		}
	case 2 :
		if((param_son->band2_gain>=-ABS_GAIN_MAX) & (param_son->band2_gain<=ABS_GAIN_MAX)){
			if(signalA==1){ //on suppose que si B est en retard, on tourne ds le sens horaire
				if(signalB==0){
					param_son->band2_gain += GAIN_INCREMENT;
				}
				else {
					param_son->band2_gain -= GAIN_INCREMENT;
				}
			}
			break;
		}
	default :
		if((param_son->band3_gain>=-ABS_GAIN_MAX) & (param_son->band3_gain<=ABS_GAIN_MAX)){
			if(signalA==1){ //on suppose que si B est en retard, on tourne ds le sens horaire
				if(signalB==0){
					param_son->band3_gain += GAIN_INCREMENT;
				}
				else {
					param_son->band3_gain -= GAIN_INCREMENT;
				}
			}
			break;
		}
	}
}

void ChangementEtat(param_sgtl_t * param_son,h_sgtl5000_t * sgtl){
	switch(param_son->bandmod)
	{
	case 0:
		sgtl5000_i2c_write_register(sgtl,SGTL5000_DAP_FILTER_COEF_ACCESS,(uint32_t) 0x1 );
		break;
	case 1:
		sgtl5000_i2c_write_register(sgtl,SGTL5000_DAP_FILTER_COEF_ACCESS,(uint32_t) 0x2 );
		break;
	case 2 :
		sgtl5000_i2c_write_register(sgtl,SGTL5000_DAP_FILTER_COEF_ACCESS,(uint32_t) 0x3 );
		break;
	default:
		sgtl5000_i2c_write_register(sgtl,SGTL5000_DAP_FILTER_COEF_ACCESS,(uint32_t) 0x0 );
		break;
	}

}


