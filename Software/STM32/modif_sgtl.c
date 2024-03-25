#include "project.h"    

void init_valeur_default(){
    param_son.band0_freq=450;
    param_son.band1_freq=1900;
    param_son.band2_freq=5500;
    param_son.band3_freq=12000;
    param_son.band0_gain=0;
    param_son.band1_gain=0;
    param_son.band2_gain=0;
    param_son.band3_gain=0;
    param_son.volume=0;
}

void modif_freq(param_sgtl_t * param_son, int signalA, int signalB){

    int difffrequence=0;
    
    switch(bandmod){

        case 0 :
            difffrequence=700/50;
            if(signalA==1){ //on suppose que si B est en retard, on tourne ds le sens horaire
                if(signalB==0){
                    *param_son.band0_freq=*param_son.band0_freq+difffrequence;
                }
                else {
                    *param_son.band0_freq=*param_son.band0_freq-difffrequence;
                }
            }
            break;

        case 1 :
            difffrequence=2200/50;
            if(signalA==1){ //on suppose que si B est en retard, on tourne ds le sens horaire
                if(signalB==0){
                    *param_son.band1_freq=*param_son.band1_freq+difffrequence;
                }
                else {
                    *param_son.band1_freq=*param_son.band1_freq-difffrequence;
                }
            }
            break;

        case 2 :
            difffrequence=5000/50;
            if(signalA==1){ //on suppose que si B est en retard, on tourne ds le sens horaire
                if(signalB==0){
                    *param_son.band2_freq=*param_son.band2_freq+difffrequence;
                }
                else {
                    *param_son.band2_freq=*param_son.band2_freq-difffrequence;
                }
            }
            break;

        case 3 :
            difffrequence=8000/50;
            if(signalA==1){ //on suppose que si B est en retard, on tourne ds le sens horaire
                if(signalB==0){
                    *param_son.band3_freq=*param_son.band3_freq+difffrequence;
                }
                else {
                    *param_son.band3_freq=*param_son.band3_freq-difffrequence;
                }
            }
            break;
    }
}

void modif_gain(param_sgtl_t * param_son, int signalA, singal B){
    
    switch(bandmod){

        case 0 :
            if(signalA==1){ //on suppose que si B est en retard, on tourne ds le sens horaire
                if(signalB==0){
                    *param_son.band0_gain=*param_son.band0_gain+1;
                }
                else {
                    *param_son.band0_gain=*param_son.band0_gain-1;
                }
            }
            break;

        case 1 :
            if(signalA==1){ //on suppose que si B est en retard, on tourne ds le sens horaire
                if(signalB==0){
                    *param_son.band1_gain=*param_son.band1_gain+1;
                }
                else {
                    *param_son.band1_gain=*param_son.band1_gain-1;
                }
            }
            break;

        case 2 :
            if(signalA==1){ //on suppose que si B est en retard, on tourne ds le sens horaire
                if(signalB==0){
                    *param_son.band2_gain=*param_son.band2_gain+1;
                }
                else {
                    *param_son.band2_gain=*param_son.band2_gain-1;
                }
            }
            break;

        case 3 :
            if(signalA==1){ //on suppose que si B est en retard, on tourne ds le sens horaire
                if(signalB==0){
                    *param_son.band3_gain=*param_son.band3_gain+1;
                }
                else {
                    *param_son.band3_gain=*param_son.band3_gain-1;
                }
            }
            break;
    }
}

void modif_volume(param_sgtl_t * param_son, int in12bits){
	int nombrevaleurs=(int)(0.6*4096/128);
	for(int multiple=0; multiple<128; multiple++){
		if(in12bits>nombrevaleurs*multiple-1 && in12bits < nombrevaleurs * multiple + nombrevaleurs){
			*param_son.volume=multiple;
		}
	}
}
