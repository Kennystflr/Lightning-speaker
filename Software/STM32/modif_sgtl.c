#include "project.h"    
#include <stm32l4xx_hal.h> // a changer en fct de la stm
extern UART_HandleTypeDef huart2;

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


//Gestion bouton


// Déclaration de la fonction de configuration des GPIO
void GPIO_Init(void);

// Fonction principale
int ChangementEtat(int etat)
{
    // Initialisation de la librairie HAL (Hardware Abstraction Layer)
    HAL_Init();

    // Configuration des GPIO
    GPIO_Init();

    while (1)
    {
        if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
            // Le bouton est enfoncé
            switch(etat)
            {
                case 1:
                	while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET){}
					return 2;
                case 2:
                	while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET){}
                	return 3;
                case 3:
                	while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET){}
                	return 4;
                case 4:
                	while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET){}
                	return 1;
            }
        } else {
            // Le bouton n'est pas enfoncé
        }
    }
}


// Fonction de configuration des GPIO
void GPIO_Init(void)
{
	// Structure de configuration des GPIO
	    GPIO_InitTypeDef GPIO_InitStruct;

	// Activer le périphérique GPIOC
	    __HAL_RCC_GPIOC_CLK_ENABLE();

	// Configurer le GPIO comme entrée avec pull-up
	    GPIO_InitStruct.Pin = GPIO_PIN_13;
	    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	    GPIO_InitStruct.Pull = GPIO_PULLUP;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

