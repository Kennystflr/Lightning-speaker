#include "project.h"    

void modificationvaleur(param_sgtl_t * param_sgtl, int signalA, int signalB)
{
    int difffrequence=0;
    
    switch(bandmod){
        case 0 :
            difffrequence=700/50;
            if(signalA==1){ //on suppose que si B est en retard, on tourne ds le sens horaire
        if(signalB==0){
            frequenceactuel=frequenceactuel+difffrequence;
            printf('%d \r\n',frequenceactuel);
        }
        else{
            frequenceactuel=frequenceactuel-difffrequence;
            printf('%d \r\n',frequenceactuel);
        }
            break;
        case 1 :
            difffrequence=2200/50;
            break;
        case 2 :
            difffrequence=5000/50;
            break;
        case 3 :
            difffrequence=8000/50;
            break;
    }

    
    }
}

int partitionpotar(int in12bits, int n){
	int nombrevaleurs=4096/n;
	for(int multiple=0;multiple<n;multiple++){
		if(in12bits>nombrevaleurs*multiple-1 && in12bits<nombrevaleurs*multiple+nombrevaleurs){
			return multiple;
		}
	}
}
