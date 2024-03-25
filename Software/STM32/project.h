#include <stdio.h>

typedef struct param_sgtl_struct 
{   
    int volume;
    int band_mod;
    int encoder;
    int band0_freq; 
    int band0_gain;
    int band1_freq; 
    int band1_gain;
    int band2_freq; 
    int band2_gain;
    int band3_freq; 
    int band3_gain;
} param_sgtl_t;

void modificationvaleur(param_sgtl_t * param_sgtl, int signalA, int signalB);
