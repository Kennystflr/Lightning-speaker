#include "project.h"

extern param_sgtl_t param_son;
param_son.band0_freq=450;
param_son.band1_freq=1900;
param_son.band2_freq=5500;
param_son.band3_freq=12000;
param_son.band0_gain=0;
param_son.band1_gain=0;
param_son.band2_gain=0;
param_son.band3_gain=0;

int main()
{
    
    modificationvaleur(1,850,1,0);

    return 0;
}
