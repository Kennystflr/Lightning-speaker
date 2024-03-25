#ifndef _project_h
#define _project_h

typedef struct param_sgtl_struct 
{   
    int volume;
    int band_mod;
    int band0_freq; 
    int band0_gain;
    int band1_freq; 
    int band1_gain;
    int band2_freq; 
    int band2_gain;
    int band3_freq; 
    int band3_gain;
} param_sgtl_t;

void ini_valeur_default(void);
int modif_volume(param_sgtl_t *,int);
void modif_valeur(param_sgtl_t *,int,int);
void modif_gain(param_sgtl_t *,int,int);
