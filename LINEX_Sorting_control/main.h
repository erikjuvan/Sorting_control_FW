#pragma once

#define	N_CHANNELS		8

#define SWVER "RK_SC_FWMAIN V1.0.0" 
#define HWVER "RK_SC_SCCUPCB V1.0.0" 
#define COMPATIBILITYMODE "CM00"

struct SystemParameters{
	int timer_period;	// us
	int verbose_level;
};

typedef enum {RAW, TRAINED, FILTERED} DisplayData;