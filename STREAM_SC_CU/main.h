#pragma once

#define SWVER "RK_SC_FWMAIN V1.0.0"
#define HWVER "RK_SC_SCCUPCB V1.0.0"
#define COMPATIBILITYMODE "CM00"

#define N_CHANNELS 8

#define TIM_COUNT_FREQ 1000000 // f=1MHz, T=1us

typedef enum { SORT,
               CONFIG } Mode;

typedef struct {
    const uint32_t delim;
    uint32_t       packet_id;
} Header;