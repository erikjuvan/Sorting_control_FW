#pragma once

#define SWVER "RK_SC_FWMAIN V1.0.0"
#define HWVER "RK_SC_SCCUPCB V1.0.0"
#define COMPATIBILITYMODE "CM00"

#define N_CHANNELS 8

typedef enum { SORT,
               CONFIG } Mode;

typedef struct {
    const uint32_t delim;
    uint32_t       packet_id;
} Header;

void     SetSampleFrequency(int freq_hz);
uint32_t GetSampleFrequency();
void     SetSequence(int* seq, int num_of_elements);
char*    GetSequence(char* buf, int sizeof_buf);
void     SetSyncPinAsOutput();
void     SetSyncPinAsInput();