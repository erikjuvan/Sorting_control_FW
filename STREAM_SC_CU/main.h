#pragma once

#define SWVER "SC_CU_FW V1.3.0.1"
#define HWVER "N/A"
#define COMPATIBILITYMODE "N/A"

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
void     ResetHeaderID();
void     Train();
void     Untrain();
void     SetValve(unsigned int valve_idx, unsigned int state);
int      ChooseFilterType(int filter_number);