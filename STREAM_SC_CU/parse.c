#include "parse.h"
#include "communication.h"
#include "main.h"
#include "uart.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

extern Mode                   g_mode;
extern CommunicationInterface g_communication_interface;

extern Header header;

extern int g_training;
extern int g_sample_every_N_counts;
extern int g_verbose_level;

extern uint32_t g_delay_ticks_param;
extern uint32_t g_duration_ticks_param;
extern uint32_t g_blind_ticks_param;

extern float g_lpf1_K;
extern float g_hpf_K;
extern float g_lpf2_K;

extern float g_threshold;

extern void ChangeSampleFrequency();

extern uint8_t UART_Address;

const static char Delims[] = "\n\r\t, ";

static void Function_SORT(char* str, write_func Write)
{
    g_mode = SORT;
}

static void Function_CONFIG(char* str, write_func Write)
{
    g_mode = CONFIG;
}

static void Function_USB(char* str, write_func Write)
{
    g_communication_interface = USB;
}

static void Function_UART(char* str, write_func Write)
{
    g_communication_interface = UART;
}

static void Function_UART_SORT(char* str, write_func Write)
{
    g_communication_interface = UART;
    g_mode                    = SORT;
}
static void Function_VERSION(char* str, write_func Write)
{
    char buf[100] = {0};
    snprintf(buf, sizeof(buf), "VERSION,%s,%s,%s", SWVER, HWVER, COMPATIBILITYMODE);
    Write((uint8_t*)buf, strlen(buf));
}

static void Function_VRBS(char* str, write_func Write)
{
    str             = strtok(NULL, Delims);
    g_verbose_level = atoi((char*)str);

    // Reset packet ID when entering verbose mode
    if (g_verbose_level != 0)
        header.packet_id = 0;
}

static void Function_VRBG(char* str, write_func Write)
{
    char buf[10] = {0};
    snprintf(buf, sizeof(buf), "VRBG,%u", g_verbose_level);

    Write((uint8_t*)buf, strlen(buf));
}

static void Function_SETID(char* str, write_func Write)
{
    str = strtok(NULL, Delims);
    if (str != NULL) {
        int num = atoi(str);
        if (num <= 127) {
            UART_Set_Address(num);
        }
    }

    // Echo
    char buf[10] = {0};
    snprintf(buf, sizeof(buf), "SETID,%u", UART_Address);
    Write((uint8_t*)buf, strlen(buf));
}

static void Function_GETID(char* str, write_func Write)
{
    char buf[10] = {0};
    snprintf(buf, sizeof(buf), "ID,%u", UART_Address);
    Write((uint8_t*)buf, strlen(buf));
}

static void Function_RESET(char* str, write_func Write)
{
    NVIC_SystemReset();
}

static void Function_TRAIN(char* str, write_func Write)
{
    // Not currently supported
}

// "SETFREQ,1000" - 1000 Hz
static void Function_SETFREQ(char* str, write_func Write)
{
    str                          = strtok(NULL, Delims);
    int requested_sample_freq_hz = atoi((char*)str);
    if (requested_sample_freq_hz > 0) {
        g_sample_every_N_counts = TIM_COUNT_FREQ / requested_sample_freq_hz; // convert val which are hertz to period which is in us
        ChangeSampleFrequency();
    }

    // Echo
    char buf[30];
    snprintf(buf, sizeof(buf), "SETFREQ,%u", requested_sample_freq_hz);
    Write((uint8_t*)buf, strlen(buf));
}

static void Function_SETSORTTICKS(char* str, write_func Write)
{
    str                    = strtok(NULL, Delims);
    g_delay_ticks_param    = atoi(str);
    str                    = strtok(NULL, Delims);
    g_duration_ticks_param = atoi(str);
    str                    = strtok(NULL, Delims);
    g_blind_ticks_param    = atoi(str);

    // Echo
    char buf[50];
    snprintf(buf, sizeof(buf), "SETSORTTICKS,%u,%u,%u", g_delay_ticks_param, g_duration_ticks_param, g_blind_ticks_param);
    Write((uint8_t*)buf, strlen(buf));
}

static void Function_SETFILTERCOEFF(char* str, write_func Write)
{
    str      = strtok(NULL, Delims);
    g_lpf1_K = atof(str);
    str      = strtok(NULL, Delims);
    g_hpf_K  = atof(str);
    str      = strtok(NULL, Delims);
    g_lpf2_K = atof(str);

    // Echo
    char buf[80];
    snprintf(buf, sizeof(buf), "SETFILTERCOEFF,%.3f,%.3f,%.3f", g_lpf1_K, g_hpf_K, g_lpf2_K);
    Write((uint8_t*)buf, strlen(buf));
}

static void Function_SETTHRESHOLD(char* str, write_func Write)
{
    str         = strtok(NULL, Delims);
    g_threshold = atof(str);

    // Echo
    char buf[30];
    snprintf(buf, sizeof(buf), "SETTHRESHOLD,%.1f", g_threshold);
    Write((uint8_t*)buf, strlen(buf));
}

static void Function_PING(char* str, write_func Write)
{
    Write((uint8_t*)"PING", 4);
}

static void Function_GETFREQ(char* str, write_func Write)
{
    char buf[10]        = {0};
    int  sample_freq_hz = 0;
    if (g_sample_every_N_counts > 0)
        sample_freq_hz = TIM_COUNT_FREQ / g_sample_every_N_counts;

    snprintf(buf, sizeof(buf), "%u", sample_freq_hz);

    Write((uint8_t*)buf, strlen(buf));
}

static void Function_GETSORTTICKS(char* str, write_func Write)
{
    char buf[50] = {0};
    snprintf(buf, sizeof(buf), "%u,%u,%u", g_delay_ticks_param, g_duration_ticks_param, g_blind_ticks_param);

    Write((uint8_t*)buf, strlen(buf));
}

static void Function_GETFILTERCOEFF(char* str, write_func Write)
{
    char buf[50] = {0};
    snprintf(buf, sizeof(buf), "%.3f,%.3f,%.3f", g_lpf1_K, g_hpf_K, g_lpf2_K);

    Write((uint8_t*)buf, strlen(buf));
}

static void Function_GETTHRESHOLD(char* str, write_func Write)
{
    char buf[10] = {0};
    snprintf(buf, sizeof(buf), "%.1f", g_threshold);

    Write((uint8_t*)buf, strlen(buf));
}

static void Function_GETSETTINGS(char* str, write_func Write)
{
    char buf[300]; // don't need to zero it out
    int  sample_freq_hz = 0;
    if (g_sample_every_N_counts > 0)
        sample_freq_hz = TIM_COUNT_FREQ / g_sample_every_N_counts;
    snprintf(buf, sizeof(buf), "FREQ,%u\nSORTTICKS,%u,%u,%u\nFILTERCOEFF,%.3f,%.3f,%.3f\nTHRESHOLD,%.1f\n",
             sample_freq_hz, g_delay_ticks_param, g_duration_ticks_param, g_blind_ticks_param, g_lpf1_K, g_hpf_K, g_lpf2_K, g_threshold);

    Write((uint8_t*)buf, strlen(buf));
}

#define COMMAND(NAME)          \
    {                          \
#NAME, Function_##NAME \
    }

static struct {
    const char* name;
    void (*Func)(char*, write_func);
} command[] = {
    COMMAND(SORT),
    COMMAND(CONFIG),
    COMMAND(USB),
    COMMAND(UART),
    COMMAND(UART_SORT),
    COMMAND(VERSION),
    COMMAND(VRBG),
    COMMAND(VRBS),
    COMMAND(SETID),
    COMMAND(GETID),
    COMMAND(PING),
    COMMAND(RESET),
    COMMAND(TRAIN),

    COMMAND(GETFREQ),
    COMMAND(GETSORTTICKS),
    COMMAND(GETFILTERCOEFF),
    COMMAND(GETTHRESHOLD),
    COMMAND(GETSETTINGS),

    COMMAND(SETFREQ),
    COMMAND(SETSORTTICKS),
    COMMAND(SETFILTERCOEFF),
    COMMAND(SETTHRESHOLD)};

void Parse(char* string, write_func Write)
{
    char* str;

    str = strtok(string, Delims);
    while (str != NULL) {

        for (int i = 0; i < sizeof(command) / sizeof(command[0]); ++i) {
            if (strcmp(str, command[i].name) == 0) {
                command[i].Func(str, Write);
                break;
            }
        }

        str = strtok(NULL, Delims);
    }
}
