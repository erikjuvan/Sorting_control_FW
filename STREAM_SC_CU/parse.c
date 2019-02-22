#include "parse.h"
#include "communication.h"
#include "main.h"
#include "uart.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

extern Mode                   g_mode;
extern CommunicationInterface g_communication_interface;

extern int g_training;
extern int g_timer_period;
extern int g_verbose_level;

extern float A1;
extern float A2;
extern float A4;
extern float FTR_THRSHLD;

extern uint32_t T_delay;
extern uint32_t T_duration;
extern uint32_t T_blind;

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
static void Function_VERG(char* str, write_func Write)
{
    char buf[100] = {0};
    snprintf(buf, sizeof(buf), "VERG,%s,%s,%s\n", SWVER, HWVER, COMPATIBILITYMODE);
    Write((uint8_t*)buf, strlen(buf));
}

static void Function_VRBS(char* str, write_func Write)
{
    str             = strtok(NULL, Delims);
    g_verbose_level = atoi((char*)str);
}

static void Function_VRBG(char* str, write_func Write)
{
    char buf[10] = {0};
    snprintf(buf, sizeof(buf), "VRBG,%u\n", g_verbose_level);

    Write((uint8_t*)buf, strlen(buf));
}

static void Function_IDST(char* str, write_func Write)
{
    str = strtok(NULL, Delims);
    if (str != NULL) {
        int num = atoi(str);
        if (num <= 127) {
            UART_Set_Address(num);
        }
    }
}

static void Function_IDGT(char* str, write_func Write)
{
    char buf[10] = {0};
    snprintf(buf, sizeof(buf), "ID:%u\n", UART_Address);

    Write((uint8_t*)buf, strlen(buf));
}

// "CSETF,1000" - 1000 is in hertz
static void Function_SETFREQ(char* str, write_func Write)
{
    str            = strtok(NULL, Delims);
    int val        = atoi((char*)str);
    g_timer_period = 1e6 / val; // convert val which are hertz to period which is in us
    ChangeSampleFrequency();
}

static void Function_RESET(char* str, write_func Write)
{
    NVIC_SystemReset();
}

static void Function_TRAIN(char* str, write_func Write)
{
    // Not currently supported
}

static void Function_SETPARAMS(char* str, write_func Write)
{
    str         = strtok(NULL, ",");
    A1          = atof(str);
    str         = strtok(NULL, ",");
    A2          = atof(str);
    str         = strtok(NULL, ",");
    A4          = atof(str);
    str         = strtok(NULL, Delims);
    FTR_THRSHLD = atof(str);
}

static void Function_SETTIMES(char* str, write_func Write)
{
    str        = strtok(NULL, ",");
    T_delay    = atoi(str);
    str        = strtok(NULL, ",");
    T_duration = atoi(str);
    str        = strtok(NULL, Delims);
    T_blind    = atoi(str);
}

static void Function_PING(char* str, write_func Write)
{
    Write((uint8_t*)"OK\n", 2);
}

static void Function_GETFREQ(char* str, write_func Write)
{
    char buf[10] = {0};
    int  hz      = 1e6 / g_timer_period;
    snprintf(buf, sizeof(buf), "%u\n", hz);

    Write((uint8_t*)buf, strlen(buf));
}

static void Function_GETPARAMS(char* str, write_func Write)
{
    char buf[50] = {0};
    snprintf(buf, sizeof(buf), "%.3f,%.3f,%.3f,%.1f\n", A1, A2, A4, FTR_THRSHLD);

    Write((uint8_t*)buf, strlen(buf));
}

static void Function_GETTIMES(char* str, write_func Write)
{
    char buf[50] = {0};
    snprintf(buf, sizeof(buf), "%u,%u,%u\n", T_delay, T_duration, T_blind);

    Write((uint8_t*)buf, strlen(buf));
}

static void Function_GETSETTINGS(char* str, write_func Write)
{
    char buf[300]; // don't need to zero it out
    int  freq = 1e6 / g_timer_period;
    snprintf(buf, sizeof(buf), "FREQ:%u\nTIMES:%u,%u,%u\nFILTER_PARAMS:%.3f,%.3f,%.3f,%.1f\n", freq, T_delay, T_duration, T_blind, A1, A2, A4, FTR_THRSHLD);

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
    COMMAND(VERG),
    COMMAND(VRBG),
    COMMAND(VRBS),
    COMMAND(IDST),
    COMMAND(IDGT),
    COMMAND(PING),
    COMMAND(RESET),
    COMMAND(TRAIN),

    COMMAND(GETFREQ),
    COMMAND(GETPARAMS),
    COMMAND(GETTIMES),
    COMMAND(GETSETTINGS),

    COMMAND(SETFREQ),
    COMMAND(SETPARAMS),
    COMMAND(SETTIMES)};

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
