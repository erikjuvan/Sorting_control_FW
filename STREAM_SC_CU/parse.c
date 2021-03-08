/// @file parse.c
/// <summary>
/// Parsing library and commands implementation.
/// </summary>
///
/// Supervision: /
///
/// Company: Sensum d.o.o.
///
/// @authors Erik Juvan
///
/// @version /
/////-----------------------------------------------------------
// Company: Sensum d.o.o.

// C Standard Library
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// User Library
#include "communication.h"
#include "main.h"
#include "parse.h"
#include "uart.h"

extern Mode g_mode;

extern int g_verbose_level;

extern uint32_t g_delay_ticks_param;
extern uint32_t g_duration_ticks_param;
extern uint32_t g_blind_ticks_param;

extern float g_lpf1_K;
extern float g_hpf_K;
extern float g_lpf2_K;

extern float g_threshold;

extern uint8_t UART_Address;

extern unsigned int sequence_N;

const static char Delims[] = "\n\r\t, ";

static int requested_sampling_freq_hz;

//---------------------------------------------------------------------
/// <summary> Enter SORT mode. </summary>
///
/// <param name="str"> Raw text with optional function arguments. </param>
/// <param name="Write"> Function pointer to a write function (UART, USB). </param>
//---------------------------------------------------------------------
static void Function_SORT(char* str, write_func Write)
{
    // Echo
    Write((uint8_t*)"SORT", 4);

    g_mode = SORT;
}

//---------------------------------------------------------------------
/// <summary> Enter CONFIG mode. </summary>
///
/// <param name="str"> Raw text with optional function arguments. </param>
/// <param name="Write"> Function pointer to a write function (UART, USB). </param>
//---------------------------------------------------------------------
static void Function_CONF(char* str, write_func Write)
{
    // Echo
    Write((uint8_t*)"CONF", 4);

    g_mode = CONFIG;
}

//---------------------------------------------------------------------
/// <summary> Get version. </summary>
///
/// <param name="str"> Raw text with optional function arguments. </param>
/// <param name="Write"> Function pointer to a write function (UART, USB). </param>
//---------------------------------------------------------------------
static void Function_VERG(char* str, write_func Write)
{
    char buf[100] = {0};
    snprintf(buf, sizeof(buf), "VERG,%s,%s", PROJECT_TITLE, VERSION);
    Write((uint8_t*)buf, strlen(buf));
}

//---------------------------------------------------------------------
/// <summary> Set verbose level (0 - Off, !0 - On). </summary>
///
/// <param name="str"> Raw text with optional function arguments. </param>
/// <param name="Write"> Function pointer to a write function (UART, USB). </param>
//---------------------------------------------------------------------
static void Function_VRBS(char* str, write_func Write)
{
    str             = strtok(NULL, Delims);
    int verbose_lvl = atoi((char*)str);

    // Reset packet ID when entering verbose mode
    if (verbose_lvl != 0)
        ResetHeaderID();

    g_verbose_level = verbose_lvl;

    // Echo
    char buf[10] = {0};
    snprintf(buf, sizeof(buf), "VRBS,%u", verbose_lvl);
    Write((uint8_t*)buf, strlen(buf));
}

//---------------------------------------------------------------------
/// <summary> Get verbose level. </summary>
///
/// <param name="str"> Raw text with optional function arguments. </param>
/// <param name="Write"> Function pointer to a write function (UART, USB). </param>
//---------------------------------------------------------------------
static void Function_VRBG(char* str, write_func Write)
{
    char buf[10] = {0};
    snprintf(buf, sizeof(buf), "VRBG,%u", g_verbose_level);

    Write((uint8_t*)buf, strlen(buf));
}

//---------------------------------------------------------------------
/// <summary> uC UART ID SET. </summary>
///
/// <param name="str"> Raw text with optional function arguments. </param>
/// <param name="Write"> Function pointer to a write function (UART, USB). </param>
//---------------------------------------------------------------------
static void Function_ID_S(char* str, write_func Write)
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
    snprintf(buf, sizeof(buf), "ID_S,%u", UART_Address);
    Write((uint8_t*)buf, strlen(buf));
}

//---------------------------------------------------------------------
/// <summary> uC UART ID GET. </summary>
///
/// <param name="str"> Raw text with optional function arguments. </param>
/// <param name="Write"> Function pointer to a write function (UART, USB). </param>
//---------------------------------------------------------------------
static void Function_ID_G(char* str, write_func Write)
{
    char buf[10] = {0};
    snprintf(buf, sizeof(buf), "ID_G,%u", UART_Address);
    Write((uint8_t*)buf, strlen(buf));
}

//---------------------------------------------------------------------
/// <summary> Reset uC. </summary>
///
/// <param name="str"> Raw text with optional function arguments. </param>
/// <param name="Write"> Function pointer to a write function (UART, USB). </param>
//---------------------------------------------------------------------
static void Function_RSET(char* str, write_func Write)
{
    // Echo
    Write((uint8_t*)"RSET", 4);

    // Give it time to send string back
    HAL_Delay(100);

    NVIC_SystemReset();
}

//---------------------------------------------------------------------
/// <summary> Train. </summary>
///
/// <param name="str"> Raw text with optional function arguments. </param>
/// <param name="Write"> Function pointer to a write function (UART, USB). </param>
//---------------------------------------------------------------------
static void Function_TRAN(char* str, write_func Write)
{
    Train();

    // Echo
    Write((uint8_t*)"TRAN", 4);
}

//---------------------------------------------------------------------
/// <summary> Untrain. </summary>
///
/// <param name="str"> Raw text with optional function arguments. </param>
/// <param name="Write"> Function pointer to a write function (UART, USB). </param>
//---------------------------------------------------------------------
static void Function_UTRN(char* str, write_func Write)
{
    Untrain();

    // Echo
    Write((uint8_t*)"UTRN", 4);
}

//---------------------------------------------------------------------
/// <summary> Set frequency (example: "FRQS,1000" - 1000 Hz). </summary>
///
/// <param name="str"> Raw text with optional function arguments. </param>
/// <param name="Write"> Function pointer to a write function (UART, USB). </param>
//---------------------------------------------------------------------
static void Function_FRQS(char* str, write_func Write)
{
    str         = strtok(NULL, Delims);
    int freq_hz = atoi((char*)str);
    if (freq_hz > 0) { // WARNING: Currently we are using TIM1 which has 16bit ARR, so with our 1us resolution the minimum frequency we can get is approx. 15 Hz.
        requested_sampling_freq_hz = freq_hz;
        if (sequence_N > 0)
            SetSampleFrequency(requested_sampling_freq_hz * sequence_N);
    }

    // Echo
    char buf[20];
    snprintf(buf, sizeof(buf), "FRQS,%u", requested_sampling_freq_hz);
    Write((uint8_t*)buf, strlen(buf));
}

//---------------------------------------------------------------------
/// <summary> Set sorting parameters. </summary>
///
/// <param name="str"> Raw text with optional function arguments. </param>
/// <param name="Write"> Function pointer to a write function (UART, USB). </param>
//---------------------------------------------------------------------
static void Function_SRTS(char* str, write_func Write)
{
    str                    = strtok(NULL, Delims);
    g_delay_ticks_param    = atoi(str);
    str                    = strtok(NULL, Delims);
    g_duration_ticks_param = atoi(str);
    str                    = strtok(NULL, Delims);
    g_blind_ticks_param    = atoi(str);

    // Echo
    char buf[50];
    snprintf(buf, sizeof(buf), "SRTS,%u,%u,%u", g_delay_ticks_param, g_duration_ticks_param, g_blind_ticks_param);
    Write((uint8_t*)buf, strlen(buf));
}

//---------------------------------------------------------------------
/// <summary> Set filter parameters. </summary>
///
/// <param name="str"> Raw text with optional function arguments. </param>
/// <param name="Write"> Function pointer to a write function (UART, USB). </param>
//---------------------------------------------------------------------
static void Function_FILS(char* str, write_func Write)
{
    str      = strtok(NULL, Delims);
    g_lpf1_K = atof(str);
    str      = strtok(NULL, Delims);
    g_hpf_K  = atof(str);
    str      = strtok(NULL, Delims);
    g_lpf2_K = atof(str);

    // Echo
    char buf[80];
    snprintf(buf, sizeof(buf), "FILS,%.3f,%.3f,%.3f", g_lpf1_K, g_hpf_K, g_lpf2_K);
    Write((uint8_t*)buf, strlen(buf));
}

//---------------------------------------------------------------------
/// <summary> Set threshold value. </summary>
///
/// <param name="str"> Raw text with optional function arguments. </param>
/// <param name="Write"> Function pointer to a write function (UART, USB). </param>
//---------------------------------------------------------------------
static void Function_THRS(char* str, write_func Write)
{
    str         = strtok(NULL, Delims);
    g_threshold = atof(str);

    // Echo
    char buf[30];
    snprintf(buf, sizeof(buf), "THRS,%.1f", g_threshold);
    Write((uint8_t*)buf, strlen(buf));
}

//---------------------------------------------------------------------
/// <summary> Simple PING, to check if uC is alive. </summary>
///
/// <param name="str"> Raw text with optional function arguments. </param>
/// <param name="Write"> Function pointer to a write function (UART, USB). </param>
//---------------------------------------------------------------------
static void Function_PING(char* str, write_func Write)
{
    // Echo
    Write((uint8_t*)"PING", 4);
}

//---------------------------------------------------------------------
/// <summary> Get frequency. </summary>
///
/// <param name="str"> Raw text with optional function arguments. </param>
/// <param name="Write"> Function pointer to a write function (UART, USB). </param>
//---------------------------------------------------------------------
static void Function_FRQG(char* str, write_func Write)
{
    char buf[20] = {0};

    snprintf(buf, sizeof(buf), "FRQG,%u", requested_sampling_freq_hz);

    Write((uint8_t*)buf, strlen(buf));
}

//---------------------------------------------------------------------
/// <summary> Get sorting parameters. </summary>
///
/// <param name="str"> Raw text with optional function arguments. </param>
/// <param name="Write"> Function pointer to a write function (UART, USB). </param>
//---------------------------------------------------------------------
static void Function_SRTG(char* str, write_func Write)
{
    char buf[50] = {0};
    snprintf(buf, sizeof(buf), "SRTG,%u,%u,%u", g_delay_ticks_param, g_duration_ticks_param, g_blind_ticks_param);

    Write((uint8_t*)buf, strlen(buf));
}

//---------------------------------------------------------------------
/// <summary> Get filter parameters. </summary>
///
/// <param name="str"> Raw text with optional function arguments. </param>
/// <param name="Write"> Function pointer to a write function (UART, USB). </param>
//---------------------------------------------------------------------
static void Function_FILG(char* str, write_func Write)
{
    char buf[50] = {0};
    snprintf(buf, sizeof(buf), "FILG,%.3f,%.3f,%.3f", g_lpf1_K, g_hpf_K, g_lpf2_K);

    Write((uint8_t*)buf, strlen(buf));
}

//---------------------------------------------------------------------
/// <summary> Get threshold value. </summary>
///
/// <param name="str"> Raw text with optional function arguments. </param>
/// <param name="Write"> Function pointer to a write function (UART, USB). </param>
//---------------------------------------------------------------------
static void Function_THRG(char* str, write_func Write)
{
    char buf[20] = {0};
    snprintf(buf, sizeof(buf), "THRG,%.1f", g_threshold);

    Write((uint8_t*)buf, strlen(buf));
}

//---------------------------------------------------------------------
/// <summary> Get all settings. </summary>
///
/// <param name="str"> Raw text with optional function arguments. </param>
/// <param name="Write"> Function pointer to a write function (UART, USB). </param>
//---------------------------------------------------------------------
static void Function_STTG(char* str, write_func Write)
{
    char buf[300]; // don't need to zero it out
    snprintf(buf, sizeof(buf), "FREQ,%u\nSORTTICKS,%u,%u,%u\nFILTERCOEFF,%.3f,%.3f,%.3f\nTHRESHOLD,%.1f\n",
             requested_sampling_freq_hz, g_delay_ticks_param, g_duration_ticks_param, g_blind_ticks_param, g_lpf1_K, g_hpf_K, g_lpf2_K, g_threshold);

    Write((uint8_t*)buf, strlen(buf));
}

//---------------------------------------------------------------------
/// <summary> Set IR LED sequence. </summary>
///
/// <param name="str"> Raw text with optional function arguments. </param>
/// <param name="Write"> Function pointer to a write function (UART, USB). </param>
//---------------------------------------------------------------------
static void Function_SEQS(char* str, write_func Write)
{
    int seq[10];
    int i = 0;
    while ((str = strtok(NULL, Delims)) != NULL) {
        seq[i++] = atoi(str);
    }

    SetSequence(seq, i);
    SetSampleFrequency(requested_sampling_freq_hz * sequence_N);

    // Echo
    char buf[50];
    char seq_buf[10];
    snprintf(buf, sizeof(buf), "SEQS,%s", GetSequence(seq_buf, sizeof(seq_buf)));
    Write((uint8_t*)buf, strlen(buf));
}

//---------------------------------------------------------------------
/// <summary> Set sync (sync pin as input (slave) or output (master)). </summary>
///
/// <param name="str"> Raw text with optional function arguments. </param>
/// <param name="Write"> Function pointer to a write function (UART, USB). </param>
//---------------------------------------------------------------------
static void Function_SYNS(char* str, write_func Write)
{
    str              = strtok(NULL, Delims);
    int sync_enabled = atoi(str);

    if (sync_enabled)
        SetSyncPinAsOutput();
    else
        SetSyncPinAsInput();

    // Echo
    char buf[20];
    snprintf(buf, sizeof(buf), "SYNS,%d", sync_enabled);
    Write((uint8_t*)buf, strlen(buf));
}

static void Function_VALV(char* str, write_func Write)
{
    str                      = strtok(NULL, Delims);
    unsigned int valve_idx   = atoi(str);
    str                      = strtok(NULL, Delims);
    unsigned int valve_state = atoi(str);

    SetValve(valve_idx, valve_state);

    // Echo
    char buf[20];
    snprintf(buf, sizeof(buf), "VALV,%u,%u", valve_idx, valve_state);
    Write((uint8_t*)buf, strlen(buf));
}

static void Function_FLTT(char* str, write_func Write)
{
    str            = strtok(NULL, Delims);
    int filter_num = atoi(str);

    int ret = ChooseFilterType(filter_num);

    // Echo
    char buf[20];
    if (!ret) // ok
        snprintf(buf, sizeof(buf), "FLTT,%u", filter_num);
    else // filter does not exist
        snprintf(buf, sizeof(buf), "FLTT,N/A");

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
    COMMAND(SORT), // SORT MODE
    COMMAND(CONF), // CONFIG MODE
    COMMAND(VERG), // GET VERSION
    COMMAND(VRBG), // GET VERBOSE LEVEL
    COMMAND(VRBS), // SET VERBOSE LEVEL
    COMMAND(ID_S), // SET ID
    COMMAND(ID_G), // GET ID
    COMMAND(PING), // PING (echo)
    COMMAND(RSET), // RESET
    COMMAND(TRAN), // TRAIN
    COMMAND(UTRN), // UNTRAIN

    COMMAND(FRQG), // GET FREQUENCY
    COMMAND(SRTG), // GET SORTING TICKS
    COMMAND(FILG), // GET FILTER COEFFICIENTS
    COMMAND(THRG), // GET THREASHOLD
    COMMAND(STTG), // GET (ALL) SETTINGS

    COMMAND(FRQS), // SET FREQUENCY
    COMMAND(SRTS), // SET SORTING TICKS
    COMMAND(FILS), // SET FILTER COEFFICIENTS
    COMMAND(THRS), // SET THREASHOLD

    COMMAND(SEQS), // Sequence set, seq1, seq2, ... (e.g. SEQS,EVEN,ODD). For now the default are 2 sequences.
    COMMAND(SYNS), // Sync Ouptut set, on/off (1/0)

    COMMAND(VALV), // Valve control
    COMMAND(FLTT), // Choose filter type
};

//---------------------------------------------------------------------
/// <summary> Parse commands. </summary>
///
/// <param name="string"> Raw command text. </param>
/// <param name="Write"> Function pointer to a write function (UART, USB). </param>
//---------------------------------------------------------------------
void Parse(char* string, write_func Write)
{
    char* str;

    str = strtok(string, Delims);
    while (str != NULL) {

        for (int i = 0; i < sizeof(command) / sizeof(command[0]); ++i) {
            if (*(uint32_t*)str == *(uint32_t*)command[i].name) {
                command[i].Func(str, Write);
                break;
            }
        }

        str = strtok(NULL, Delims);
    }
}
