#include "parse.h"
#include "main.h"
#include "communication.h"
#include "uart.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

extern int g_protocol_ascii;
extern int g_training;
extern int g_add_trigger_info;
extern struct SystemParameters g_systemParameters;
extern DisplayData g_display_data;

extern float A1;
extern float A2;
extern float A4;
extern float FTR_THRSHLD;

extern uint32_t T_delay;
extern uint32_t T_duration;
extern uint32_t T_blind;

extern int skip_2nd;
extern int skip_2nd_cntr[N_CHANNELS];

extern void ChangeSampleFrequency();

extern uint8_t UART_Address;

const static char Delims[] = "\n\r\t, ";

static void Function_STRT(char* str) {
	g_protocol_ascii = 0;
}

static void Function_VERG(char* str) {
	char buf[100] = {0};
	strncpy(&buf[strlen(buf)], "VERG,", 5);
	strncpy(&buf[strlen(buf)], SWVER, strlen(SWVER));
	buf[strlen(buf)] = ',';
	strncpy(&buf[strlen(buf)], HWVER, strlen(HWVER));
	buf[strlen(buf)] = ',';
	strncpy(&buf[strlen(buf)], COMPATIBILITYMODE, strlen(COMPATIBILITYMODE));		
	Write((uint8_t*)buf, strlen(buf), g_protocol_ascii);
}

static void Function_VRBS(char* str) {
	str = strtok(NULL, Delims);
	g_systemParameters.verbose_level = atoi((char*)str);
}

static void Function_VRBG(char* str) {
	char buf[10] = {0};
	strncpy(buf, "VRBG,", 5);
	itoa(g_systemParameters.verbose_level, &buf[strlen(buf)], 10);
	
	Write((uint8_t*)buf, strlen(buf), 1);
}

static void Function_IDST(char* str) {
	str = strtok(NULL, Delims);
	if (str != NULL) {
		int num = atoi(str);
		if (num <= 127) {
			UART_Set_Address(num);
		}
	}
}

static void Function_IDGT(char* str) {	
	char buf[10] = {0};
	strncpy(buf, "ID:", 3);
	itoa(UART_Address, &buf[strlen(buf)], 10);
	
	Write((uint8_t*)buf, strlen(buf), 1);			
}

static void Function_USBY(char* str) {
	Communication_Set_USB();
}

static void Function_USBN(char* str) {
	Communication_Set_UART();
}

static void Function_CSETF(char* str) {
	// "CSETF,1000" - 1000 is in hertz
	str = strtok(NULL, Delims);
	int val = atoi((char*)str);		
	g_systemParameters.timer_period = 1e6 / val;	// convert val which are hertz to period which is in us 
	ChangeSampleFrequency();
}

static void Function_CRESET(char* str) {
}

static void Function_CTRAIN(char* str) {
	g_training = 1000;
}

static void Function_CRAW(char* str) {
	g_display_data = RAW;
}

static void Function_CTRAINED(char* str) {
	g_display_data = TRAINED;
}

static void Function_CFILTERED(char* str) {
	g_display_data = FILTERED;
}

static void Function_CPARAMS(char* str) {
	str = strtok(NULL, ",");
	A1 = atof(str);		
	str = strtok(NULL, ",");
	A2 = atof(str);	
	str = strtok(NULL, ",");
	A4 = atof(str);	
	str = strtok(NULL, Delims);
	FTR_THRSHLD = atof(str);
}

static void Function_CTIMES(char* str) {
	str = strtok(NULL, ",");
	T_delay = atoi(str);		
	str = strtok(NULL, ",");
	T_duration = atoi(str);	
	str = strtok(NULL, Delims);
	T_blind = atoi(str);	
}

static void Function_CSKIPSCND(char* str) {
	str = strtok(NULL, Delims);
	if (atoi(str) == 1) {
		skip_2nd = 1;
		for (int i = 0; i < N_CHANNELS; ++i)
			skip_2nd_cntr[i] = 0;
	} else {
		skip_2nd = 0;
	}
}

static void Function_PING(char* str) {
	Write((uint8_t*) "OK", 2, 1);
}

static void Function_TRGFRM(char* str) {
	if (g_add_trigger_info) g_add_trigger_info = 0;
	else g_add_trigger_info = 1;
}

static struct {
	const char* name;
	void (*Func)(char*);
} command[] = {
	{"STRT", Function_STRT},
	{"VERG", Function_VERG},
	{"VRBS", Function_VRBS},
	{"VRBG", Function_VRBG},
	{"IDST", Function_IDST},
	{"IDGT", Function_IDGT},
	{"USBY", Function_USBY},
	{"USBN", Function_USBN},
	{"PING", Function_PING},
	{"TRGFRM", Function_TRGFRM},
	
	// Legacy
	{"CSETF", Function_CSETF},	
	{"CRESET", Function_CRESET},
	{"CTRAIN", Function_CTRAIN},
	{"CRAW", Function_CRAW},
	{"CTRAINED", Function_CTRAINED},
	{"CFILTERED", Function_CFILTERED},
	{"CPARAMS", Function_CPARAMS},
	{"CTIMES", Function_CTIMES},
	{"CSKIPSCND", Function_CSKIPSCND},
};

void Parse(char* string) {
	char* str;
	
	str = strtok(string, Delims);
	while (str != NULL) {
		
		for (int i = 0; i < sizeof(command) / sizeof(command[0]); ++i) {
			if (strcmp(str, command[i].name) == 0) {
				command[i].Func(str);
				break;
			}
		}		
		
		str = strtok(NULL, Delims);
	}
}
