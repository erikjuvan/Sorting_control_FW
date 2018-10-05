#include "parse.h"
#include "main.h"
#include "communication.h"
#include "uart.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

extern Mode g_mode;
extern CommunicationInterface g_communication_interface;

extern int g_training;
extern int g_add_trigger_info;
extern int g_timer_period;
extern int g_verbose_level;
extern DisplayData g_display_data;

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

static void Function_SORT(char* str,  write_func Write) {
	g_mode = SORT;
}

static void Function_CONFIG(char* str,  write_func Write) {
	g_mode = CONFIG;
}

static void Function_USB(char* str,  write_func Write) {
	g_communication_interface = USB;
}

static void Function_UART(char* str,  write_func Write) {
	g_communication_interface = UART;
}

static void Function_UART_SORT(char* str,  write_func Write) {
	g_communication_interface = UART;
	g_mode = SORT;
}
static void Function_VERG(char* str,  write_func Write) {
	char buf[100] = {0};	
	snprintf(buf, sizeof(buf), "VERG,%s,%s,%s\n", SWVER, HWVER, COMPATIBILITYMODE);
	Write((uint8_t*)buf, strlen(buf));
}

static void Function_VRBS(char* str,  write_func Write) {
	str = strtok(NULL, Delims);
	g_verbose_level = atoi((char*)str);
}

static void Function_VRBG(char* str,  write_func Write) {
	char buf[10] = {0};	
	snprintf(buf, sizeof(buf), "VRBG,%u\n", g_verbose_level);
	
	Write((uint8_t*)buf, strlen(buf));
}

static void Function_IDST(char* str,  write_func Write) {
	str = strtok(NULL, Delims);
	if (str != NULL) {
		int num = atoi(str);
		if (num <= 127) {
			UART_Set_Address(num);
		}
	}
}

static void Function_IDGT(char* str,  write_func Write) {	
	char buf[10] = {0};	
	snprintf(buf, sizeof(buf), "ID:%u\n", UART_Address);
	
	Write((uint8_t*)buf, strlen(buf));			
}

static void Function_CSETF(char* str,  write_func Write) {
	// "CSETF,1000" - 1000 is in hertz
	str = strtok(NULL, Delims);
	int val = atoi((char*)str);		
	g_timer_period = 1e6 / val;	// convert val which are hertz to period which is in us 
	ChangeSampleFrequency();
}

static void Function_CRESET(char* str,  write_func Write) {
}

static void Function_CTRAIN(char* str,  write_func Write) {
	g_training = 1000;
}

static void Function_CRAW(char* str,  write_func Write) {
	g_display_data = RAW;
}

static void Function_CTRAINED(char* str,  write_func Write) {
	g_display_data = TRAINED;
}

static void Function_CFILTERED(char* str,  write_func Write) {
	g_display_data = FILTERED;
}

static void Function_CGETVIEW(char* str, write_func Write) {
	char buf[10] = {0};	
	snprintf(buf, sizeof(buf), "%u\n", g_display_data);
	
	Write((uint8_t*)buf, strlen(buf));	
}

static void Function_CPARAMS(char* str,  write_func Write) {
	str = strtok(NULL, ",");
	A1 = atof(str);		
	str = strtok(NULL, ",");
	A2 = atof(str);	
	str = strtok(NULL, ",");
	A4 = atof(str);	
	str = strtok(NULL, Delims);
	FTR_THRSHLD = atof(str);
}

static void Function_CTIMES(char* str,  write_func Write) {
	str = strtok(NULL, ",");
	T_delay = atoi(str);		
	str = strtok(NULL, ",");
	T_duration = atoi(str);	
	str = strtok(NULL, Delims);
	T_blind = atoi(str);	
}

static void Function_PING(char* str,  write_func Write) {
	Write((uint8_t*) "OK\n", 2);
}

static void Function_TRGFRMS(char* str,  write_func Write) {
	str = strtok(NULL, Delims);
	int set = atoi((char*)str);
	if (set != 0)
		g_add_trigger_info = 1;
	else
		g_add_trigger_info = 0;
}

static void Function_TRGFRMG(char* str,  write_func Write) {
	char buf[10] = {0};	
	snprintf(buf, sizeof(buf), "%u\n", g_add_trigger_info);
	
	Write((uint8_t*)buf, strlen(buf));
}


static void Function_CGETF(char* str,  write_func Write) {
	char buf[10] = {0};
	int hz = 1e6 / g_timer_period;
	snprintf(buf, sizeof(buf), "%u\n", hz);
	
	Write((uint8_t*)buf, strlen(buf));
}

static void Function_CGETPARAMS(char* str,  write_func Write) {
	char buf[50] = {0};
	snprintf(buf, sizeof(buf), "%.2f,%.2f,%.2f,%.1f\n", A1, A2, A4, FTR_THRSHLD);
	
	Write((uint8_t*)buf, strlen(buf));
}

static void Function_CGETTIMES(char* str,  write_func Write) {
	char buf[50] = {0};
	snprintf(buf, sizeof(buf), "%u,%u,%u\n", T_delay, T_duration, T_blind);
	
	Write((uint8_t*)buf, strlen(buf));
}


static struct {
	const char* name;
	void (*Func)(char*,  write_func);
} command[] = {
	{"SORT", Function_SORT},
	{"CONFIG", Function_CONFIG},
	{"USB", Function_USB},
	{"UART", Function_UART},
	{"UART_SORT", Function_UART_SORT},
	{"VERG", Function_VERG},
	{"VRBS", Function_VRBS},
	{"VRBG", Function_VRBG},
	{"IDST", Function_IDST},
	{"IDGT", Function_IDGT},	
	{"PING", Function_PING},
	{"TRGFRMS", Function_TRGFRMS},
	{"TRGFRMG", Function_TRGFRMG},
	
	// New but built on legacy format
	{"CGETF", Function_CGETF},		
	{"CGETPARAMS", Function_CGETPARAMS},
	{"CGETTIMES", Function_CGETTIMES},
	{"CGETVIEW", Function_CGETVIEW},
	
	// Legacy
	{"CSETF", Function_CSETF},	
	{"CRESET", Function_CRESET},
	{"CTRAIN", Function_CTRAIN},
	{"CRAW", Function_CRAW},
	{"CTRAINED", Function_CTRAINED},
	{"CFILTERED", Function_CFILTERED},
	{"CPARAMS", Function_CPARAMS},
	{"CTIMES", Function_CTIMES},
};

void Parse(char* string, write_func Write) {
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
