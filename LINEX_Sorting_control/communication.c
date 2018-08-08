#include "main.h"
#include "communication.h"
#include "uart.h"
#include "parse.h"
#include <string.h>

extern uint8_t UART_Address;
extern uint16_t g_trigger_output;
extern uint16_t g_detected_objects;
extern const uint8_t CharacterMatch;


int VCP_read(void *pBuffer, int size);
int VCP_write(const void *pBuffer, int size);

CommunicationInterface g_communication_interface = UART;
CommunicationMode g_communication_mode = ASCII;

static uint8_t rx_buffer[UART_BUFFER_SIZE];

void UART_RX_Complete_Callback(const uint8_t* data, int size) {
	if (g_communication_interface == UART) {
		memcpy(rx_buffer, data, size);
		rx_buffer[size] = 0;
		// could trigger software irq here which has smaller priority than UART and then exit this function
		//EXTI->SWIER = EXTI_SWIER_SWIER0;
	
		if (g_communication_mode == ASCII) { // ASCII mode
			Parse((char*)rx_buffer);
		} else { // Binary mode
			for (int i = 0; i < size; ++i) {
				uint8_t rx_byte = rx_buffer[i];
				if ((rx_byte & 0x30) == 0x30) { // Data
					if (i % 2 == 0) rx_buffer[i/2] = (rx_byte & 0x0F) << 4;
					else rx_buffer[i/2] |= (rx_byte & 0x0F);
				} else if (rx_byte == 0x1B) { // Escape
					g_communication_mode = ASCII;
					return;
				}
			}
		
			g_trigger_output = ((uint16_t)rx_buffer[0] << 8) | rx_buffer[1];
				
			__disable_irq();
			uint16_t det_obj = g_detected_objects;
			g_detected_objects = 0;
			__enable_irq();
				
			uint8_t txBuf[2];
			txBuf[0] = (det_obj >> 8) & 0xFF;
			txBuf[1] = det_obj & 0xFF;
			Write(txBuf, sizeof(txBuf));
		}
	}
}

int Read(uint8_t* buffer, int max_size) {
	int len = 0;
	
	if (g_communication_interface == USB) {
		// This silly loop with delays seems neccessary at least when using PC program terminal.exe when sending a file (not when sending normally via command line)		
		int tmp = 0;
		while ((tmp = VCP_read(&buffer[len], max_size - len)) > 0) {
			len += tmp;
			for (int i = 0; i < 100000; ++i);	// improvised Delay
		} 
		buffer[len] = 0;
		if (g_communication_mode == BINARY) {
			if (strncmp((const char*)buffer, "ASCII", 5) == 0) { // Escape from BIN to ASCII mode
				return -1;
			}
		}
	}	

	return len;
}

int Write(const uint8_t* buffer, int size) {
	uint8_t buf[UART_BUFFER_SIZE];
	int len = 0;		
	
	if (size <= 0) return 0;
	
	if (g_communication_interface == USB) {
		memcpy(buf, buffer, size);
		len = VCP_write(buffer, size);
	} else {
		if (g_communication_mode == ASCII) {
			int packet_size = 1 + size + 1; // 1 - address byte, + size - payload, + 1 - terminating character
			
			if (packet_size > UART_BUFFER_SIZE)
				return 0;
			
			buf[0] = 0x80 | UART_Address;	// add origin address byte
			memcpy(&buf[1], buffer, size);
			buf[packet_size-1] = CharacterMatch;	// add terminating character
			len = UART_Write(buf, packet_size);
		} else {
			int packet_size = 1 + size * 2 + 1; // 1 - address byte, size*2 - payload (each byte is split into 2 send bytes), + 1 - terminating character
	
			if (packet_size > UART_BUFFER_SIZE)
				return 0;
			
			buf[0] = 0x80 | UART_Address;	// add origin address byte
			for (int i = 0; i < size; ++i) {
				uint8_t tmp_data = ((uint8_t*)buffer)[i];
				buf[1 + i*2] = 0x30 | (tmp_data >> 4);
				buf[2 + i*2] = 0x30 | (tmp_data & 0x0F);
			}
			
			buf[packet_size-1] = CharacterMatch;	// add terminating character
			len = UART_Write(buf, packet_size);
		}
	}

	return len;
}