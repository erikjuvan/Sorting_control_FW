#include "communication.h"
#include "uart.h"
#include <string.h>

int VCP_read(void *pBuffer, int size);
int VCP_write(const void *pBuffer, int size);

extern uint8_t UART_Address;

static uint8_t rx_buffer[UART_BUFFER_SIZE];
static uint16_t rx_buffer_size;

extern const uint8_t CharacterMatch;

static int usb = 0;

void UART_RX_Complete_Callback(const uint8_t* data, int size) {
	rx_buffer_size = size > sizeof(rx_buffer) ? sizeof(rx_buffer) : size;
	memcpy(rx_buffer, data, rx_buffer_size);	
}

int Read(uint8_t* buffer, int max_size, int ascii) {
	int len = 0;
	
	if (usb) {
		int tmp = 0;
		// This silly loop with delays seems neccessary at least when using PC program terminal.exe when sending a file (not when sending normally via command line)
		do {
			tmp = VCP_read(&buffer[len], max_size - len);
			len += tmp;
			HAL_Delay(1);
		} while (tmp > 0);
		buffer[len] = 0;
	} else {
		if (rx_buffer_size > 0 && rx_buffer_size < max_size) {
			len = rx_buffer_size;
			memcpy(buffer, rx_buffer, len);
			rx_buffer_size = 0;
			
			if (ascii) { // ASCII mode
				return len;
			} else { // Binary mode
				uint8_t* u8_data = buffer;
		
				for (int i = 0; i < len; ++i) {
					uint8_t rx_byte = rx_buffer[i];
					if ((rx_byte & 0x30) == 0x30) { // Data
						if (i % 2 == 0) u8_data[i/2] = (rx_byte & 0x0F) << 4;
						else u8_data[i/2] |= (rx_byte & 0x0F);
					} else if (rx_byte == 0x1B) { // Escape
						return -1;
					}
				}
				return len / 2;
			}			
		}
	}		
	
	return len;
}

int Write(const uint8_t* buffer, int size, int ascii) {
	uint8_t buf[UART_BUFFER_SIZE];
	int len = 0;		
	
	if (size <= 0) return 0;
	
	if (usb) {
		memcpy(buf, buffer, size);
		len = VCP_write(buffer, size);
	} else {
		if (ascii) {
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

void Communication_Set_USB() {
	usb = 1;
}

void Communication_Set_UART() {
	usb = 0;
}

int Communication_Get_USB() {
	return usb;
}