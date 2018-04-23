#include "protocol.h"
#include "uart.h"
#include <string.h>

uint8_t	Device_Address = 0;

static void ReceivePacket(uint8_t* data, int size) {
	for (int i = 0, i < size; ++i) {
		uint8_t tmp_data = data[i];
		
		if ((tmp_data & 0x30) == 0x30) { // Data
			if (byte_cnt % 2 == 0)
				data[byte_cnt / 2] = (tmp_data & 0x0F) << 4;
			else 
				data[byte_cnt / 2] |= tmp_data & 0x0F;
			byte_cnt++;
			if ((byte_cnt / 2) >= max_len) { // input data buffer full
				break;
			}
		} else if ((tmp_data & 0x10) == 0x10) {	// Command
			if (tmp_data == 0x1B) {	// Escape
			}
		}
	}
}

void UART_Char_Match_Callback(uint8_t* data, int size) {
	uint8_t buf[100];
	
	memcpy(buf, data, size > sizeof(buf) ? sizeof(buf) : size);
	
	uint8_t addr = 0x80 | Device_Address;
	for (int i = 0; i < size; ++i) {
		uint8_t tmp_data = data[i];
		if (tmp_data == addr || tmp_data == (0x80 | BROADCAST_ADDRESS)) {
			ReceivePacket(&data[i], size - i);
		}
	}		
}

int Protocol_Write(uint8_t* data, int size) {
	uint8_t buf[UART_BUFFER_SIZE];
	int packet_size = 1 + size * 2; // 1 - address byte, *2 - each byte is split into 2 send bytes
	
	if (packet_size > UART_BUFFER_SIZE)
		return 0;
	
	buf[0] = 0x80 | Device_Address;	// add origin address byte
	for (int i = 0; i < size; ++i) {
		uint8_t tmp_data = data[i];
		buf[1 + i*2] = 0x30 | (tmp_data >> 4);
		buf[2 + i*2] = 0x30 | (tmp_data & 0x0F);
	}
	
	return UART_Write(buf, packet_size);
}