#include "protocol.h"
#include "uart.h"

int8_t	Device_Address = 0;

int Protocol_Read(uint8_t* data, int max_len) {
	int read = 0, tmp = 0, byte_cnt = 0;
	uint8_t buf[UART_BUFFER_SIZE];
	
	do {
		tmp = UART_Read(&buf[read], UART_BUFFER_SIZE - read);
		if (tmp > 0) {
			HAL_Delay(1);
			read += tmp;
		}
	} while (tmp);
	
	for (int i = 0, data_for_me = 0; i < read; ++i) {
		uint8_t tmp_data = buf[i];
		
		if (tmp_data & 0x80) {	// address byte
			if (((tmp_data & 0x7F) == Device_Address) || ((tmp_data & 0x7F) == BROADCAST_ADDRESS)) {
				data_for_me = 1;
				byte_cnt = 0;
			} else {
				data_for_me = 0;
			}
		} else if (data_for_me) {
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
	
	if (byte_cnt % 2)
		return -1;
	else	
		return byte_cnt / 2;
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