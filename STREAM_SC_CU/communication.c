#include "communication.h"
#include "main.h"
#include "parse.h"
#include "uart.h"
#include <string.h>

extern uint8_t       UART_Address;
extern const uint8_t CharacterMatch;
extern Mode          g_mode;

int VCP_read(void* pBuffer, int size);
int VCP_write(const void* pBuffer, int size);

static uint8_t rx_buffer[UART_BUFFER_SIZE];
static int     rx_buffer_size = 0;

// UART driver Callback
///////////////////////
void UART_RX_Complete_Callback(const uint8_t* data, int size)
{
    rx_buffer_size = size;
    memcpy(rx_buffer, data, size);
    rx_buffer[size] = 0;
    EXTI->SWIER     = EXTI_SWIER_SWIER0; // This triggers EXTI interrupt
}
///////////////////////

// IRQ
//////
static void UART_RX_Process();
void        EXTI0_IRQHandler(void)
{
    EXTI->PR = EXTI_PR_PR0; // Clear pending bit
    UART_RX_Process();
}
//////

static void UART_RX_Process()
{
    // If we are in sorting mode then extract data
    if (g_mode == SORT) {
        for (int i = 0; i < rx_buffer_size; ++i) {
            uint8_t rx_byte = rx_buffer[i];
            if ((rx_byte & 0x30) == 0x30) { // Data
                if (i % 2 == 0)
                    rx_buffer[i / 2] = (rx_byte & 0x0F) << 4;
                else
                    rx_buffer[i / 2] |= (rx_byte & 0x0F);
            } else if (rx_byte == 0x1B) { // Escape
                g_mode = CONFIG;
                return;
            }
        }
    }

    COM_UART_RX_Complete_Callback(rx_buffer, rx_buffer_size);
}

// External callback - this function should contain per-program needed functionality, as so making sense to not include that code here
__weak void COM_UART_RX_Complete_Callback(uint8_t* buf, int size)
{
}

int UARTWrite(const uint8_t* buffer, int size)
{
    uint8_t buf[UART_BUFFER_SIZE];
    int     len = 0;

    if (size <= 0)
        return 0;

    if (g_mode == CONFIG) {
        int packet_size = 1 + size + 1; // 1 - address byte, + size - payload, + 1 - terminating character

        if (packet_size > UART_BUFFER_SIZE)
            return 0;

        buf[0] = 0x80 | UART_Address; // add origin address byte
        memcpy(&buf[1], buffer, size);
        buf[packet_size - 1] = CharacterMatch; // add terminating character
        len                  = UART_Write(buf, packet_size);
    } else if (g_mode == SORT) {
        int packet_size = 1 + size * 2 + 1; // 1 - address byte, size*2 - payload (each byte is split into 2 send bytes), + 1 - terminating character

        if (packet_size > UART_BUFFER_SIZE)
            return 0;

        buf[0] = 0x80 | UART_Address; // add origin address byte
        for (int i = 0; i < size; ++i) {
            uint8_t tmp_data = ((uint8_t*)buffer)[i];
            buf[1 + i * 2]   = 0x30 | (tmp_data >> 4);
            buf[2 + i * 2]   = 0x30 | (tmp_data & 0x0F);
        }

        buf[packet_size - 1] = CharacterMatch; // add terminating character
        len                  = UART_Write(buf, packet_size);
    }
    return len;
}

int USBRead(uint8_t* buffer, int max_size)
{
    static uint32_t last_read_tick = 0;
    static int      total_read     = 0;
    int             read           = 0;
    // Read while there is data in buffer
    /* NOTE: Added delayed return since there was a problem with USB driver which reads data in 64 byte chunks.
       This will erronously parse data not received in its entirety (over 64 bytes)).
       I have measured, and it takes approx 200 us between receptions of 64 bytes packets. */
    if ((read = VCP_read(&buffer[total_read], max_size - total_read)) > 0) {
        last_read_tick = HAL_GetTick();
        total_read += read;
    } else if (total_read > 0 && ((HAL_GetTick() - last_read_tick) > 3)) { // at least 3 ms ("terminal.exe" sometimes takes > 2 ms when sending files, so this is to be on the safe side, otherwise try to use some other/better program, I use a hand written one)
        buffer[total_read] = 0;
        int tmp            = total_read;
        total_read         = 0;
        return tmp;
    }

    return 0;
}

int USBWrite(const uint8_t* buffer, int size)
{
    uint8_t buf[UART_BUFFER_SIZE];
    int     len = 0;

    if (size <= 0 || size > (UART_BUFFER_SIZE - 1)) // -1 for newline at the end
        return 0;

    memcpy(buf, buffer, size);
    buf[size++] = CharacterMatch; // add terminating character
    len         = VCP_write(buf, size);

    return len; // len will be size + 1 (because of terminating character)
}