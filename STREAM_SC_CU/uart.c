#include "uart.h"
#include "flash.h"
#include <string.h>

const uint8_t             CharacterMatch = 0x0A; // Newline
static UART_HandleTypeDef UartHandle;

uint8_t UART_Address = 0;

static struct {
    uint8_t data[UART_BUFFER_SIZE];
    int     i;
} uart_rx_buffer = {.i = 0};

static struct {
    uint8_t data[UART_BUFFER_SIZE];
    int     i, size;
} uart_tx_buffer = {.i = 0, .size = 0};

void USARTx_IRQHandler()
{
    uint32_t isrflags = USARTx->ISR;
    uint32_t cr1its   = USARTx->CR1;

    if ((isrflags & USART_ISR_RXNE) && (cr1its & USART_CR1_RXNEIE)) {
        uint8_t rx_byte = USARTx->RDR;
        if (rx_byte == CharacterMatch) {
            HAL_MultiProcessor_EnterMuteMode(&UartHandle);
            UART_RX_Complete_Callback(uart_rx_buffer.data, uart_rx_buffer.i);
            uart_rx_buffer.i = 0;
        } else if (uart_rx_buffer.i < UART_BUFFER_SIZE - 1 && rx_byte < 0x80) { // -1 to fit terminating zero, rx_byte < 0x80 don't copy the address byte
            uart_rx_buffer.data[uart_rx_buffer.i++] = rx_byte;
        }
    }

    if ((isrflags & USART_ISR_TXE) && (cr1its & USART_CR1_TXEIE)) {
        USARTx->TDR = uart_tx_buffer.data[uart_tx_buffer.i++];
        if (uart_tx_buffer.i >= uart_tx_buffer.size) {
            USARTx->CR1 &= ~USART_CR1_TXEIE; // disable transmission
            uart_tx_buffer.i    = 0;
            uart_tx_buffer.size = 0;
        }
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    USARTx_TX_GPIO_CLK_ENABLE();
    USARTx_RX_GPIO_CLK_ENABLE();
    USARTx_DE_GPIO_CLK_ENABLE();

    /* Select SysClk as source of USART clocks */
    USARTx_RCC_PERIPHCLKINIT();

    /* Enable USARTx clock */
    USARTx_CLK_ENABLE();

    /*##-2- Configure peripheral GPIO ##########################################*/
    /* UART TX GPIO pin configuration  */
    GPIO_InitStruct.Pin       = USARTx_TX_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = USARTx_TX_AF;

    HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

    /* UART RX GPIO pin configuration  */
    GPIO_InitStruct.Pin       = USARTx_RX_PIN;
    GPIO_InitStruct.Alternate = USARTx_RX_AF;

    HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);

    // UART DE (Driver enable for RS-422) GPIO pin configuration
    GPIO_InitStruct.Pin       = USARTx_DE_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = USARTx_DE_AF;

    HAL_GPIO_Init(USARTx_DE_GPIO_PORT, &GPIO_InitStruct);
}

void UART_Set_Address(uint8_t addr)
{
    if (addr <= 127) {
        USARTx->CR1 &= ~USART_CR1_UE;
        MODIFY_REG(USARTx->CR2, USART_CR2_ADD, ((uint32_t)addr << UART_CR2_ADDRESS_LSB_POS));
        USARTx->CR1 |= USART_CR1_UE;

        if (addr == ((USARTx->CR2 & USART_CR2_ADD_Msk) >> UART_CR2_ADDRESS_LSB_POS)) {
            UART_Address = addr;
            FLASH_WriteID(UART_Address);
        }
    }
}

void UART_Init()
{
    UartHandle.Instance = USARTx;

    UartHandle.Init.BaudRate     = 115200;
    UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits     = UART_STOPBITS_1;
    UartHandle.Init.Parity       = UART_PARITY_NONE;
    UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    UartHandle.Init.Mode         = UART_MODE_TX_RX;
    UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

    uint8_t id = FLASH_ReadID();
    if (id < 128)
        UART_Address = id;

    HAL_RS485Ex_Init(&UartHandle, UART_DE_POLARITY_HIGH, 16, 16); // 16 - with oversampling 16, that comes out to 1 bit delay between DE(high) -> START, and STOP -> DE(low).
    HAL_MultiProcessor_Init(&UartHandle, UART_Address, UART_WAKEUPMETHOD_ADDRESSMARK);
    HAL_MultiProcessor_EnableMuteMode(&UartHandle);
    HAL_MultiProcessor_EnterMuteMode(&UartHandle);

    HAL_NVIC_SetPriority(USARTx_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USARTx_IRQn);

    USARTx->CR1 |= USART_CR1_RXNEIE;
}

int UART_Write(const uint8_t* data, int size)
{
    // Disable UART transmission to prevent uart_tx_buffer corruption
    USARTx->CR1 &= ~USART_CR1_TXEIE;

    // Check if new data fits
    if (uart_tx_buffer.size + size <= sizeof(uart_tx_buffer.data)) {
        memcpy(&uart_tx_buffer.data[uart_tx_buffer.size], data, size);
        uart_tx_buffer.size += size;
    } else {
        // Write an error message
        const char ovrflw[] = "OVRFLW";
        memcpy(uart_tx_buffer.data, ovrflw, sizeof(ovrflw));
        uart_tx_buffer.size = sizeof(ovrflw);
        uart_tx_buffer.i    = 0;
    }

    // Check if there is anything to send
    if (uart_tx_buffer.size > 0)
        USARTx->CR1 |= USART_CR1_TXEIE; // Enable transmission

    return uart_tx_buffer.size - uart_tx_buffer.i; // return remaining
}

// Weak callback functions - to be implemented by the user in protocol specific section
__weak void UART_RX_Complete_Callback(const uint8_t* data, int size)
{
    UNUSED(data);
    UNUSED(size);
}