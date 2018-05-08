#include <string.h>
#include "uart.h"
#include "flash.h"

const uint8_t CharacterMatch = 0x0A;	// Newline
static UART_HandleTypeDef UartHandle;

uint8_t	UART_Address = 0;

static struct {
	uint8_t data[UART_BUFFER_SIZE];
	int		i;
} uart_rx_buffer;

static struct {
	uint8_t data[UART_BUFFER_SIZE];
	int		i, size;
} uart_tx_buffer;

void USARTx_IRQHandler() {
	uint32_t isrflags = USARTx->ISR;
	uint32_t cr1its = USARTx->CR1;

	if ((isrflags & USART_ISR_RXNE) && (cr1its & USART_CR1_RXNEIE)) {
		uint8_t rx_byte = USARTx->RDR;
		if (rx_byte == CharacterMatch) {
			HAL_MultiProcessor_EnterMuteMode(&UartHandle);
			uart_rx_buffer.data[uart_rx_buffer.i] = 0;
			UART_RX_Complete_Callback(uart_rx_buffer.data, uart_rx_buffer.i);
			uart_rx_buffer.i = 0;
		} else if (uart_rx_buffer.i < UART_BUFFER_SIZE-1 && rx_byte < 0x80) { // -1 to fit terminating zero, rx_byte < 0x80 don't copy the address byte
			uart_rx_buffer.data[uart_rx_buffer.i++] = rx_byte;
		}
	}

	if ((isrflags & USART_ISR_TXE) && (cr1its & USART_CR1_TXEIE)) {
		USARTx->TDR = uart_tx_buffer.data[uart_tx_buffer.i++];
		if (uart_tx_buffer.i >= uart_tx_buffer.size) 
			USARTx->CR1 &= ~USART_CR1_TXEIE;
	}
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
	GPIO_InitTypeDef  GPIO_InitStruct;

	RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;

	/*##-1- Enable peripherals and GPIO Clocks #################################*/
	/* Enable GPIO TX/RX clock */
	USARTx_TX_GPIO_CLK_ENABLE();
	USARTx_RX_GPIO_CLK_ENABLE();

	/* Select SysClk as source of USART1 clocks */
	RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
	RCC_PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK;
	HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

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
	GPIO_InitStruct.Pin = USARTx_RX_PIN;
	GPIO_InitStruct.Alternate = USARTx_RX_AF;

	HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);		
}

void UART_Set_Address(uint8_t addr) {
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

void UART_Init() {
	UartHandle.Instance        = USARTx;

	UartHandle.Init.BaudRate   = 115200;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits   = UART_STOPBITS_1;
	UartHandle.Init.Parity     = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode       = UART_MODE_TX_RX;
	UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;		
	
	uint8_t id = FLASH_ReadID();
	if (id < 128) UART_Address = id;
	
	HAL_MultiProcessor_Init(&UartHandle, UART_Address, UART_WAKEUPMETHOD_ADDRESSMARK);
	HAL_MultiProcessor_EnableMuteMode(&UartHandle);
	HAL_MultiProcessor_EnterMuteMode(&UartHandle);
	
	HAL_NVIC_SetPriority(USARTx_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(USARTx_IRQn);
	
	USARTx->CR1 |= USART_CR1_RXNEIE;		
}

int UART_Write(const uint8_t* data, int size) {
	int ret = 0;
	
	if (size > 0 && !(USARTx->CR1 & USART_CR1_TXEIE)) {
		uart_tx_buffer.size = size > sizeof(uart_tx_buffer.data) ? sizeof(uart_tx_buffer.data) : size;
		ret = uart_tx_buffer.size;
		uart_tx_buffer.i = 0;
		memcpy(uart_tx_buffer.data, data, uart_tx_buffer.size);
		USARTx->CR1 |= USART_CR1_TXEIE;
	}
	
	return ret;
}


// Weak callback functions - to be implemented by the user in protocol specific section
__weak void UART_RX_Complete_Callback(const uint8_t* data, int size) {
	UNUSED(data);
	UNUSED(size);
}