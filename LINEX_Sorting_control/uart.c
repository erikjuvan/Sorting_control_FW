#include <string.h>
#include "uart.h"

UART_HandleTypeDef UartHandle;

static struct {
	uint8_t data[UART_BUFFER_SIZE];
	int		i;
	int		overflow;
} uart_rx_buffer;

static struct {
	uint8_t data[UART_BUFFER_SIZE];
	int		i, size;
} uart_tx_buffer;

void USARTx_IRQHandler() {
	uint32_t isrflags = USARTx->ISR;
	uint32_t cr1its = USARTx->CR1;

	if ((isrflags & USART_ISR_RXNE) && (cr1its & USART_CR1_RXNEIE)) {
		if (uart_rx_buffer.i < UART_BUFFER_SIZE) {
			uart_rx_buffer.data[uart_rx_buffer.i++] = USARTx->RDR;
		} else {
			uart_rx_buffer.overflow = 1;
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

void UART_Init() {
	UartHandle.Instance        = USARTx;

	UartHandle.Init.BaudRate   = 115200;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits   = UART_STOPBITS_1;
	UartHandle.Init.Parity     = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode       = UART_MODE_TX_RX;
	UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
	
	HAL_UART_Init(&UartHandle);	
	
	HAL_NVIC_SetPriority(USARTx_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(USARTx_IRQn);
	
	USARTx->CR1 |= USART_CR1_RXNEIE;
}

int UART_CheckAndClearOverflow() {
	int of = uart_rx_buffer.overflow;
	uart_rx_buffer.overflow = 0;
	return of;
}

int UART_BytesToRead() {
	return uart_rx_buffer.i;
}

int UART_Read(uint8_t* data, int max_len) {
	if (uart_rx_buffer.i <= 0) {
		return 0;
	} else {	// There is data
		USARTx->CR1 &= ~USART_CR1_RXNEIE;
		int len = uart_rx_buffer.i > max_len ? max_len : uart_rx_buffer.i;
	
		memcpy(data, uart_rx_buffer.data, len);
		uart_rx_buffer.i = 0;
		USARTx->CR1 |= USART_CR1_RXNEIE;
		return len;
	}
}

int UART_Write(uint8_t* data, int size) {
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

