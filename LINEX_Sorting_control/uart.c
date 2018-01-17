#include "uart.h"

extern uint16_t trigger_output;
extern uint16_t detected_objects;

UART_HandleTypeDef UartHandle;

#define BUF_422_SIZE	10
uint8_t rxBuf_422[BUF_422_SIZE] = {0};
uint8_t txBuf_422[BUF_422_SIZE] = {0xBE, 0, 0, 0xEF, 0};

// IRQ
/////////////////////////////
void USARTx_IRQHandler() {
	HAL_UART_IRQHandler(&UartHandle);
}

__attribute__((optimize("O2"))) void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) {
}

__attribute__((optimize("O2"))) void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {

	if (rxBuf_422[0] == 0xDE && rxBuf_422[3] == 0xAD) {
		uint16_t tmp = ((uint16_t)rxBuf_422[1] << 8) | rxBuf_422[2];
		trigger_output = tmp;
	}
	UART_read((uint8_t*)rxBuf_422, BUF_422_SIZE);
			
	__disable_irq();
	uint16_t tmp = detected_objects;
	detected_objects = 0;
	__enable_irq();
			
	txBuf_422[1] = (tmp >> 8) & 0xFF;
	txBuf_422[2] = tmp & 0xFF;			
	UART_write(txBuf_422, BUF_422_SIZE);					
}
/////////////////////////////

void HAL_UART_MspInit(UART_HandleTypeDef *huart) {  
	
	GPIO_InitTypeDef  GPIO_InitStruct;
  	
	/* Enable GPIO TX/RX clock */
	USARTx_TX_GPIO_CLK_ENABLE();
	USARTx_RX_GPIO_CLK_ENABLE();
	/* Enable USART clock */
	USARTx_CLK_ENABLE(); 

	/* UART TX GPIO pin configuration  */
	GPIO_InitStruct.Pin       = USARTx_TX_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = USARTx_TX_AF;
  
	HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);
    
	/* UART RX GPIO pin configuration  */
	GPIO_InitStruct.Pin = USARTx_RX_PIN;
	GPIO_InitStruct.Alternate = USARTx_RX_AF;
    
	HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);
    
	/* NVIC for USARTx */
	HAL_NVIC_SetPriority(USARTx_IRQn, 1, 1);
	HAL_NVIC_EnableIRQ(USARTx_IRQn);
}

void UART_init() {
	UartHandle.Instance        = USARTx;

	UartHandle.Init.BaudRate   = 115200;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits   = UART_STOPBITS_1;
	UartHandle.Init.Parity     = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode       = UART_MODE_TX_RX;
	UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
	
	HAL_UART_Init(&UartHandle);	
	
	UART_read((uint8_t*)rxBuf_422, BUF_422_SIZE);
}

void UART_read(uint8_t* data_in, int len) {
	HAL_UART_Receive_IT(&UartHandle, data_in, len);
}

void UART_write(uint8_t* data_out, int len) {
	HAL_UART_Transmit_IT(&UartHandle, data_out, len);
}