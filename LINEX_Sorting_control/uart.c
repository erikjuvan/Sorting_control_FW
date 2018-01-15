#include "uart.h"

UART_HandleTypeDef UartHandle;
__IO uint8_t UartRxComplete = 0;

// IRQ
/////////////////////////////
void USARTx_IRQHandler() {
	HAL_UART_IRQHandler(&UartHandle);
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) {
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
	UartRxComplete = 1;
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
	
	UartRxComplete = 0;
	
	HAL_UART_Init(&UartHandle);	
}

void UART_read(uint8_t* data_in, int len) {
	HAL_UART_Receive_IT(&UartHandle, data_in, len);
}

void UART_write(uint8_t* data_out, int len) {
	HAL_UART_Transmit_IT(&UartHandle, data_out, len);
}

/* Example

	UART_Init();

	char rxBuf_422[255] = {0};
	char rxBuf_usb[50] = {0};
	int read = 0, ret = 0;
	const char rxEcho[] = "PLC_RX_STATUS_OK";
	UART_Read((uint8_t*)rxBuf_422, 4);
	
	while (1) {
		
		ret = VCP_read(&rxBuf_usb[read], sizeof(rxBuf_usb) - read);
		while (ret) {
			read += ret;
			HAL_Delay(10);
			ret = VCP_read(&rxBuf_usb[read], sizeof(rxBuf_usb) - read);
		}

		if (read > 0) {
			GPIOC->BSRR = GPIO_PIN_8 | GPIO_PIN_9;
			UART_Write((uint8_t*)rxBuf_usb, read);					
			read = 0;
		}
		
		GPIOC->BSRR = GPIO_PIN_9 << 16;
		if (UartRxComplete) {
			GPIOC->BSRR = GPIO_PIN_8 << 16;
			VCP_write(rxBuf_422, 4);
			UartRxComplete = 0;
			UART_Read((uint8_t*)rxBuf_422, 4);
		}		
	}
*/