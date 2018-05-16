#pragma once

#include "stm32f7xx_hal.h"

#define UART_BUFFER_SIZE	512

//#define NUCLEO

#ifndef	NUCLEO

#define USARTx							USART3
#define USARTx_CLK_ENABLE()				__HAL_RCC_USART3_CLK_ENABLE()

#define USARTx_FORCE_RESET()			__HAL_RCC_USART3_FORCE_RESET()
#define USARTx_RELEASE_RESET()			__HAL_RCC_USART3_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_RX_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOB_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOB_CLK_ENABLE() 
#define USARTx_DE_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOB_CLK_ENABLE() 

#define USARTx_TX_PIN					GPIO_PIN_10
#define USARTx_TX_GPIO_PORT				GPIOB
#define USARTx_TX_AF					GPIO_AF7_USART3
#define USARTx_RX_PIN					GPIO_PIN_11
#define USARTx_RX_GPIO_PORT				GPIOB
#define USARTx_RX_AF					GPIO_AF7_USART3
#define USARTx_DE_PIN					GPIO_PIN_14
#define USARTx_DE_GPIO_PORT				GPIOB
#define USARTx_DE_AF					GPIO_AF7_USART3

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART3_IRQn
#define USARTx_IRQHandler                USART3_IRQHandler

/* UART CLK SOURCE */
#define USARTx_RCC_PERIPHCLKINIT() \
	RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit; \
	RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3; \
	RCC_PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_SYSCLK; \
	HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit)

#else
#define USARTx							USART2
#define USARTx_CLK_ENABLE()				__HAL_RCC_USART2_CLK_ENABLE()
#define USARTx_RCC_PERIPHCLK			RCC_PERIPHCLK_USART2

#define USARTx_RX_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOD_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOD_CLK_ENABLE() 
#define USARTx_DE_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOD_CLK_ENABLE() 

#define USARTx_FORCE_RESET()			__HAL_RCC_USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET()			__HAL_RCC_USART2_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN					GPIO_PIN_5
#define USARTx_TX_GPIO_PORT				GPIOD
#define USARTx_TX_AF					GPIO_AF7_USART2
#define USARTx_RX_PIN					GPIO_PIN_6
#define USARTx_RX_GPIO_PORT				GPIOD
#define USARTx_RX_AF					GPIO_AF7_USART2
#define USARTx_DE_PIN					GPIO_PIN_4
#define USARTx_DE_GPIO_PORT				GPIOD
#define USARTx_DE_AF					GPIO_AF7_USART2

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART2_IRQn
#define USARTx_IRQHandler                USART2_IRQHandler

/* UART CLK SOURCE */
#define USARTx_RCC_PERIPHCLKINIT() \
	RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit; \
	RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2; \
	RCC_PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK; \
	HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit)

#endif

void UART_Init();
int  UART_Write(const uint8_t* data, int size);
void UART_Set_Address(uint8_t addr);

void UART_RX_Complete_Callback(const uint8_t* data, int size);
