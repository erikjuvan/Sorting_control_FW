#include <usbd_core.h>
#include <usbd_cdc.h>
#include "usbd_cdc_if.h"
#include <usbd_desc.h>

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "main.h"
#include "uart.h"
#include "communication.h"
#include "parse.h"

USBD_HandleTypeDef USBD_Device;
void SysTick_Handler(void);
void OTG_FS_IRQHandler(void);
void OTG_HS_IRQHandler(void);
extern PCD_HandleTypeDef hpcd;
	
int VCP_read(void *pBuffer, int size);
int VCP_write(const void *pBuffer, int size); 
extern char g_VCPInitialized;

#define GPIO_SET_BIT(PORT, BIT)		PORT->BSRR = BIT
#define GPIO_CLR_BIT(PORT, BIT)		PORT->BSRR = (BIT << 16)

#define IR_LED_PORT			GPIOE
#define IR_LED_PIN			GPIO_PIN_8
#define IR_LED_CLK			__GPIOE_CLK_ENABLE

#define TIMx				TIM1
#define TIMx_CLK_ENALBE		__TIM1_CLK_ENABLE
#define TIMx_IRQ_Handler	TIM1_IRQHandler

//#define	DEBUG_TIM
//#define	STOPWATCH

void AddValues(float* x);
void Filter(float* x);

ADC_HandleTypeDef	ADC1_Handle;
DMA_HandleTypeDef	DMA2_Handle;

#define		BUFFER_SIZE		2
uint32_t	Buffer[BUFFER_SIZE][N_CHANNELS] = { 0 };

#define		SEND_BUFFER_SIZE (N_CHANNELS * 100)
uint32_t	SendBuffer_i = 0;
float		SendBuffer[SEND_BUFFER_SIZE] = { 0 };

// GUI settable parameters
float A1 = 0.01;
float A2 = 0.03;
float A4 = 0.03;
float FTR_THRSHLD = 7.0;
uint32_t T_delay = 0;
uint32_t T_duration = 100;
uint32_t T_blind = 1000;
int skip_2nd = 0;
int skip_2nd_cntr[N_CHANNELS] = {0};

struct SystemParameters systemParameters = {
		.timer_period = 1000,
		.verbose_level = 0
	};

#define VALVE_PORT	GPIOD
#define VALVE_PINS	(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7)
#define VALVE_CLK_ENABLE	__GPIOD_CLK_ENABLE
uint16_t Valve_Pins[N_CHANNELS] = {GPIO_PIN_7, GPIO_PIN_6, GPIO_PIN_5, GPIO_PIN_4, GPIO_PIN_3, GPIO_PIN_2, GPIO_PIN_1, GPIO_PIN_0};

int32_t duration_timer[N_CHANNELS] = {0};
int32_t delay_timer[N_CHANNELS] = {0};

DisplayData display_data = RAW; 
uint8_t writeToPC = 0;

uint16_t trigger_output = 0;
uint16_t detected_objects = 0;

int training = 0;
float trained_coeffs[N_CHANNELS];

int protocol_ascii = 1;

// IRQ
/////////////////////////////////////
void SysTick_Handler(void) {
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();	
}

void OTG_FS_IRQHandler(void) {
	HAL_PCD_IRQHandler(&hpcd);
}

#ifdef DEBUG_TIM
void TIMx_IRQ_Handler() {
	TIMx->SR &= ~(TIM_SR_TIF | TIM_SR_UIF);
	GPIOA->BSRR = GPIO_PIN_8;
}
#endif

__attribute__((optimize("O0"))) void DMA2_Stream0_IRQHandler() {
	
#ifdef DEBUG_TIM
	GPIOA->BSRR = GPIO_PIN_8 << 16;
#endif

	float fBuf[N_CHANNELS];
	
	if (DMA2->LISR & DMA_LISR_HTIF0) {			// If half-transfer complete
		DMA2->LIFCR = DMA_LIFCR_CHTIF0;			// clear half transfer complete interrupt flag
		for (int i = 0; i < N_CHANNELS; ++i)
			fBuf[i] = (float)Buffer[0][i];									
	} else if (DMA2->LISR & DMA_LISR_TCIF0) {	// If transfer complete
		DMA2->LIFCR = DMA_LIFCR_CTCIF0;			// clear half transfer complete interrupt flag
		for (int i = 0; i < N_CHANNELS; ++i)
			fBuf[i] = (float)Buffer[BUFFER_SIZE/2][i];
	}
	
	if (training) {
		training--;
		const float A = 0.01;
		for (int i = 0; i < N_CHANNELS; ++i)
			trained_coeffs[i] = A * fBuf[i] + (1 - A) * trained_coeffs[i];
		
		if (training <= 0) {
			for (int i = 0; i < N_CHANNELS; ++i)
				trained_coeffs[i] = 1000.0 / trained_coeffs[i];
		}
	} else {
		if (display_data == RAW) {
			AddValues(fBuf);
		} else if (display_data == TRAINED) {
			for (int i = 0; i < N_CHANNELS; ++i) {
				fBuf[i] *= trained_coeffs[i];
			}
			AddValues(fBuf);
		} else if (display_data == FILTERED) {
			for (int i = 0; i < N_CHANNELS; ++i) {
				fBuf[i] *= trained_coeffs[i];
			}
			Filter(fBuf);
		} 
	}
	
	for (int i = 0; i < N_CHANNELS; ++i) {
		if (delay_timer[i] >= 0) {			
			if (delay_timer[i]-- == 0) {
				VALVE_PORT->BSRR = Valve_Pins[i];
				duration_timer[i] = T_duration;
			}
		}			
		
		if (duration_timer[i] >= 0) {			
			if (duration_timer[i]-- == 0) {
				VALVE_PORT->BSRR = Valve_Pins[i] << 16;
			}
		}
	}		
}
/////////////////////////////////////		

static void SystemClock_Config(void) {
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
	
	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;  
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		asm("bkpt 255");
	}
  
	// Activate the OverDrive to reach the 216 Mhz Frequency
	/*if(HAL_PWREx_EnableOverDrive() != HAL_OK) {
		asm("bkpt 255");
	}*/
  
	/* Select PLLSAI output as USB clock source */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
	PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
	//PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
	//PeriphClkInitStruct.PLLSAI.PLLSAIQ = 7; 
	//PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
	if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct)  != HAL_OK) {
		asm("bkpt 255");
	}
  
	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
	clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		asm("bkpt 255");
	}
	
	SystemCoreClockUpdate();
}

void DMA_Configure() {
	__HAL_RCC_DMA2_CLK_ENABLE();
	
	DMA2_Handle.Instance = DMA2_Stream0;
	DMA2_Handle.Init.Channel  = DMA_CHANNEL_0;
	DMA2_Handle.Init.Direction = DMA_PERIPH_TO_MEMORY;
	DMA2_Handle.Init.PeriphInc = DMA_PINC_DISABLE;
	DMA2_Handle.Init.MemInc = DMA_MINC_ENABLE;
	DMA2_Handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	DMA2_Handle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	DMA2_Handle.Init.Mode = DMA_CIRCULAR;
	DMA2_Handle.Init.Priority = DMA_PRIORITY_HIGH;
	DMA2_Handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;         
	DMA2_Handle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
	DMA2_Handle.Init.MemBurst = DMA_MBURST_SINGLE;
	DMA2_Handle.Init.PeriphBurst = DMA_PBURST_SINGLE; 
    
	HAL_DMA_Init(&DMA2_Handle);
    
	__HAL_LINKDMA(&ADC1_Handle, DMA_Handle, DMA2_Handle);
 
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);   
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}


void ADC_Configure() {
	GPIO_InitTypeDef	GPIO_InitStructure;
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_ADC1_CLK_ENABLE();
	
	GPIO_InitStructure.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.Pin = GPIO_PIN_4 | GPIO_PIN_5;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
		
	ADC1_Handle.Instance = ADC1;
	ADC1_Handle.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4; // ADC_CLOCKPRESCALER_PCLK_DIV2
	ADC1_Handle.Init.Resolution = ADC_RESOLUTION_12B;
	ADC1_Handle.Init.ScanConvMode = ENABLE;
	ADC1_Handle.Init.ContinuousConvMode = DISABLE;	// ENABLE
	ADC1_Handle.Init.DiscontinuousConvMode = DISABLE;
	ADC1_Handle.Init.NbrOfDiscConversion = 0;
	ADC1_Handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	ADC1_Handle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;
	ADC1_Handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	ADC1_Handle.Init.NbrOfConversion = N_CHANNELS;
	ADC1_Handle.Init.DMAContinuousRequests = ENABLE;
	ADC1_Handle.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	HAL_ADC_Init(&ADC1_Handle);
	
	ADC_ChannelConfTypeDef adcChannelConfig;
	
	adcChannelConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;	// ADC_SAMPLETIME_84CYCLES
	adcChannelConfig.Channel = ADC_CHANNEL_9;
	adcChannelConfig.Rank = 1;	
	if (HAL_ADC_ConfigChannel(&ADC1_Handle, &adcChannelConfig) != HAL_OK) {
	}
	
	adcChannelConfig.Channel = ADC_CHANNEL_8;
	adcChannelConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&ADC1_Handle, &adcChannelConfig) != HAL_OK) {
	}	
	
	adcChannelConfig.Channel = ADC_CHANNEL_15;
	adcChannelConfig.Rank = 3;
	if (HAL_ADC_ConfigChannel(&ADC1_Handle, &adcChannelConfig) != HAL_OK) {
	}
	
	adcChannelConfig.Channel = ADC_CHANNEL_14;
	adcChannelConfig.Rank = 4;
	if (HAL_ADC_ConfigChannel(&ADC1_Handle, &adcChannelConfig) != HAL_OK) {
	}
	
	adcChannelConfig.Channel = ADC_CHANNEL_7;
	adcChannelConfig.Rank = 5;
	if (HAL_ADC_ConfigChannel(&ADC1_Handle, &adcChannelConfig) != HAL_OK) {
	}
	
	adcChannelConfig.Channel = ADC_CHANNEL_6;
	adcChannelConfig.Rank = 6;
	if (HAL_ADC_ConfigChannel(&ADC1_Handle, &adcChannelConfig) != HAL_OK) {
	}
	
	adcChannelConfig.Channel = ADC_CHANNEL_5;
	adcChannelConfig.Rank = 7;
	if (HAL_ADC_ConfigChannel(&ADC1_Handle, &adcChannelConfig) != HAL_OK) {
	}
	
	adcChannelConfig.Channel = ADC_CHANNEL_4;
	adcChannelConfig.Rank = 8;
	if (HAL_ADC_ConfigChannel(&ADC1_Handle, &adcChannelConfig) != HAL_OK) {
	}
}

void TIM_Configure() {
	TIMx_CLK_ENALBE();
	
	TIMx->PSC	= (uint32_t)((SystemCoreClock / 2) / 1e6) - 1;
	TIMx->ARR	= 1e2 - 1;	//1e2 -1
	TIMx->CR2	= TIM_CR2_MMS_1;
	TIMx->EGR	= TIM_EGR_UG;   	// Reset the counter and generate update event
	TIMx->SR	= 0;	// Clear interrupts	
	TIMx->DIER	= 0;
	TIMx->CR1	= TIM_CR1_CEN;
	
#ifdef DEBUG_TIM
	TIMx->DIER	= TIM_DIER_TIE | TIM_DIER_UIE;
	
	HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);   
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
#endif
}

void GPIO_Configure() {
	GPIO_InitTypeDef	GPIO_InitStructure;
	
	// IR LED
	IR_LED_CLK();
	GPIO_InitStructure.Pin = IR_LED_PIN;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;	// GPIO_SPEED_FREQ_HIGH
	HAL_GPIO_Init(IR_LED_PORT, &GPIO_InitStructure);
	
	// Valve GPIO
	VALVE_CLK_ENABLE();
	GPIO_InitStructure.Pin = VALVE_PINS;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;	// GPIO_SPEED_FREQ_HIGH
	HAL_GPIO_Init(VALVE_PORT, &GPIO_InitStructure);
}

void ChangeSampleFrequency() {
	TIMx->ARR = systemParameters.timer_period;
	TIMx->EGR = TIM_EGR_UG;		
}

static void Train() {
	uint32_t adc_channels[] = {ADC_CHANNEL_9, ADC_CHANNEL_8, ADC_CHANNEL_15, ADC_CHANNEL_14, ADC_CHANNEL_7, ADC_CHANNEL_6, ADC_CHANNEL_5, ADC_CHANNEL_4};
	
	HAL_ADC_Stop_DMA(&ADC1_Handle);
	
	ADC1_Handle.Init.ScanConvMode = DISABLE;
	ADC1_Handle.Init.NbrOfConversion = 1;
	ADC1_Handle.Init.DMAContinuousRequests = DISABLE;
	ADC1_Handle.Init.EOCSelection = DISABLE;
	HAL_ADC_Init(&ADC1_Handle);
	
	ADC_ChannelConfTypeDef adcChannelConfig;
	
	adcChannelConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;	// ADC_SAMPLETIME_84CYCLES
	adcChannelConfig.Rank = 1;	
	
	for (int adc_idx = 0; adc_idx < N_CHANNELS; ++adc_idx) {
	
		adcChannelConfig.Channel = adc_channels[adc_idx];	
		if (HAL_ADC_ConfigChannel(&ADC1_Handle, &adcChannelConfig) != HAL_OK) {
		}
		
		uint32_t accum = 0;
		const int Size = 1000;
		
		for (int i = 0; i < Size; ++i) {
			HAL_ADC_Start(&ADC1_Handle);
			if (HAL_ADC_PollForConversion(&ADC1_Handle, 500) == HAL_OK)
				accum += HAL_ADC_GetValue(&ADC1_Handle);
		}
		
		HAL_ADC_Stop(&ADC1_Handle);
		float avg = accum / Size;
		trained_coeffs[adc_idx] = 1000.0 / avg;
	}
	
	HAL_ADC_Stop(&ADC1_Handle);
	ADC_Configure();
	HAL_ADC_Start_DMA(&ADC1_Handle, (uint32_t*)(&Buffer[0][0]), BUFFER_SIZE * N_CHANNELS);
}

__attribute__((optimize("O2"))) void AddValues(float* x) {	
	memcpy(&SendBuffer[SendBuffer_i], x, N_CHANNELS * sizeof(float));
	SendBuffer_i += N_CHANNELS;

	if (SendBuffer_i >= SEND_BUFFER_SIZE) {
		writeToPC = 1;
		SendBuffer_i = 0;
	}
}

__attribute__((optimize("O2"))) void ObjectDetected(int idx) {
	int set = 1;
	if (skip_2nd) {				
		if (skip_2nd_cntr[idx]++ % 2)
			set = 0;
	}
	
	if (set && ((1 << idx) & trigger_output)) {
		delay_timer[idx] = T_delay;
	}
	
	detected_objects |= 1 << idx;
}

__attribute__((optimize("O2"))) void Filter(float* x) {
	/*
	1.stage: LPF
	y[i] = a * x[i] + (1 - a) * y[i - 1]
	2.stage: HPf
	u[i] = a * x[i] + (1 - a) * u[i - 1]
	y[i] = x[i] - u[i]
	3.stage: Feature integration implemented with a LPF
	y[i] = a * abs(x[i]) + (1 - a) * y[i - 1]
	*/
	static float y0[N_CHANNELS], y1[N_CHANNELS], y2[N_CHANNELS], y3[N_CHANNELS], y4[N_CHANNELS];	
	static int blind_time[N_CHANNELS] = { 0 };					
	
	for (int i = 0; i < N_CHANNELS; i++) {
		y0[i] = (float)x[i];
		// LPF
		y1[i] = A1 * y0[i] + ((float)1.0 - A1) * y1[i];
		//HPF
		y2[i] = A2 * y1[i] + ((float)1.0 - A2) * y2[i];					
		y3[i] = y1[i] - y2[i];
		// Feature low pass filter
		y3[i] = fabsf(y3[i]);
		y4[i] = A4 * y3[i] + ((float)1.0 - A4) * y4[i];
				
		blind_time[i] -=  (blind_time[i] > 0);
		
		if (y4[i] > FTR_THRSHLD && blind_time[i] <= 0) {
			blind_time[i] = T_blind;
			ObjectDetected(i);
		}
	}
	
	AddValues(y4);
}

static void USB_Init() {
	USBD_Init(&USBD_Device, &VCP_Desc, 0);

	USBD_RegisterClass(&USBD_Device, &USBD_CDC);
	USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_LINEX_Sorting_control_fops);
	USBD_Start(&USBD_Device);
	
	// Wait for USB to Initialize
	while (USBD_Device.pClassData == 0) {
	}
}

static void USB_Deinit() {
	USBD_Stop(&USBD_Device);
	USBD_DeInit(&USBD_Device);	
}

static void Init() {
	HAL_Init();	
	SystemClock_Config();
	GPIO_Configure();
	ADC_Configure();
	DMA_Configure();
	TIM_Configure();
		
	UART_Init();
	
	GPIO_SET_BIT(IR_LED_PORT, IR_LED_PIN);
	
#ifdef STOPWATCH
	EnableCC();	  
#endif

	// Set all coeffs to 1.0 (untrained)
	for (int i = 0; i < N_CHANNELS; ++i) trained_coeffs[i] = 1.0;
}

int main() {
	uint8_t rxBuf[UART_BUFFER_SIZE] = {0};
	uint8_t txBuf[16] = {0};
	int tmp = 0, read = 0;
	
	Init();
	HAL_ADC_Start_DMA(&ADC1_Handle, (uint32_t*)(&Buffer[0][0]), BUFFER_SIZE * N_CHANNELS);

	USB_Init();
	Communication_Set_USB();
	protocol_ascii = 1;
	
	while (1) {
				
		read = Read(rxBuf, sizeof(rxBuf), protocol_ascii);
		
		if (read > 0 && protocol_ascii ) {	// ASCII
			Parse((char*)rxBuf);
			memset(rxBuf, 0, read);
		}
		
		if (read != 0 && !protocol_ascii) { // Binary
			if (read == -1) { // Escape from Bin mode
				protocol_ascii = 1;
			} else if (read > 0) {
				trigger_output = ((uint16_t)rxBuf[0] << 8) | rxBuf[1];
		
				__disable_irq();
				uint16_t det_obj = detected_objects;
				detected_objects = 0;
				__enable_irq();
		
				txBuf[0] = (det_obj >> 8) & 0xFF;
				txBuf[1] = det_obj & 0xFF;
				if (!Communication_Get_USB())	// temporary hack whilst using USB for debuging 
					Write(txBuf, 2, 0);
			}													
		}	
		
		if (writeToPC && Communication_Get_USB() && systemParameters.verbose_level > 0) {
			VCP_write(SendBuffer, SEND_BUFFER_SIZE * sizeof(SendBuffer[0]));
			writeToPC = 0;
		}
	}
}

	