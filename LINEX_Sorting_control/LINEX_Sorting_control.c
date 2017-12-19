#include <usbd_core.h>
#include <usbd_cdc.h>
#include "usbd_cdc_if.h"
#include <usbd_desc.h>

USBD_HandleTypeDef USBD_Device;
void SysTick_Handler(void);
void OTG_FS_IRQHandler(void);
void OTG_HS_IRQHandler(void);
extern PCD_HandleTypeDef hpcd;
	
int VCP_read(void *pBuffer, int size);
int VCP_write(const void *pBuffer, int size);
extern char g_VCPInitialized;

#include <stdlib.h>
#include <string.h>
#include <math.h>

#define GPIO_SET_BIT(PORT, BIT)		PORT->BSRR = BIT
#define GPIO_CLR_BIT(PORT, BIT)		PORT->BSRR = (BIT << 16)

#define DEBUG_PIN	GPIO_PIN_11
#define DEBUG_PORT	GPIOC
#define DEBUG_CLK	__GPIOC_CLK_ENABLE

#define DEBUG_PIN1	GPIO_PIN_7
#define DEBUG_PORT1	GPIOC
#define DEBUG_CLK1	__GPIOC_CLK_ENABLE

#define DEBUG_PIN2	GPIO_PIN_4
#define DEBUG_PORT2	GPIOD
#define DEBUG_CLK2	__GPIOD_CLK_ENABLE

#define DEBUG_TOGGLE()		DEBUG_PORT->ODR ^= DEBUG_PIN;
#define DEBUG1_TOGGLE()		DEBUG_PORT1->ODR ^= DEBUG_PIN1;
#define DEBUG2_TOGGLE()		DEBUG_PORT2->ODR ^= DEBUG_PIN2;

TIM_HandleTypeDef	TIM2_Handle, TIM3_Handle;
ADC_HandleTypeDef	g_AdcHandle;
DMA_HandleTypeDef	g_DmaHandle;

#define PC 0
#define STOPWATCH

#if PC == 1
#define		ADC_BUFFER_SIZE		64
#else
#define		ADC_BUFFER_SIZE		2
#endif

#define		N_CHANNELS		3
uint16_t	g_ADCBuffer[ADC_BUFFER_SIZE][N_CHANNELS] = { 0 };
uint16_t	startFrame = 0xFFFF;

enum { IDLE = 0, HALF_CPLT, CPLT } adcState = IDLE;

struct {
	int timer_period;	// us
} g_parameters = {.timer_period = 1000};

uint32_t capsule_track_cnt[N_CHANNELS] = { 0 };
uint32_t capsule_cnt = 0;

uint8_t pinToggleTimer = 0;

// IRQ
/////////////////////////////////////
void SysTick_Handler(void) {
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
		
	if (pinToggleTimer > 0) {
		pinToggleTimer--;
	} else {
		GPIOE->BSRR = GPIO_PIN_7 << 16;
	}
}

void OTG_FS_IRQHandler(void)
{
	HAL_PCD_IRQHandler(&hpcd);
}
	 
void DMA2_Stream0_IRQHandler() {
	HAL_DMA_IRQHandler(&g_DmaHandle);
}
 
void ADC_IRQHandler() {
	HAL_ADC_IRQHandler(&g_AdcHandle);
}
	
void TIM2_IRQHandler() {
	HAL_TIM_IRQHandler(&TIM2_Handle);
}
	
void TIM3_IRQHandler() {
	HAL_TIM_IRQHandler(&TIM3_Handle);
}
/////////////////////////////////////		

// IRQ Callbacks
/////////////////////////////////////
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {			// low speed (page)					
	}
	else if (htim->Instance == TIM3) {	// high speed (line)
		HAL_ADC_Start(&g_AdcHandle);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	adcState = CPLT;
}
	
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
	adcState = HALF_CPLT;
}
/////////////////////////////////////

static void SystemClock_Config(void) {
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	__PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);

	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

void DMA_Configure() {
	__HAL_RCC_DMA2_CLK_ENABLE();
	g_DmaHandle.Instance = DMA2_Stream0;
  
	g_DmaHandle.Init.Channel  = DMA_CHANNEL_0;
	g_DmaHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
	g_DmaHandle.Init.PeriphInc = DMA_PINC_DISABLE;
	g_DmaHandle.Init.MemInc = DMA_MINC_ENABLE;
	g_DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	g_DmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	g_DmaHandle.Init.Mode = DMA_CIRCULAR;
	g_DmaHandle.Init.Priority = DMA_PRIORITY_HIGH;
	g_DmaHandle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;         
	g_DmaHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
	g_DmaHandle.Init.MemBurst = DMA_MBURST_SINGLE;
	g_DmaHandle.Init.PeriphBurst = DMA_PBURST_SINGLE; 
    
	HAL_DMA_Init(&g_DmaHandle);
    
	__HAL_LINKDMA(&g_AdcHandle, DMA_Handle, g_DmaHandle);
 
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);   
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}


void ADC_Configure() {
	GPIO_InitTypeDef	GPIO_InitStructure;
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_ADC1_CLK_ENABLE();
	
	GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	HAL_NVIC_SetPriority(ADC_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(ADC_IRQn);
	
	g_AdcHandle.Instance = ADC1;
	g_AdcHandle.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4; // ADC_CLOCKPRESCALER_PCLK_DIV2
	g_AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;
	g_AdcHandle.Init.ScanConvMode = ENABLE;
	g_AdcHandle.Init.ContinuousConvMode = DISABLE;	// ENABLE
	g_AdcHandle.Init.DiscontinuousConvMode = DISABLE;
	g_AdcHandle.Init.NbrOfDiscConversion = 0;
	g_AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;	// ADC_EXTERNALTRIGCONVEDGE_RISING
	g_AdcHandle.Init.ExternalTrigConv = ADC_SOFTWARE_START;	// ADC_EXTERNALTRIGCONV_T3_CC1
	g_AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	g_AdcHandle.Init.NbrOfConversion = 3;
	g_AdcHandle.Init.DMAContinuousRequests = ENABLE;	// ENABLE
	g_AdcHandle.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	HAL_ADC_Init(&g_AdcHandle);
	
	ADC_ChannelConfTypeDef adcChannelConfig;
	
	adcChannelConfig.Channel = ADC_CHANNEL_0;
	adcChannelConfig.Rank = 1;
	adcChannelConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;	// ADC_SAMPLETIME_84CYCLES
	if (HAL_ADC_ConfigChannel(&g_AdcHandle, &adcChannelConfig) != HAL_OK) {
	}

	// Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
	adcChannelConfig.Channel = ADC_CHANNEL_1;
	adcChannelConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&g_AdcHandle, &adcChannelConfig) != HAL_OK) {
	}

	// Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
	adcChannelConfig.Channel = ADC_CHANNEL_2;
	adcChannelConfig.Rank = 3;
	if (HAL_ADC_ConfigChannel(&g_AdcHandle, &adcChannelConfig) != HAL_OK) {
	}
}

void TIM2_Configure() {
	__TIM2_CLK_ENABLE();
	
	TIM2_Handle.Instance = TIM2;
	TIM2_Handle.Init.Prescaler = (uint32_t)((SystemCoreClock / 2) / 1e5) - 1;	// 100 kHz frequency
	TIM2_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;	
	TIM2_Handle.Init.Period = 1e3 - 1;	// count to 1e3 gives 10 ms interrupt
	TIM2_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TIM2_Handle.Init.RepetitionCounter = 0;
	
	HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
	
	HAL_TIM_Base_Init(&TIM2_Handle);
	HAL_TIM_Base_Start_IT(&TIM2_Handle);
}

void TIM3_Configure() {
	TIM_OC_InitTypeDef TIMConfig;
	__TIM3_CLK_ENABLE();
	
	TIM3_Handle.Instance = TIM3;
	TIM3_Handle.Init.Prescaler = (uint32_t)((SystemCoreClock / 2) / 1e6) - 1;
	TIM3_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	TIM3_Handle.Init.Period = 1e2 - 1;
	TIM3_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TIM3_Handle.Init.RepetitionCounter = 0;		
	
	HAL_NVIC_SetPriority(TIM3_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
		
	HAL_TIM_Base_Init(&TIM3_Handle);
	HAL_TIM_Base_Start_IT(&TIM3_Handle);
}

void GPIO_Configure() {
	GPIO_InitTypeDef	GPIO_InitStructure;
	
	DEBUG_CLK();
	DEBUG_CLK1();
	
	GPIO_InitStructure.Pin = DEBUG_PIN;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;	// GPIO_SPEED_FREQ_HIGH
	HAL_GPIO_Init(DEBUG_PORT, &GPIO_InitStructure);	
	
	GPIO_InitStructure.Pin = DEBUG_PIN1;
	HAL_GPIO_Init(DEBUG_PORT1, &GPIO_InitStructure);	
	
	// Product present GPIO: PE7 (O0)
	__GPIOE_CLK_ENABLE();
	GPIO_InitStructure.Pin = GPIO_PIN_7;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	// Prozenje za ADC-je za timing test 
	/*
	__GPIOA_CLK_ENABLE();
	GPIO_InitStructure.Pin = GPIO_PIN_3;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);	
	GPIO_InitStructure.Pin = GPIO_PIN_3;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);	
	*/
}

void OutputMCO() {
	__GPIOA_CLK_ENABLE();
	__GPIOC_CLK_ENABLE();
	
	GPIO_InitTypeDef	GPIO_InitStructure;
	
	GPIO_InitStructure.Pin = GPIO_PIN_8;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_InitStructure.Alternate = 0;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	GPIO_InitStructure.Pin = GPIO_PIN_9;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	__HAL_RCC_MCO1_CONFIG(RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_5);
	__HAL_RCC_MCO2_CONFIG(RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_5);
}

void WaitForGo() {
	uint8_t ready = 0, buf[3] = { 0, 0, 0 };
	while (!ready) {
		VCP_read(buf, 3);
		if (buf[0] == 'g' && buf[1] == 'o') {
			ready = 1;
		}
	}
}

void ChangeSampleFrequency() {
	TIM3_Handle.Init.Period = g_parameters.timer_period;
	TIM3_Handle.Instance->CNT = 0;
	HAL_TIM_Base_Init(&TIM3_Handle);
}

void ResetValues() {
	capsule_cnt = 0;
	
	for (int i = 0; i < N_CHANNELS; i++) {
		capsule_track_cnt[i] = 0;
	}
}

int ParseCMD(uint8_t *buf, int len) {
	uint8_t *pt = NULL;
	
	pt = (uint8_t *)strtok((char*)buf, ",");
	
	// "CSETF,1000" - 1000 is in hertz
	if (strncmp((char *)pt, "CSETF", strlen((char*)pt)) == 0) {
		pt = (uint8_t *)strtok(NULL, ",");
		int val = atoi((char*)pt);		
		if (val > 0) {
			g_parameters.timer_period = 1e6 / val;	// convert val which are hertz to period which is in us 
			ChangeSampleFrequency();
			return 0;	
		}
		else {
			return 1;
		}
	} else if (strncmp((char *)pt, "CRST", strlen((char*)pt)) == 0) {
		ResetValues();
	}
	
	return 1;
}

////////////////////////////
// Counting cycles using DWT
////////////////////////////
/* Registers
DWT_CYCCNT   = (int *)0xE0001004; DWT->CYCCNT
DWT_CONTROL  = (int *)0xE0001000; DWT->CTRL
SCB_DEMCR    = (int *)0xE000EDFC; CoreDebug->DEMCR
*/

/* Example C with ASM
EnableCC();
while (1) {
	__asm__("ldr r0, =0");
	__asm__("ldr r4, =0xE0001004");
	__asm__("str r0, [r4]");	// DWT->CYCCNT = 0;
	// Action to time
	__asm__("ldr r0, [r4]");	// r0 = DWT->CYCCNT
}

Only C:
EnableCC();
ResetCC();
// Action to time
int cycles = GetCC();
double seconds_taken = cycles * (1.0 / (float)SystemCoreClock);
__asm__("nop");
*/

void EnableCC() {
	CoreDebug->DEMCR |= 0x01000000;	// Enable DWT and ITM features
	DWT->CYCCNT = 0;	// Set Cycle Count register to zero	
	DWT->CTRL |= 1;	// Enable CYCCNT
}

void DisableCC() {
	DWT->CTRL &= ~1;	// Disable CYCCNT
}

#define ResetCC() DWT->CYCCNT = 0	// Set Cycle Count register to zero
#define GetCC() DWT->CYCCNT
/////////////////////////////////

////////////////////////////////
// Counting cycles using SysTick
////////////////////////////////
/* Registers
int *STCSR = (int *)0xE000E010;	SysTick->CTRL
int *STRVR = (int *)0xE000E014;	SysTick->LOAD
int *STCVR = (int *)0xE000E018;	SysTick->VAL
*/

/* Example (no initalization needed, because HAL_Init() already initializes SysTick
__asm__("ldr r4, =0xE000E018");
__asm__("ldr r0, [r4]");	// r0 = SysTick->VAL;
// function to time
__asm__("ldr r1, [r4]");	// r1 = SysTick->VAL;
__asm__("sub r0, r0, r1");	// r0 = r0 - r1 -> timer counts down
*/

// Usually not neede because HAL_Init() already initializes SysTick
void StartST() {
	SysTick->LOAD = SysTick_LOAD_RELOAD_Msk;
	SysTick->VAL = 0;
	SysTick->CTRL = 5;
}

void StopST() {
	SysTick->CTRL = 0;
}

#define GetSTCVR() SysTick->VAL // The number of core clock cycles taken by the operation is given by: (STCVR1 - STCVR2 - 2)
/////////////////////////////////

#define ARR_I_SIZE	N_CHANNELS * 1000
int arr_i = 0;
float arr[ARR_I_SIZE] = { 0 };

void AddValue(float f[N_CHANNELS], int n_elements) {
	static int state = CPLT;
	
	for (int i = 0; i < n_elements; i++) {
		arr[arr_i++] = f[i];
		
		if (arr_i >= ARR_I_SIZE / 2 && state == CPLT) {
			state = HALF_CPLT;
			VCP_write(arr, sizeof(arr)/2);
		}
		
		if (arr_i >= ARR_I_SIZE && state == HALF_CPLT) {
			state = CPLT;
			VCP_write(&arr[ARR_I_SIZE / 2], sizeof(arr) / 2);
			arr_i = 0;
		}
	}	
}

__attribute__((optimize("O2"))) void Filter(uint16_t* x) {
	/*
	1.stage: LPF
	y[i] = a * x[i] + (1 - a) * y[i - 1]
	2.stage: HPf
	u[i] = a * x[i] + (1 - a) * u[i - 1]
	y[i] = x[i] - u[i]
	3.stage: Feature integration implemented with a LPF
	y[i] = a * abs(x[i]) + (1 - a) * y[i - 1]
	*/
	const int BLIND_TIME = 1000;
	static float y0[N_CHANNELS], y1[N_CHANNELS], y2[N_CHANNELS], y3[N_CHANNELS], y4[N_CHANNELS];
	const float A1 = 0.01;
	const float A2 = 0.03;
	const float A4 = 0.03;
	const float FTR_THRSHLD = 7.0;
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
		
		if (blind_time[i] > 0) {
			blind_time[i]--;
		}
		
		if (y4[i] > FTR_THRSHLD && blind_time[i] <= 0) {
			capsule_track_cnt[i]++;
			capsule_cnt++;
			blind_time[i] = BLIND_TIME;
			//VCP_write(&capsule_cnt, sizeof(capsule_cnt));
			GPIOE->BSRR = GPIO_PIN_7;
			pinToggleTimer = 9;
		}
	}
	
	AddValue(y4, N_CHANNELS);

	//ResetCC();
	//foo();
	//volatile int cycles = GetCC();		
	//AddValue(cycles);	
	//VCP_write(y4, sizeof(y4));	
}

static void Init() {
	HAL_Init();	
	SystemClock_Config();

	USBD_Init(&USBD_Device, &VCP_Desc, 0);

	USBD_RegisterClass(&USBD_Device, &USBD_CDC);
	USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_LINEX_Sorting_control_fops);
	USBD_Start(&USBD_Device);
	
	// Wait for USB to Initialize
	while (USBD_Device.pClassData == 0) {
	}			
	
	GPIO_Configure();
	ADC_Configure();
	DMA_Configure();
	//TIM2_Configure();	
	TIM3_Configure();	
	
#ifdef STOPWATCH
	EnableCC();	  
#endif

}

int main() {				
	Init();
	//WaitForGo();
	
	HAL_ADC_Start_DMA(&g_AdcHandle, (uint32_t*)(&g_ADCBuffer[0][0]), ADC_BUFFER_SIZE * N_CHANNELS);
	
	uint8_t rxBuf[20] = { 0 };
	while (1) {
		int read = VCP_read(rxBuf, sizeof(rxBuf));	
		
		if (read > 0) {					
			ParseCMD(rxBuf, read);
			memset(rxBuf, 0, sizeof(rxBuf));
		}
		
		if (adcState == HALF_CPLT) {
			adcState = IDLE;
#if PC == 1
			VCP_write(&g_ADCBuffer[0][0], (ADC_BUFFER_SIZE * N_CHANNELS));
#else
			Filter(&g_ADCBuffer[0][0]);
#endif
		} else if (adcState == CPLT) {
			adcState = IDLE;
#if PC == 1
			VCP_write(&g_ADCBuffer[ADC_BUFFER_SIZE / 2][0], (ADC_BUFFER_SIZE * N_CHANNELS));
#else
			Filter(&g_ADCBuffer[ADC_BUFFER_SIZE / 2][0]);
#endif
		}			
	}
}