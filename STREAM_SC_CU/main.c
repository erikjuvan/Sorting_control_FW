#include "usbd_cdc_if.h"
#include <usbd_cdc.h>
#include <usbd_core.h>
#include <usbd_desc.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "communication.h"
#include "main.h"
#include "parse.h"
#include "uart.h"

typedef union {
    uint64_t u64;
    struct {
        uint32_t raw_data : 31;
        uint32_t ejection_window : 1;
        uint32_t undef : 31;
        uint32_t object_detected : 1;
    } u32;

    struct {
        float undef;
        float filtered_data_w_obj_det;
    } f32;
} ProtocolDataType;

USBD_HandleTypeDef       USBD_Device;
void                     SysTick_Handler(void);
void                     OTG_FS_IRQHandler(void);
void                     OTG_HS_IRQHandler(void);
extern PCD_HandleTypeDef hpcd;

int         VCP_read(void* pBuffer, int size);
int         VCP_write(const void* pBuffer, int size);
extern char g_VCPInitialized;

#define GPIO_SET_BIT(PORT, BIT) PORT->BSRR = BIT
#define GPIO_CLR_BIT(PORT, BIT) PORT->BSRR = (BIT << 16)

/*
// IR TX LEDs EVEN
#define IR_LED_EVEN_PORT GPIOA
#define IR_LED_EVEN_PIN GPIO_PIN_8
#define IR_LED_EVEN_CLK __GPIOA_CLK_ENABLE
// IR TX LEDs ODD
#define IR_LED_ODD_PORT GPIOB
#define IR_LED_ODD_PIN GPIO_PIN_6
#define IR_LED_ODD_CLK __GPIOB_CLK_ENABLE

// SYNC PIN
#define SYNC_PORT GPIOA
#define SYNC_PIN GPIO_PIN_10
#define SYNC_CLK __GPIOA_CLK_ENABLE
*/

// Za testiranje na NUCLEO boardu uporabljam port E
// IR TX LEDs EVEN
#define IR_LED_EVEN_PORT GPIOE
#define IR_LED_EVEN_PIN GPIO_PIN_11
#define IR_LED_EVEN_CLK __GPIOE_CLK_ENABLE
// IR TX LEDs ODD
#define IR_LED_ODD_PORT GPIOE
#define IR_LED_ODD_PIN GPIO_PIN_12
#define IR_LED_ODD_CLK __GPIOE_CLK_ENABLE
// SYNC PIN
#define SYNC_PORT GPIOE
#define SYNC_PIN GPIO_PIN_10
#define SYNC_CLK __GPIOE_CLK_ENABLE

// Timer
#define TIMx TIM1
#define TIMx_CLK_SOURCE_APB2 // TIM1 is under APB2
#define TIMx_CLK_ENALBE __TIM1_CLK_ENABLE
#define TIMx_UP_IRQ_Handler TIM1_UP_TIM10_IRQHandler
#define TIMx_CC_IRQ_Handler TIM1_CC_IRQHandler
#define TIMx_UP_IRQn TIM1_UP_TIM10_IRQn
#define TIMx_CC_IRQn TIM1_CC_IRQn

// GPIO pins for debugging
#define DEBUG_PORT GPIOE
#define DEBUG_PORT_CLK __GPIOE_CLK_ENABLE
#define DEBUG_PIN_1 GPIO_PIN_14
#define DEBUG_PIN_2 GPIO_PIN_15

#define DEBUG_TIM
//#define STOPWATCH

static void Filter(uint32_t* x);

Mode g_mode = CONFIG;

ADC_HandleTypeDef ADC1_Handle;
DMA_HandleTypeDef DMA2_Handle;

#define BUFFER_SIZE 2
uint32_t buffer[BUFFER_SIZE][N_CHANNELS] = {0};

#define DATA_PER_CHANNEL 100
#define SEND_BUFFER_SIZE (N_CHANNELS * DATA_PER_CHANNEL)
uint32_t          send_buffer_i                    = 0;
int               send_buffer_alt                  = 0;
ProtocolDataType  send_buffer[2][SEND_BUFFER_SIZE] = {0};
ProtocolDataType* p_send_buffer;
Header            header = {0xDEADBEEF, 0};

// Sorting parameters
// NOTE! - units are ticks NOT time units e.g. ms
// So e.g. g_delay_ticks_param = 100 -> that means 100 ticks which is 100 * T = 100 * 1/sample_freq.
// Example: if sample_freq=5 kHz then g_delay_ticks_param takes 100 * 1 / 5000 = 20 ms
uint32_t g_delay_ticks_param    = 0;
uint32_t g_duration_ticks_param = 0;
uint32_t g_blind_ticks_param    = 0;

// Filter coefficients
float g_lpf1_K = 0.f;
float g_hpf_K  = 0.f;
float g_lpf2_K = 0.f;

// Detection threshold
float g_threshold = 0.f;

// Ticker counters
int32_t g_delay_ticker[N_CHANNELS]    = {-1, -1, -1, -1, -1, -1, -1, -1}; // -1 to prevent turn on at power on
int32_t g_duration_ticker[N_CHANNELS] = {-1, -1, -1, -1, -1, -1, -1, -1};

int g_verbose_level = 0;

#define VALVE_PORT GPIOD
#define VALVE_PINS (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7)
#define VALVE_CLK_ENABLE __GPIOD_CLK_ENABLE
uint16_t g_Valve_Pins[N_CHANNELS] = {GPIO_PIN_7, GPIO_PIN_6, GPIO_PIN_5, GPIO_PIN_4, GPIO_PIN_3, GPIO_PIN_2, GPIO_PIN_1, GPIO_PIN_0};

uint8_t g_writeToPC = 0;

uint16_t g_ejection_window  = 0;
uint16_t g_detected_objects = 0;

int   g_system_trained = 0;
int   g_training       = 0;
float g_trained_coeffs[N_CHANNELS];

static const uint32_t TIM_COUNT_FREQ = 1000000; // f=1MHz, T=1us

// IR LEDs
/////////////////////////////////////
typedef enum {
    OFF  = 0,
    EVEN = 1,
    ODD  = 2,
    ALL  = 3
} ActiveLEDs;

ActiveLEDs         sequence[]   = {ALL, ALL}; // default sequence.
unsigned int       sequence_idx = 0;
const unsigned int sequence_N   = sizeof(sequence) / sizeof(sequence[0]);

int g_sync_output_enabled = 0;
/////////////////////////////////////

static void SetIRLEDs(ActiveLEDs activate_led)
{
    switch (activate_led) {
    case OFF:
        GPIO_CLR_BIT(IR_LED_EVEN_PORT, IR_LED_EVEN_PIN);
        GPIO_CLR_BIT(IR_LED_ODD_PORT, IR_LED_ODD_PIN);
        break;
    case EVEN:
        GPIO_CLR_BIT(IR_LED_ODD_PORT, IR_LED_ODD_PIN);
        GPIO_SET_BIT(IR_LED_EVEN_PORT, IR_LED_EVEN_PIN);
        break;
    case ODD:
        GPIO_CLR_BIT(IR_LED_EVEN_PORT, IR_LED_EVEN_PIN);
        GPIO_SET_BIT(IR_LED_ODD_PORT, IR_LED_ODD_PIN);
        break;
    case ALL:
    default:
        GPIO_SET_BIT(IR_LED_EVEN_PORT, IR_LED_EVEN_PIN);
        GPIO_SET_BIT(IR_LED_ODD_PORT, IR_LED_ODD_PIN);
        break;
    }
}

static void ChangeCurrentSequence(unsigned int si)
{
    if (si < sequence_N)
        sequence_idx = si;
    else
        sequence_idx = 0;

    SetIRLEDs(sequence[sequence_idx]);
}

static void NextSequence()
{
    ChangeCurrentSequence(sequence_idx + 1);
}

// IRQ
/////////////////////////////////////
void SysTick_Handler(void)
{
    HAL_IncTick();
}

void OTG_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd);
}

__attribute__((optimize("O1"))) void EXTI15_10_IRQHandler(void)
{
    /* EXTI line interrupt detected */
    if (__HAL_GPIO_EXTI_GET_IT(SYNC_PIN) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(SYNC_PIN);

        if (TIMx->CNT > ((TIMx->ARR * 2) / 10) || TIMx->CNT < ((TIMx->ARR * 8) / 10)) { // Timer
            TIMx->EGR = TIM_EGR_UG;                                                     // Reset the counter and generate update event
        }

        if (sequence_idx != 0)
            ChangeCurrentSequence(0);
    }
}

__attribute__((optimize("O1"))) void TIMx_CC_IRQ_Handler()
{
    // CC1 IRQ
    if (TIMx->SR & TIM_SR_CC1IF) {
        TIMx->SR &= ~TIM_SR_CC1IF;
        NextSequence();
    }
}

__attribute__((optimize("O1"))) void
TIMx_UP_IRQ_Handler()
{
    // Update IRQ
    if (TIMx->SR & TIM_SR_UIF) {
        TIMx->SR &= ~TIM_SR_UIF;

        // If Sync output
        if (g_sync_output_enabled) {
            if (sequence_idx == 0) // At the start of sequence
                GPIO_SET_BIT(SYNC_PORT, SYNC_PIN);
            else // reset pin on ODD
                GPIO_CLR_BIT(SYNC_PORT, SYNC_PIN);
        }
    }
}

__attribute__((optimize("O1"))) void DMA2_Stream0_IRQHandler()
{
    static uint32_t buf[N_CHANNELS] = {0};

    if (DMA2->LISR & DMA_LISR_HTIF0) {  // If half-transfer complete
        DMA2->LIFCR = DMA_LIFCR_CHTIF0; // clear half transfer complete interrupt flag
        for (int i = sequence_idx; i < N_CHANNELS; i += sequence_N)
            buf[i] = buffer[0][i];
    } else if (DMA2->LISR & DMA_LISR_TCIF0) { // If transfer complete
        DMA2->LIFCR = DMA_LIFCR_CTCIF0;       // clear half transfer complete interrupt flag
        for (int i = sequence_idx; i < N_CHANNELS; i += sequence_N)
            buf[i] = buffer[1][i];
    }

    if (sequence_idx == (sequence_N - 1)) // last sequence
        Filter(buf);

    for (int i = 0; i < N_CHANNELS; ++i) {
        if (g_delay_ticker[i] >= 0) {
            if (g_delay_ticker[i]-- == 0) {
                VALVE_PORT->BSRR     = g_Valve_Pins[i];
                g_duration_ticker[i] = g_duration_ticks_param;
            }
        }

        if (g_duration_ticker[i] >= 0) {
            if (g_duration_ticker[i]-- == 0) {
                VALVE_PORT->BSRR = g_Valve_Pins[i] << 16;
            }
        }
    }
}
/////////////////////////////////////

void SetSequence(int* seq, int num_of_elements)
{
    int size = num_of_elements > sequence_N ? sequence_N : num_of_elements;

    for (int i = 0; i < size; ++i) {
        sequence[i] = seq[i];
    }
}

char* GetSequence(char* buf, int sizeof_buf)
{
    snprintf(buf, sizeof_buf, "%d,%d", sequence[0], sequence[1]);

    return buf;
}

void SetSyncPinAsOutput()
{
    g_sync_output_enabled = 1;

    // GPIO
    GPIO_InitTypeDef GPIO_InitStructure;
    SYNC_CLK();
    GPIO_InitStructure.Pin   = SYNC_PIN;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull  = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW; // GPIO_SPEED_FREQ_HIGH
    HAL_GPIO_Init(SYNC_PORT, &GPIO_InitStructure);

    // Disable IRQ
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
}

void SetSyncPinAsInput()
{
    g_sync_output_enabled = 0;

    // GPIO
    GPIO_InitTypeDef GPIO_InitStructure;
    SYNC_CLK();
    GPIO_InitStructure.Pin   = SYNC_PIN;
    GPIO_InitStructure.Mode  = GPIO_MODE_IT_RISING;
    GPIO_InitStructure.Pull  = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW; // GPIO_SPEED_FREQ_HIGH
    HAL_GPIO_Init(SYNC_PORT, &GPIO_InitStructure);

    // Enable IRQ
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 1);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef       RCC_ClkInitStruct;
    RCC_OscInitTypeDef       RCC_OscInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /* Enable HSE Oscillator and activate PLL with HSE as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSIState       = RCC_HSI_OFF;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 8;
    RCC_OscInitStruct.PLL.PLLN       = 336;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        asm("bkpt 255");
    }

    // Activate the OverDrive to reach the 216 Mhz Frequency
    /*if(HAL_PWREx_EnableOverDrive() != HAL_OK) {
		asm("bkpt 255");
	}*/

    /* Select PLLSAI output as USB clock source */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
    PeriphClkInitStruct.Clk48ClockSelection  = RCC_CLK48SOURCE_PLL;
    //PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
    //PeriphClkInitStruct.PLLSAI.PLLSAIQ = 7;
    //PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        asm("bkpt 255");
    }

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
	clocks dividers */
    RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        asm("bkpt 255");
    }

    SystemCoreClockUpdate();
}

static void DMA_Configure()
{
    __HAL_RCC_DMA2_CLK_ENABLE();

    DMA2_Handle.Instance                 = DMA2_Stream0;
    DMA2_Handle.Init.Channel             = DMA_CHANNEL_0;
    DMA2_Handle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    DMA2_Handle.Init.PeriphInc           = DMA_PINC_DISABLE;
    DMA2_Handle.Init.MemInc              = DMA_MINC_ENABLE;
    DMA2_Handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    DMA2_Handle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    DMA2_Handle.Init.Mode                = DMA_CIRCULAR;
    DMA2_Handle.Init.Priority            = DMA_PRIORITY_HIGH;
    DMA2_Handle.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    DMA2_Handle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_HALFFULL;
    DMA2_Handle.Init.MemBurst            = DMA_MBURST_SINGLE;
    DMA2_Handle.Init.PeriphBurst         = DMA_PBURST_SINGLE;

    HAL_DMA_Init(&DMA2_Handle);

    __HAL_LINKDMA(&ADC1_Handle, DMA_Handle, DMA2_Handle);

    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

static void ADC_Configure()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();

    GPIO_InitStructure.Pin  = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

    ADC1_Handle.Instance                   = ADC1;
    ADC1_Handle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV4; // ADC_CLOCKPRESCALER_PCLK_DIV2
    ADC1_Handle.Init.Resolution            = ADC_RESOLUTION_12B;
    ADC1_Handle.Init.ScanConvMode          = ENABLE;
    ADC1_Handle.Init.ContinuousConvMode    = DISABLE; // ENABLE
    ADC1_Handle.Init.DiscontinuousConvMode = DISABLE;
    ADC1_Handle.Init.NbrOfDiscConversion   = 0;
    ADC1_Handle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
    ADC1_Handle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_TRGO;
    ADC1_Handle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    ADC1_Handle.Init.NbrOfConversion       = N_CHANNELS;
    ADC1_Handle.Init.DMAContinuousRequests = ENABLE;
    ADC1_Handle.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
    HAL_ADC_Init(&ADC1_Handle);

    ADC_ChannelConfTypeDef adcChannelConfig;

    adcChannelConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES; // ADC_SAMPLETIME_84CYCLES
    adcChannelConfig.Channel      = ADC_CHANNEL_9;
    adcChannelConfig.Rank         = 1;
    if (HAL_ADC_ConfigChannel(&ADC1_Handle, &adcChannelConfig) != HAL_OK) {
    }

    adcChannelConfig.Channel = ADC_CHANNEL_8;
    adcChannelConfig.Rank    = 2;
    if (HAL_ADC_ConfigChannel(&ADC1_Handle, &adcChannelConfig) != HAL_OK) {
    }

    adcChannelConfig.Channel = ADC_CHANNEL_15;
    adcChannelConfig.Rank    = 3;
    if (HAL_ADC_ConfigChannel(&ADC1_Handle, &adcChannelConfig) != HAL_OK) {
    }

    adcChannelConfig.Channel = ADC_CHANNEL_14;
    adcChannelConfig.Rank    = 4;
    if (HAL_ADC_ConfigChannel(&ADC1_Handle, &adcChannelConfig) != HAL_OK) {
    }

    adcChannelConfig.Channel = ADC_CHANNEL_7;
    adcChannelConfig.Rank    = 5;
    if (HAL_ADC_ConfigChannel(&ADC1_Handle, &adcChannelConfig) != HAL_OK) {
    }

    adcChannelConfig.Channel = ADC_CHANNEL_6;
    adcChannelConfig.Rank    = 6;
    if (HAL_ADC_ConfigChannel(&ADC1_Handle, &adcChannelConfig) != HAL_OK) {
    }

    adcChannelConfig.Channel = ADC_CHANNEL_5;
    adcChannelConfig.Rank    = 7;
    if (HAL_ADC_ConfigChannel(&ADC1_Handle, &adcChannelConfig) != HAL_OK) {
    }

    adcChannelConfig.Channel = ADC_CHANNEL_4;
    adcChannelConfig.Rank    = 8;
    if (HAL_ADC_ConfigChannel(&ADC1_Handle, &adcChannelConfig) != HAL_OK) {
    }
}

static void TIM_Configure()
{
    TIMx_CLK_ENALBE();

    // NOTE: Timer clocks can be tricky since they can be different from the bus frequency, so when in doubt check the datasheet.
#if defined(TIMx_CLK_SOURCE_APB1)
    uint32_t timer_freq = HAL_RCC_GetPCLK1Freq();
    if (RCC->CFGR & RCC_CFGR_PPRE1_2) // if MSB is not zero (clk divison by more than 1)
        timer_freq *= 2;
#elif defined(TIMx_CLK_SOURCE_APB2)
    uint32_t timer_freq = HAL_RCC_GetPCLK2Freq();
    if (RCC->CFGR & RCC_CFGR_PPRE2_2) // if MSB is not zero (clk divison by more than 1)
        timer_freq *= 2;
#endif

    TIMx->PSC  = (uint32_t)(timer_freq / TIM_COUNT_FREQ) - 1; // Set prescaler to count with 1/TIM_COUNT_FREQ period
    TIMx->CR2  = TIM_TRGO_UPDATE;
    TIMx->EGR  = TIM_EGR_UG; // Reset the counter and generate update event
    TIMx->SR   = 0;          // Clear interrupts
    TIMx->DIER = TIM_DIER_CC1IE | TIM_DIER_UIE;
    TIMx->CR1  = TIM_CR1_CEN;

    HAL_NVIC_SetPriority(TIMx_UP_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIMx_UP_IRQn);

    HAL_NVIC_SetPriority(TIMx_CC_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(TIMx_CC_IRQn);
}

static void GPIO_Configure()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // IR LED 1
    IR_LED_EVEN_CLK();
    GPIO_InitStructure.Pin   = IR_LED_EVEN_PIN;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull  = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW; // GPIO_SPEED_FREQ_HIGH
    HAL_GPIO_Init(IR_LED_EVEN_PORT, &GPIO_InitStructure);

    // IR LED 2
    IR_LED_ODD_CLK();
    GPIO_InitStructure.Pin = IR_LED_ODD_PIN;
    HAL_GPIO_Init(IR_LED_ODD_PORT, &GPIO_InitStructure);

    // Valve GPIO
    VALVE_CLK_ENABLE();
    GPIO_InitStructure.Pin   = VALVE_PINS;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull  = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW; // GPIO_SPEED_FREQ_HIGH
    HAL_GPIO_Init(VALVE_PORT, &GPIO_InitStructure);

#ifdef DEBUG_TIM
    DEBUG_PORT_CLK();
    GPIO_InitStructure.Pin   = DEBUG_PIN_1 | DEBUG_PIN_2;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull  = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW; // GPIO_SPEED_FREQ_HIGH
    HAL_GPIO_Init(DEBUG_PORT, &GPIO_InitStructure);
#endif
}

static void EXTI_Configure()
{
    // UART
    EXTI->IMR |= EXTI_IMR_IM0;
    HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    // Sync pin
    SetSyncPinAsInput();
}

static void USB_Init()
{
    USBD_Init(&USBD_Device, &VCP_Desc, 0);

    USBD_RegisterClass(&USBD_Device, &USBD_CDC);
    USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_STREAM_SC_CU_fops);
    USBD_Start(&USBD_Device);
}

static void USB_Deinit()
{
    USBD_Stop(&USBD_Device);
    USBD_DeInit(&USBD_Device);
}

static void Init()
{
    HAL_Init();
    SystemClock_Config();
    GPIO_Configure();
    ADC_Configure();
    DMA_Configure();
    TIM_Configure();
    EXTI_Configure();

    UART_Init();

#ifdef STOPWATCH
    EnableCC();
#endif

    // Set all coeffs to 1.0 (untrained)
    for (int i = 0; i < N_CHANNELS; ++i)
        g_trained_coeffs[i] = 1.0;

    // reset counter just to be sure it's 0 at start
    header.packet_id = 0;

    HAL_ADC_Start_DMA(&ADC1_Handle, (uint32_t*)(&buffer[0][0]), BUFFER_SIZE * N_CHANNELS);

    USB_Init();
}

void SetSampleFrequency(int freq_hz)
{
    int sample_every_N_counts = TIM_COUNT_FREQ / freq_hz;
    TIMx->ARR                 = sample_every_N_counts - 1;
    TIMx->CCR1                = TIMx->ARR / 2;
    TIMx->EGR                 = TIM_EGR_UG;
}

uint32_t GetSampleFrequency()
{
    int sample_every_N_counts = TIMx->ARR + 1;
    return TIM_COUNT_FREQ / sample_every_N_counts;
}

static void Train()
{
    uint32_t adc_channels[] = {ADC_CHANNEL_9, ADC_CHANNEL_8, ADC_CHANNEL_15, ADC_CHANNEL_14, ADC_CHANNEL_7, ADC_CHANNEL_6, ADC_CHANNEL_5, ADC_CHANNEL_4};

    HAL_ADC_Stop_DMA(&ADC1_Handle);

    ADC1_Handle.Init.ScanConvMode          = DISABLE;
    ADC1_Handle.Init.NbrOfConversion       = 1;
    ADC1_Handle.Init.DMAContinuousRequests = DISABLE;
    ADC1_Handle.Init.EOCSelection          = DISABLE;
    HAL_ADC_Init(&ADC1_Handle);

    ADC_ChannelConfTypeDef adcChannelConfig;

    adcChannelConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES; // ADC_SAMPLETIME_84CYCLES
    adcChannelConfig.Rank         = 1;

    for (int adc_idx = 0; adc_idx < N_CHANNELS; ++adc_idx) {

        adcChannelConfig.Channel = adc_channels[adc_idx];
        if (HAL_ADC_ConfigChannel(&ADC1_Handle, &adcChannelConfig) != HAL_OK) {
        }

        uint32_t  accum = 0;
        const int Size  = 1000;

        for (int i = 0; i < Size; ++i) {
            HAL_ADC_Start(&ADC1_Handle);
            if (HAL_ADC_PollForConversion(&ADC1_Handle, 500) == HAL_OK)
                accum += HAL_ADC_GetValue(&ADC1_Handle);
        }

        HAL_ADC_Stop(&ADC1_Handle);
        float avg                 = accum / Size;
        g_trained_coeffs[adc_idx] = 1000.0 / avg;
    }

    HAL_ADC_Stop(&ADC1_Handle);
    ADC_Configure();
    HAL_ADC_Start_DMA(&ADC1_Handle, (uint32_t*)(&buffer[0][0]), BUFFER_SIZE * N_CHANNELS);

    g_system_trained = 1;
}

static __attribute__((optimize("O2"))) void AddValues(uint32_t* raw_data, float* filtered_data)
{
    ProtocolDataType data[N_CHANNELS]; // doesn't need to be zero initialized

    // Encode data
    for (int i = 0; i < N_CHANNELS; ++i) {
        // Add raw data
        data[i].u32.raw_data = raw_data[i];
        // Add ejection window info
        data[i].u32.ejection_window = (g_ejection_window & (1 << i)) != 0;
        // Add filtered data
        data[i].f32.filtered_data_w_obj_det = filtered_data[i];
        // Add object detected
        data[i].u32.object_detected = (g_detected_objects & (1 << i)) != 0;

        // Add data to sending buffer
        // Organize data like so: ch1_0,ch1_1,...,ch1_DATA_PER_CHANNEL, ch2_0, ch2_1, ... chN_DATA_PER_CHANNEL.
        send_buffer[send_buffer_alt][i * DATA_PER_CHANNEL + send_buffer_i] = data[i];
    }
    send_buffer_i++;

    if (send_buffer_i >= DATA_PER_CHANNEL) {
        send_buffer_i   = 0;
        p_send_buffer   = &send_buffer[send_buffer_alt][0];
        send_buffer_alt = send_buffer_alt ? 0 : 1;
        ++header.packet_id;
        g_writeToPC = 1;
    }
}

static __attribute__((optimize("O2"))) void ObjectDetected(int ch)
{
    if ((1 << ch) & g_ejection_window)
        g_delay_ticker[ch] = g_delay_ticks_param;

    g_detected_objects |= 1 << ch;
}

static __attribute__((optimize("O2"))) void Filter(uint32_t* raw_data)
{
    static float y0[N_CHANNELS], y1[N_CHANNELS], y2[N_CHANNELS], y3[N_CHANNELS], y4[N_CHANNELS];
    static int   blind_ticker[N_CHANNELS] = {0};

    for (int i = 0; i < N_CHANNELS; i++) {
        y0[i] = (float)raw_data[i];
        // LPF
        y1[i] = g_lpf1_K * y0[i] + ((float)1.0 - g_lpf1_K) * y1[i];
        // HPF
        y2[i] = g_hpf_K * y1[i] + ((float)1.0 - g_hpf_K) * y2[i];
        y3[i] = y1[i] - y2[i];
        // Feature
        y3[i] = fabsf(y3[i]); // added benefit: avoiding negative numbers (not to clash with object detection encoding)
        // LPF
        y4[i] = g_lpf2_K * y3[i] + ((float)1.0 - g_lpf2_K) * y4[i];

        // Overriding values can be dangerous if shape of signal makes it such that signal rises at the begining instead of falls
        //if (y4[i] < 0)
        //  y4[i] = 0; // avoid negative numbers (not to clash with object detection encoding)
        // Square it to increase max/min ratio (increase dynamic resolution)
        // y4[i] = y4[i] * y4[i]; // not used at the moment

        blind_ticker[i] -= (blind_ticker[i] > 0);

        if (y4[i] > g_threshold && blind_ticker[i] <= 0) {
            blind_ticker[i] = g_blind_ticks_param;
            ObjectDetected(i);
        }
    }

    if (g_verbose_level)
        AddValues(raw_data, y4);
}

void COM_UART_RX_Complete_Callback(uint8_t* buf, int size)
{
    if (g_mode == CONFIG) { // Config mode
        Parse((char*)buf, UARTWrite);
    } else if (g_mode == SORT) { // Sorting mode
        g_ejection_window = ((uint16_t)buf[0] << 8) | buf[1];

        __disable_irq();
        uint16_t det_obj   = g_detected_objects;
        g_detected_objects = 0;
        __enable_irq();

        uint8_t txBuf[2];
        txBuf[0] = det_obj >> 8; // no masking needed
        txBuf[1] = det_obj;      // no masking needed
        UARTWrite(txBuf, sizeof(txBuf));
    }
}

int main()
{
    uint8_t rxBuf[UART_BUFFER_SIZE] = {0};
    int     usb_read                = 0;

    Init();

    while (1) {

        if (g_VCPInitialized) { // Make sure USB is initialized (calling, VCP_write can halt the system if the data structure hasn't been malloc-ed yet)
            usb_read = USBRead(rxBuf, sizeof(rxBuf));
            if (usb_read > 0) {
                Parse((char*)rxBuf, USBWrite);
                memset(rxBuf, 0, usb_read);
            }

            if (g_writeToPC) {
                g_writeToPC = 0; // reset it right away - so that we register a new write request
                VCP_write(&header, sizeof(header));
                VCP_write(p_send_buffer, SEND_BUFFER_SIZE * sizeof(ProtocolDataType));
            }
        }
    }
}
