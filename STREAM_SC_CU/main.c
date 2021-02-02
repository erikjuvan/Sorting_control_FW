/// @file main.c
/// <summary>
/// Main file.
/// </summary>
///
/// Supervision: /
///
/// Company: Sensum d.o.o.
///
/// @authors Erik Juvan
///
/// @version /
/////-----------------------------------------------------------
// Company: Sensum d.o.o.

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

#define GPIO_SET_BIT(PORT, BIT) PORT->BSRR = BIT
#define GPIO_CLR_BIT(PORT, BIT) PORT->BSRR = (BIT << 16)

//#define DEBUG_MODE
//#define STOPWATCH

// GPIO pins for debugging
#define DEBUG_PORT GPIOE
#define DEBUG_PORT_CLK __GPIOE_CLK_ENABLE
#define DEBUG_PIN_0 GPIO_PIN_10
#define DEBUG_PIN_1 GPIO_PIN_11
#define DEBUG_PIN_2 GPIO_PIN_12
#define DEBUG_PIN_3 GPIO_PIN_13
#define DEBUG_PIN_4 GPIO_PIN_14
#define DEBUG_PIN_5 GPIO_PIN_15
#define DEBUG_ALL_PINS (DEBUG_PIN_0 | DEBUG_PIN_1 | DEBUG_PIN_2 | DEBUG_PIN_3 | DEBUG_PIN_4 | DEBUG_PIN_5)

// IR TX LEDs CH1
#define IR_LED_CH1_PORT GPIOE
#define IR_LED_CH1_PIN GPIO_PIN_8
#define IR_LED_CH1_CLK __GPIOE_CLK_ENABLE

// IR TX LEDs CH2
#define IR_LED_CH2_PORT GPIOE
#define IR_LED_CH2_PIN GPIO_PIN_9
#define IR_LED_CH2_CLK __GPIOE_CLK_ENABLE

// SYNC PIN
#define SYNC_PORT GPIOC
#define SYNC_PIN GPIO_PIN_11
#define SYNC_CLK __GPIOC_CLK_ENABLE

// IO PIN (CURRENTLY NOT IN USE)
#define IO_PORT GPIOC
#define IO_PIN GPIO_PIN_12
#define IO_CLK __GPIOC_CLK_ENABLE

// Timer
#define TIMx TIM1
#define TIMx_CLK_SOURCE_APB2 // TIM1 is under APB2
#define TIMx_CLK_ENALBE __TIM1_CLK_ENABLE
#define TIMx_UP_IRQ_Handler TIM1_UP_TIM10_IRQHandler
#define TIMx_CC_IRQ_Handler TIM1_CC_IRQHandler
#define TIMx_UP_IRQn TIM1_UP_TIM10_IRQn
#define TIMx_CC_IRQn TIM1_CC_IRQn

// Valves
#define VALVE_PORT GPIOD
#define VALVE_PINS (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7)
#define VALVE_CLK_ENABLE __GPIOD_CLK_ENABLE

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

typedef enum {
    OFF = 0,
    CH1 = 1,
    CH2 = 2,
    ALL = 3
} ActiveLEDs;

extern PCD_HandleTypeDef hpcd;

extern char g_VCPInitialized;

Mode g_mode = CONFIG;

USBD_HandleTypeDef USBD_Device;

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

uint16_t g_Valve_Pins[N_CHANNELS] = {GPIO_PIN_7, GPIO_PIN_6, GPIO_PIN_5, GPIO_PIN_4, GPIO_PIN_3, GPIO_PIN_2, GPIO_PIN_1, GPIO_PIN_0};

uint8_t g_writeToPC = 0;

uint16_t g_ejection_window  = 0;
uint16_t g_detected_objects = 0;

uint8_t g_training = 0; // flag that signals the system is in training mode
float   g_trained_coeffs[N_CHANNELS];

static void Filter(uint32_t* x);
static void SetIRLEDs(ActiveLEDs activate_led);
static void GotoSequenceIndex(unsigned int si);
static void NextInSequence();
static void ADC_Configure();
static void TrainCoefficients(uint32_t* buf);

static ADC_HandleTypeDef ADC1_Handle;
static DMA_HandleTypeDef DMA2_Handle;

static uint32_t buffer[2][N_CHANNELS] = {0}; // size 2 to allow for alternating read/writes, to make sure we aren't reading while the DMA is overwritting

#define DATA_PER_CHANNEL 100
#define SEND_BUFFER_SIZE (N_CHANNELS * DATA_PER_CHANNEL)
static uint32_t          send_buffer_i                    = 0;
static int               send_buffer_alt                  = 0;
static ProtocolDataType  send_buffer[2][SEND_BUFFER_SIZE] = {0}; // size 2 to allow for alternating read/writes, to make sure we aren't reading while the DMA is overwritting
static ProtocolDataType* p_send_buffer;
static Header            header = {0xDEADBEEF, 0};

static const uint32_t TIM_COUNT_FREQ = 1000000; // f=1MHz, T=1us

// IR LEDs
/////////////////////////////////////
static ActiveLEDs   sequence[2]  = {OFF}; // no valid sequence by default (size is 2 since current support is for 2 channels only, so more doesnt make sense)
static unsigned int sequence_idx = 0;
unsigned int        sequence_N   = 0;

static int sync_output_enabled = 0;
/////////////////////////////////////

//---------------------------------------------------------------------
/// <summary> System tick interrupt handler. </summary>
//---------------------------------------------------------------------
void SysTick_Handler(void)
{
    HAL_IncTick();
}

//---------------------------------------------------------------------
/// <summary> Full speed USB interrupt handler. </summary>
//---------------------------------------------------------------------
void OTG_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd);
}

//---------------------------------------------------------------------
/// <summary> External interrupt interrupt handler. </summary>
//---------------------------------------------------------------------
__attribute__((optimize("O1"))) void EXTI15_10_IRQHandler(void)
{
    /* EXTI line interrupt detected */
    if (__HAL_GPIO_EXTI_GET_IT(SYNC_PIN) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(SYNC_PIN);

        TIMx->EGR = TIM_EGR_UG; // Reset the counter and generate update event

        if (sequence_idx != 0)
            GotoSequenceIndex(0);
    }
}

//---------------------------------------------------------------------
/// <summary> Timer Capture/Compare interrupt handler. </summary>
//---------------------------------------------------------------------
__attribute__((optimize("O1"))) void TIMx_CC_IRQ_Handler()
{
    // CC1 IRQ
    if (TIMx->SR & TIM_SR_CC1IF) {
        TIMx->SR &= ~TIM_SR_CC1IF;
        NextInSequence();
    }
}

//---------------------------------------------------------------------
/// <summary> Timer Update interrupt handler. </summary>
//---------------------------------------------------------------------
__attribute__((optimize("O1"))) void TIMx_UP_IRQ_Handler()
{
    // Update IRQ
    if (TIMx->SR & TIM_SR_UIF) {
        TIMx->SR &= ~TIM_SR_UIF;

        // If Sync output
        if (sync_output_enabled) {
            if (sequence_idx == 0) // At the start of sequence
                GPIO_SET_BIT(SYNC_PORT, SYNC_PIN);
            else // reset pin on ODD
                GPIO_CLR_BIT(SYNC_PORT, SYNC_PIN);
        }
    }
}

//---------------------------------------------------------------------
/// <summary> DMA interrupt handler. </summary>
//---------------------------------------------------------------------
__attribute__((optimize("O1"))) void DMA2_Stream0_IRQHandler()
{
    static uint32_t buf[N_CHANNELS] = {0};
    uint32_t*       pBuffer;

    if (DMA2->LISR & DMA_LISR_HTIF0) {  // If half-transfer complete
        DMA2->LIFCR = DMA_LIFCR_CHTIF0; // clear half transfer complete interrupt flag
        pBuffer     = buffer[0];
    } else if (DMA2->LISR & DMA_LISR_TCIF0) { // If transfer complete
        DMA2->LIFCR = DMA_LIFCR_CTCIF0;       // clear half transfer complete interrupt flag
        pBuffer     = buffer[1];
    }

    if (sequence[sequence_idx] == CH1) {
        for (int i = 0; i < N_CHANNELS; i += sequence_N)
            buf[i] = pBuffer[i];
    } else if (sequence[sequence_idx] == CH2) {
        for (int i = 1; i < N_CHANNELS; i += sequence_N)
            buf[i] = pBuffer[i];
    } else { // ALL or OFF
        for (int i = 0; i < N_CHANNELS; ++i)
            buf[i] = pBuffer[i];
    }

    if (sequence_idx == (sequence_N - 1)) { // last sequence

        if (g_training)
            TrainCoefficients(buf);

        for (int i = 0; i < N_CHANNELS; ++i)
            buf[i] *= g_trained_coeffs[i];

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
}

//---------------------------------------------------------------------
/// <summary> Set IR LED sequence. </summary>
///
/// <param name="seq"> Sequence (array of ints). </param>
/// <param name="num_of_elements"> Number of steps in the sequence. </param>
//---------------------------------------------------------------------
void SetSequence(int* seq, int num_of_elements)
{
    const int max_sequence_size = sizeof(sequence) / sizeof(sequence[0]);

    if (num_of_elements > max_sequence_size) // too many steps in the sequence
        return;

    sequence_N = num_of_elements;

    for (int i = 0; i < num_of_elements; ++i) {
        sequence[i] = seq[i];
    }

    sequence_idx = 0;
}

//---------------------------------------------------------------------
/// <summary> Get sequence as string. </summary>
///
/// <param name="buf"> Pointer to buffer to write sequence to. </param>
/// <param name="sizeof_buf"> Size of buffer. </param>
///
/// <returns> Sequence converted to text. </returns>
//---------------------------------------------------------------------
char* GetSequence(char* buf, int sizeof_buf)
{
    if (sequence_N <= 0)
        return NULL;

    buf[0] = 0;

    for (int i = 0; i < sequence_N; ++i)
        snprintf(&buf[strlen(buf)], sizeof_buf - strlen(buf), "%d,", sequence[i]);

    buf[strlen(buf) - 1] = 0; // remove dangling comma

    return buf;
}

//---------------------------------------------------------------------
/// <summary> Set sync pin as output ('master' mode). </summary>
//---------------------------------------------------------------------
void SetSyncPinAsOutput()
{
    sync_output_enabled = 1;

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

//---------------------------------------------------------------------
/// <summary> Set sync pin as input ('slave' mode). </summary>
//---------------------------------------------------------------------
void SetSyncPinAsInput()
{
    sync_output_enabled = 0;

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

//---------------------------------------------------------------------
/// <summary> Set sample frequency. </summary>
///
/// <param name="freq_hz"> Sampling frequency in Hz </param>
//---------------------------------------------------------------------
void SetSampleFrequency(int freq_hz)
{
    int sample_every_N_counts = TIM_COUNT_FREQ / freq_hz;
    TIMx->ARR                 = sample_every_N_counts - 1;
    TIMx->CCR1                = TIMx->ARR / 2;
    TIMx->EGR                 = TIM_EGR_UG;
}

//---------------------------------------------------------------------
/// <summary> Get sample frequency. </summary>
///
/// <returns> Sample frequency in Hz. </returns>
//---------------------------------------------------------------------
uint32_t GetSampleFrequency()
{
    int sample_every_N_counts = TIMx->ARR + 1;
    return TIM_COUNT_FREQ / sample_every_N_counts;
}

//---------------------------------------------------------------------
/// <summary> Reset packet header ID to 0. </summary>
//---------------------------------------------------------------------
void ResetHeaderID()
{
    header.packet_id = 0;
}

//---------------------------------------------------------------------
/// <summary> Normalize channels. </summary>
//---------------------------------------------------------------------
void Train()
{
    g_training = 1;
}

//---------------------------------------------------------------------
/// <summary> Return channel values back to raw (unnormalize). </summary>
//---------------------------------------------------------------------
void Untrain()
{
    g_training = 0;

    for (int i = 0; i < N_CHANNELS; ++i)
        g_trained_coeffs[i] = 1.0f;
}

//---------------------------------------------------------------------
/// <summary> Configure system clock. </summary>
//---------------------------------------------------------------------
static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef       RCC_ClkInitStruct;
    RCC_OscInitTypeDef       RCC_OscInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    // Enable HSE Oscillator and activate PLL with HSE as source
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
    //if(HAL_PWREx_EnableOverDrive() != HAL_OK) {
    //	asm("bkpt 255");
    //}

    // Select PLLSAI output as USB clock source
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
    PeriphClkInitStruct.Clk48ClockSelection  = RCC_CLK48SOURCE_PLL;
    //PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
    //PeriphClkInitStruct.PLLSAI.PLLSAIQ = 7;
    //PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        asm("bkpt 255");
    }

    // Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers
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

//---------------------------------------------------------------------
/// <summary> DMA configure. </summary>
//---------------------------------------------------------------------
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

//---------------------------------------------------------------------
/// <summary> ADC configure. </summary>
//---------------------------------------------------------------------
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
    ADC1_Handle.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV2;
    ADC1_Handle.Init.Resolution            = ADC_RESOLUTION_12B;
    ADC1_Handle.Init.ScanConvMode          = ENABLE;
    ADC1_Handle.Init.ContinuousConvMode    = DISABLE;
    ADC1_Handle.Init.DiscontinuousConvMode = DISABLE;
    ADC1_Handle.Init.NbrOfDiscConversion   = 0;
    ADC1_Handle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING; // TODO: why RISINGFALLING insted of just RISING
    ADC1_Handle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_TRGO;
    ADC1_Handle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    ADC1_Handle.Init.NbrOfConversion       = N_CHANNELS;
    ADC1_Handle.Init.DMAContinuousRequests = ENABLE;
    ADC1_Handle.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
    HAL_ADC_Init(&ADC1_Handle);

    ADC_ChannelConfTypeDef adcChannelConfig;

    adcChannelConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
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

//---------------------------------------------------------------------
/// <summary> Timer configure. </summary>
//---------------------------------------------------------------------
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

//---------------------------------------------------------------------
/// <summary> GPIO Configure. </summary>
//---------------------------------------------------------------------
static void GPIO_Configure()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // IR LED 1
    IR_LED_CH2_CLK();
    GPIO_InitStructure.Pin   = IR_LED_CH2_PIN;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull  = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW; // GPIO_SPEED_FREQ_HIGH
    HAL_GPIO_Init(IR_LED_CH2_PORT, &GPIO_InitStructure);

    // IR LED 2
    IR_LED_CH1_CLK();
    GPIO_InitStructure.Pin = IR_LED_CH1_PIN;
    HAL_GPIO_Init(IR_LED_CH1_PORT, &GPIO_InitStructure);

    // Valve GPIO
    VALVE_CLK_ENABLE();
    GPIO_InitStructure.Pin   = VALVE_PINS;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull  = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW; // GPIO_SPEED_FREQ_HIGH
    HAL_GPIO_Init(VALVE_PORT, &GPIO_InitStructure);

#ifdef DEBUG_MODE
    DEBUG_PORT_CLK();
    GPIO_InitStructure.Pin   = DEBUG_ALL_PINS;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull  = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM; // GPIO_SPEED_FREQ_HIGH
    HAL_GPIO_Init(DEBUG_PORT, &GPIO_InitStructure);
#endif
}

//---------------------------------------------------------------------
/// <summary> External interrupt configure. </summary>
//---------------------------------------------------------------------
static void EXTI_Configure()
{
    // UART
    EXTI->IMR |= EXTI_IMR_IM0;
    HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    // Sync pin
    SetSyncPinAsInput();
}

//---------------------------------------------------------------------
/// <summary> Initialize USB. </summary>
//---------------------------------------------------------------------
static void USB_Init()
{
    USBD_Init(&USBD_Device, &VCP_Desc, 0);

    USBD_RegisterClass(&USBD_Device, &USBD_CDC);
    USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_STREAM_SC_CU_fops);
    USBD_Start(&USBD_Device);
}

//---------------------------------------------------------------------
/// <summary> Deinitialize USB. </summary>
//---------------------------------------------------------------------
static void USB_Deinit()
{
    USBD_Stop(&USBD_Device);
    USBD_DeInit(&USBD_Device);
}

//---------------------------------------------------------------------
/// <summary> Main initialization routine. </summary>
//---------------------------------------------------------------------
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
    Untrain();

    // reset counter just to be sure it's 0 at start
    header.packet_id = 0;

    HAL_ADC_Start_DMA(&ADC1_Handle, (uint32_t*)(&buffer[0][0]), sizeof(buffer) / sizeof(buffer[0][0]));

    USB_Init();
}

//---------------------------------------------------------------------
/// <summary> Normalize (train) coefficients. </summary>
///
/// <param name="buf"> Buffer holding ADC channel values. </param>
//---------------------------------------------------------------------
static void TrainCoefficients(uint32_t* buf)
{
    const int  Size              = 1000;
    static int cnt               = Size;
    static int accum[N_CHANNELS] = {0};

    for (int i = 0; i < N_CHANNELS; ++i)
        accum[i] += buf[i];

    if (--cnt <= 0) {
        float avg[N_CHANNELS];

        for (int i = 0; i < N_CHANNELS; ++i)
            avg[i] = (float)accum[i] / Size; // round instead of floor

        float total_mean = 0.f;

        for (int i = 0; i < N_CHANNELS; ++i)
            total_mean += avg[i];

        total_mean /= N_CHANNELS;

        for (int i = 0; i < N_CHANNELS; ++i)
            g_trained_coeffs[i] = total_mean / avg[i];

        // Reset local static variables
        cnt = Size;
        for (int i = 0; i < N_CHANNELS; ++i)
            accum[i] = 0;

        // Training over
        g_training = 0;
    }
}

//---------------------------------------------------------------------
/// <summary> Set output pins that drive IR LEDs. </summary>
///
/// <param name="activate_led"> State of LEDs. </param>
//---------------------------------------------------------------------
static void SetIRLEDs(ActiveLEDs activate_led)
{
    switch (activate_led) {
    case OFF:
        GPIO_CLR_BIT(IR_LED_CH1_PORT, IR_LED_CH1_PIN);
        GPIO_CLR_BIT(IR_LED_CH2_PORT, IR_LED_CH2_PIN);
        break;
    case CH1:
        GPIO_CLR_BIT(IR_LED_CH2_PORT, IR_LED_CH2_PIN);
        GPIO_SET_BIT(IR_LED_CH1_PORT, IR_LED_CH1_PIN);
        break;
    case CH2:
        GPIO_CLR_BIT(IR_LED_CH1_PORT, IR_LED_CH1_PIN);
        GPIO_SET_BIT(IR_LED_CH2_PORT, IR_LED_CH2_PIN);
        break;
    case ALL:
    default:
        GPIO_SET_BIT(IR_LED_CH1_PORT, IR_LED_CH1_PIN);
        GPIO_SET_BIT(IR_LED_CH2_PORT, IR_LED_CH2_PIN);
        break;
    }
}

//---------------------------------------------------------------------
/// <summary> Set active sequence index. </summary>
///
/// <param name="si"> Sequence index. </param>
//---------------------------------------------------------------------
static void GotoSequenceIndex(unsigned int si)
{
    // If no valid sequence just return
    if (sequence_N <= 0)
        return;

    if (si < sequence_N)
        sequence_idx = si;
    else
        sequence_idx = 0;

    SetIRLEDs(sequence[sequence_idx]);
}

//---------------------------------------------------------------------
/// <summary> Move to next index in sequence. </summary>
//---------------------------------------------------------------------
static void NextInSequence()
{
    GotoSequenceIndex(sequence_idx + 1);
}

//---------------------------------------------------------------------
/// <summary> Insert sorting data to buffer for later sending to PC. </summary>
///
/// <param name="raw_data"> Raw ADC values. </param>
/// <param name="filtered_data"> Filtered ADC values. </param>
//---------------------------------------------------------------------
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

//---------------------------------------------------------------------
/// <summary> Signal that object was detected. </summary>
///
/// <param name="ch"> Channel number where object was detected. </param>
//---------------------------------------------------------------------
static __attribute__((optimize("O2"))) void ObjectDetected(int ch)
{
    if ((1 << ch) & g_ejection_window)
        g_delay_ticker[ch] = g_delay_ticks_param;

    g_detected_objects |= 1 << ch;
}

//---------------------------------------------------------------------
/// <summary> Filter raw ADC channel data which is then used for object detection. </summary>
///
/// <param name="raw_data"> Pointer to raw ADC data for all channels. </param>
//---------------------------------------------------------------------
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

//---------------------------------------------------------------------
/// <summary> See communication.c for documentation. </summary>
//---------------------------------------------------------------------
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

//---------------------------------------------------------------------
/// <summary> Main program function. </summary>
//---------------------------------------------------------------------
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
