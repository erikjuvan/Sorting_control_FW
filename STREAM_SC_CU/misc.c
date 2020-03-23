#include "stm32f7xx_hal.h"
#include "usbd_cdc_if.h"

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
volatile int cycles = GetCC();
double seconds_taken = cycles * (1.0 / (float)SystemCoreClock);
__asm__("nop");
*/

void EnableCC()
{
    CoreDebug->DEMCR |= 0x01000000; // Enable DWT and ITM features
    DWT->CYCCNT = 0;                // Set Cycle Count register to zero
    DWT->CTRL |= 1;                 // Enable CYCCNT
}

void DisableCC()
{
    DWT->CTRL &= ~1; // Disable CYCCNT
}

#define ResetCC() DWT->CYCCNT = 0 // Set Cycle Count register to zero
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
void StartST()
{
    SysTick->LOAD = SysTick_LOAD_RELOAD_Msk;
    SysTick->VAL  = 0;
    SysTick->CTRL = 5;
}

void StopST()
{
    SysTick->CTRL = 0;
}

#define GetSTCVR() SysTick->VAL // The number of core clock cycles taken by the operation is given by: (STCVR1 - STCVR2 - 2)
/////////////////////////////////

void OutputMCO()
{
    __GPIOA_CLK_ENABLE();
    __GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.Pin       = GPIO_PIN_8;
    GPIO_InitStructure.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull      = GPIO_NOPULL;
    GPIO_InitStructure.Speed     = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStructure.Alternate = 0;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

    __HAL_RCC_MCO1_CONFIG(RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_5);
    __HAL_RCC_MCO2_CONFIG(RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_5);
}

void WaitForGo()
{
    uint8_t ready = 0, buf[3] = {0, 0, 0};
    while (!ready) {
        VCP_read(buf, 3);
        if (buf[0] == 'g' && buf[1] == 'o') {
            ready = 1;
        }
    }
}
