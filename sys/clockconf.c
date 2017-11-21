#include "stm32f1xx.h"
#include "clockconf.h"

void configureSysClock (void) {
	FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_1;

	// Aktiviere Quarzoszillator
	RCC->CR = RCC_CR_HSEON | (16 << RCC_CR_HSITRIM_Pos) | RCC_CR_HSION;
	while ((RCC->CR & RCC_CR_HSERDY) == 0) __NOP ();
	
	// Konfiguriere PLL (Taktmultiplikation x9 => 72MHz), Bus-Prescaler (APB1 = 36MHz, APB2 = 72MHz)
	RCC->CFGR = (7 << RCC_CFGR_PLLMULL_Pos) | RCC_CFGR_PLLSRC | (4 << RCC_CFGR_PPRE1_Pos);
	
	// Aktiviere PLL
	RCC->CR = RCC_CR_HSEON | (16 << RCC_CR_HSITRIM_Pos) | RCC_CR_HSION | RCC_CR_PLLON;
	// Warte auf PLL Einschwingvorgang
	while ((RCC->CR & RCC_CR_PLLRDY) == 0) __NOP ();
	
	// Schalte Systemtakt auf PLL um
	RCC->CFGR = (7 << RCC_CFGR_PLLMULL_Pos) | RCC_CFGR_PLLSRC | (4 << RCC_CFGR_PPRE1_Pos) | RCC_CFGR_SW_1;
	while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_1) __NOP ();
	
	// Schalte internen RC-Oszillator ab
	RCC->CR = RCC_CR_HSEON | (16 << RCC_CR_HSITRIM_Pos) | RCC_CR_PLLON;
}
