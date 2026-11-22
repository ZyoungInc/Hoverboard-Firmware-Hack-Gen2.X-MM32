// Gen2.4.2 hardware diagnostics: no TIM1, PWM alternate functions or motor control.
#include <stdint.h>
#include "mcu.h"

#define DIAG_MAGIC 0x47443234u   // "GD24" in the debugger's little-endian memory view

struct DiagnosticStatus {
	uint32_t magic;
	uint32_t milliseconds;
	uint16_t vbat_adc;       // PB1 / ADC1 channel 9, raw 12-bit count; divider not calibrated
	uint8_t hall;            // bit 0 PC15, bit 1 PC14, bit 2 PC13
	uint8_t button;          // PB5, raw level
	uint8_t latch;           // PB2 output level
	uint8_t adc_valid;
};
volatile struct DiagnosticStatus diagnostic_status;
static volatile uint32_t milliseconds;

void SysTick_Handler(void){ milliseconds++; }

static void mode(GPIO_TypeDef *port, unsigned pin, uint32_t nibble){
	volatile u32 *reg = pin < 8 ? &port->CRL : &port->CRH;
	unsigned shift = (pin & 7u) * 4u;
	*reg = (*reg & ~(0xFu << shift)) | (nibble << shift);
}

static void pins_init(void){
	RCC->AHBENR |= RCC_AHBENR_GPIOA | RCC_AHBENR_GPIOB | RCC_AHBENR_GPIOC;
	// Make all six bridge inputs high impedance. Never select a TIM1 alternate function.
	for(unsigned pin = 8; pin <= 10; pin++) mode(GPIOA, pin, GPIO_CNF_MODE_FLOATING);
	for(unsigned pin = 13; pin <= 15; pin++) mode(GPIOB, pin, GPIO_CNF_MODE_FLOATING);
	for(unsigned pin = 13; pin <= 15; pin++) mode(GPIOC, pin, GPIO_CNF_MODE_FLOATING);
	mode(GPIOB, 5, GPIO_CNF_MODE_FLOATING); // SW sense
	mode(GPIOB, 1, GPIO_CNF_MODE_AIN);      // divided battery voltage
	// Ghost labels PB2 as the power hold output. Set its data bit before enabling output mode.
	GPIOB->BSRR = 1u << 2;
	mode(GPIOB, 2, GPIO_CNF_MODE_50MHZ_OUT_PP);
}

static void uart_init(void){
	RCC->APB2ENR |= RCC_APB2ENR_UART1;
	// Ghost: PB6 UART1 TX, PB4 UART1 RX. Match the main firmware's open-drain bus TX.
	GPIOB->BSRR = 1u << 6;
	GPIOB->AFRL = (GPIOB->AFRL & ~(0xFu << 24)) | (0u << 24);
	GPIOB->AFRL = (GPIOB->AFRL & ~(0xFu << 16)) | (3u << 16);
	mode(GPIOB, 6, GPIO_CNF_MODE_50MHZ_AF_OD);
	mode(GPIOB, 4, GPIO_CNF_MODE_FLOATING);
	uint32_t div = SystemCoreClock / 115200u;
	UART1->CCR = UART_CCR_CHAR_8b;
	UART1->BRR = div / 16u;
	UART1->FRA = div % 16u;
	UART1->GCR = UART_GCR_UART | UART_GCR_TX | UART_GCR_RX;
}

static void adc_init(void){
	RCC->APB2ENR |= RCC_APB2ENR_ADC1;
	ADC1->ADCFG = ADC_CFGR_PRE_6 | ADC_CFGR_SAMCTL_239_5;
	ADC1->ADCR = ADC_CR_CONTINUE;           // software-triggered, no TIM1 trigger
	ADC1->ADCHS = ADC_CHSR_CH9;
	ADC1->ADCFG |= ADC_CFGR_ADEN;
	ADC1->ADCR |= ADC_CR_ADST;
}

static void putc_uart(char c){
	// A disconnected UART must never stall the diagnostic heartbeat.
	uint32_t limit = 100000u;
	while((UART1->CSR & UART_CSR_TXFULL) && --limit) {}
	if(limit) UART1->TDR = (uint8_t)c;
}

static void puts_uart(const char *s){ while(*s) putc_uart(*s++); }

static void putnum(uint32_t v){
	char digits[10];
	unsigned n = 0;
	do { digits[n++] = (char)('0' + v % 10u); v /= 10u; } while(v && n < sizeof digits);
	while(n) putc_uart(digits[--n]);
}

int main(void){
	pins_init();
	SysTick_Config(SystemCoreClock / 1000u);
	uart_init();
	adc_init();
	diagnostic_status.magic = DIAG_MAGIC;
	puts_uart("GEN2.4.2 DIAG SPIN05 115200 8N1\r\n");
	uint32_t last = 0;
	for(;;){
		uint32_t now = milliseconds;
		if(now - last < 500u) continue;
		last = now;
		uint32_t hall = GPIOC->IDR;
		diagnostic_status.milliseconds = now;
		diagnostic_status.hall = (uint8_t)(((hall >> 15) & 1u) | ((hall >> 13) & 2u) | ((hall >> 11) & 4u));
		diagnostic_status.button = (uint8_t)((GPIOB->IDR >> 5) & 1u);
		diagnostic_status.latch = (uint8_t)((GPIOB->ODR >> 2) & 1u);
		diagnostic_status.adc_valid = (uint8_t)((ADC1->ADSTA >> (ADC_SR_VALID_Pos + 9)) & 1u);
		diagnostic_status.vbat_adc = (uint16_t)(ADC1->ADDR9 & 0xFFFu);
		puts_uart("ms="); putnum(now);
		puts_uart(" hall="); putnum(diagnostic_status.hall);
		puts_uart(" button="); putnum(diagnostic_status.button);
		puts_uart(" latch="); putnum(diagnostic_status.latch);
		puts_uart(" vbat_adc="); putnum(diagnostic_status.vbat_adc);
		puts_uart(" adc_valid="); putnum(diagnostic_status.adc_valid);
		puts_uart("\r\n");
	}
}
