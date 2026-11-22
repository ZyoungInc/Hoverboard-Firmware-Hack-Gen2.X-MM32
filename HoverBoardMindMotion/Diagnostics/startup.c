// Standalone SPIN05 diagnostic startup. No motor timer or bridge registers are accessed.
#include <stdint.h>
#include "mm32_device.h"

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
extern void SystemInit(void);
extern int main(void);
extern void SysTick_Handler(void);

static void Default_Handler(void){ for(;;) __WFI(); }

void Reset_Handler(void){
	uint32_t *src = &_sidata, *dst = &_sdata;
	while(dst < &_edata) *dst++ = *src++;
	for(dst = &_sbss; dst < &_ebss;) *dst++ = 0;
	SystemInit();
	main();
	Default_Handler();
}

__attribute__((section(".isr_vector"), used))
static void (*const vectors[48])(void) = {
	[0] = (void (*)(void))&_estack,
	[1] = Reset_Handler,
	[2] = Default_Handler,
	[3] = Default_Handler,
	[11] = Default_Handler,
	[14] = Default_Handler,
	[15] = SysTick_Handler,
	[16 ... 47] = Default_Handler,
};
