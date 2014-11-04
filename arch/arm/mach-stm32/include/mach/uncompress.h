#include <mach/stm32-regs.h>

static inline void putc(int c)
{
	volatile struct stm32_serial* base = (struct stm32_serial *)STM32_USART1_BASE;
	while((base->USART_SR & USART_SR_FLAG_TXE) == 0);
	base->USART_DR = c;
}

static inline void flush(void) {}
static inline void arch_decomp_setup(void) {}
