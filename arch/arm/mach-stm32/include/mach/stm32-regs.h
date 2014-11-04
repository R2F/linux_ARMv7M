#define STM32_USART1_BASE	0x40011000 /* APB2 */
#define USART_SR_FLAG_RXNE	0x20
#define USART_SR_FLAG_TXE	0x80

struct stm32_serial {
	uint32_t USART_SR;
	uint32_t USART_DR;
	uint32_t USART_BRR;
	uint32_t USART_CR1;
	uint32_t USART_CR2;
	uint32_t USART_CR3;
	uint32_t USART_GTPR;
};
