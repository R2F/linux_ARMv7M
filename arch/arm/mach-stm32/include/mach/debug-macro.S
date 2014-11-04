#define STM32_USART1_BASE	0x40011000 /* APB2 */
#define USART_SR_FLAG_RXNE	0x20
#define USART_SR_FLAG_TXE	0x80


/*
 * FIXME: This is a copy of IMX_IO_P2V in hardware.h, and needs to
 * stay sync with that.  It's hard to maintain, and should be fixed
 * globally for multi-platform build to use a fixed virtual address
 * for low-level debug uart port across platforms.
 */
#define IMX_IO_P2V(x)	(						\
			(((x) & 0x80000000) >> 7) |			\
			(0xf4000000 +					\
			(((x) & 0x50000000) >> 6) +			\
			(((x) & 0x0b000000) >> 4) +			\
			(((x) & 0x000fffff))))

#define UART_VADDR	IMX_IO_P2V(STM32_USART1_BASE)

		.macro	addruart, rp, rv, tmp
		ldr	\rp, =STM32_USART1_BASE	@ physical
		ldr	\rv, =UART_VADDR	@ virtual
		.endm

		.macro	senduart,rd,rx
		str	\rd, [\rx, #0x4]	@ TXDATA
		.endm

		.macro	waituart,rd,rx
		.endm

		.macro	busyuart,rd,rx
1002:		ldr	\rd, [\rx, #0x0]	@ SR2
		tst	\rd, #1 << 7		@ TXDC
		beq	1002b			@ wait until transmit done
		.endm
