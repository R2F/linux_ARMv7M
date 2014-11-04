/*
 * board-stm32f429_discovery.c
 *
 *  Created on: Sep 23, 2014
 *      Author: limak
 */

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

static void __init stm32_board_init(void )
{

}

MACHINE_START(STM32F429_DISCO, "STM32F429 Discovery")
	.init_machine	= stm32_board_init,
MACHINE_END
