#include <linux/kernel.h>

#include <asm/v7m.h>

#include <asm/mach/arch.h>

static const char *const stm32_compat[] __initconst = {
	"stm32,discovery",
	NULL
};

DT_MACHINE_START(STM32DT, "STM32 (Device Tree Support)")
	.dt_compat = stm32_compat,
	.restart = armv7m_restart,
	.nr_irqs = 64
MACHINE_END
