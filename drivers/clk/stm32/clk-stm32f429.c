/*
 * clk-stm32f429.c

 *
 *  Created on: Dec 6, 2014
 *      Author: Rafal Fabich
 *      License GPL v2
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>

#define DRIVER_NAME "clk-stm32f429"

#if !defined(CONFIG_STM32_HSE_HZ)
#error "CONFIG_STM32_HSE_HZ not defined!"
#endif

/* PWR registers */
static void __iomem * pwr_base_reg;

#define STM32_PWR_CR			(pwr_base_reg + 0x00)
#define STM32_PWR_CR_VOS_MASK		(0xC << 6)
#define STM32_PWR_CR_VOS1		(0x2 << 6)
#define STM32_PWR_CR_VOS0		(0x1 << 6)
#define STM32_PWR_CR_VOS_SCALE_MODE_1	(0x3 << 6)
#define STM32_PWR_CR_VOS_SCALE_MODE_2	(0x2 << 6)
#define STM32_PWR_CR_VOS_SCALE_MODE_3	(0x1 << 6)

/* RCC registers */
static void __iomem * rcc_base_reg;

#define STM32_RCC_CR			(rcc_base_reg + 0x00)
#define STM32_RCC_CR_PLLSAIRDY		(0x1 << 29)
#define STM32_RCC_CR_PLLISAION		(0x1 << 28)
#define STM32_RCC_CR_PLLI2SRDY		(0x1 << 27)
#define STM32_RCC_CR_PLLI2SON		(0x1 << 26)
#define STM32_RCC_CR_PLLRDY		(0x1 << 25)
#define STM32_RCC_CR_PLLON		(0x1 << 24)
#define STM32_RCC_CR_CSSON		(0x1 << 19)
#define STM32_RCC_CR_HSEBYP		(0x1 << 18)
#define STM32_RCC_CR_HSERDY		(0x1 << 17)
#define STM32_RCC_CR_HSEON		(0x1 << 16)
#define STM32_RCC_CR_HSICAL_BIT		(8)
#define STM32_RCC_CR_HSICAL_MASK	(0xFF << 8)
#define STM32_RCC_CR_HSITRIM_BIT	(3)
#define STM32_RCC_CR_HSITRIM_MASK	(0x1F << 3)
#define STM32_RCC_CR_HSIRDY		(0x1 << 1)
#define STM32_RCC_CR_HSION		(0x1)

#define STM32_RCC_PLLCFGR		(rcc_base_reg + 0x04)
/* Divider for OTG-FS, SDIO */
#define STM32_RCC_PLLCFGR_PLLQ_BIT	(24)
#define STM32_RCC_PLLCFGR_PLLQ_MASK	(0xF << 24)
/* TODO: add 15 dividers */
/* Main PLL and audio PLL (I2S) source */
#define STM32_RCC_PLLCFGR_PLLSRC_BIT	(22)
#define STM32_RCC_PLLCFGR_PLLSRC_MASK	(0x1 << 22)
#define STM32_RCC_PLLCFGR_PLLSRC_HSI	(0x0 << 22)
#define STM32_RCC_PLLCFGR_PLLSRC_HSE	(0x1 << 22)
/* Divider for main sysclk */
#define STM32_RCC_PLLCFGR_PLLP_BIT	(16)
#define STM32_RCC_PLLCFGR_PLLP_MASK	(0x3 << 16)
#define STM32_RCC_PLLCFGR_PLLP_DIV2	(0x0 << 16)
#define STM32_RCC_PLLCFGR_PLLP_DIV4	(0x1 << 16)
#define STM32_RCC_PLLCFGR_PLLP_DIV6	(0x2 << 16)
#define STM32_RCC_PLLCFGR_PLLP_DIV8	(0x3 << 16)
/* Multiplier for VCO */
#define STM32_RCC_PLLCFGR_PLLN_BIT	(6)
#define STM32_RCC_PLLCFGR_PLLN_MASK	(0x1FF << 6)
/* TODO: add 0x1ff multiplication factors? */
/* Divider for main PLL and audio PLL clock */
#define STM32_RCC_PLLCFGR_PLLM_BIT	(0)
#define STM32_RCC_PLLCFGR_PLLM_MASK	(0x3F)
/* TODO: add 0x3f dividers? */

#define STM32_RCC_CFGR			(rcc_base_reg + 0x08)
/* MCO2 output */
#define STM32_RCC_CFGR_MCO2_BIT		(30)
#define STM32_RCC_CFGR_MCO2_MASK	(0x3 << 30)
#define STM32_RCC_CFGR_MCO2_SYSCLK	(0x0 << 30)
#define STM32_RCC_CFGR_MCO2_PLLI2S	(0x1 << 30)
#define STM32_RCC_CFGR_MCO2_HSE		(0x2 << 30)
#define STM32_RCC_CFGR_MCO2_PLL		(0x3 << 30)
/* MCO2 prescaler */
#define STM32_RCC_CFGR_MCO2PRE_BIT	(27)
#define STM32_RCC_CFGR_MCO2PRE_MASK	(0x7 << 27)
#define STM32_RCC_CFGR_MCO2PRE_DIVNO	(0x0 << 27)
#define STM32_RCC_CFGR_MCO2PRE_DIV2	(0x4 << 27)
#define STM32_RCC_CFGR_MCO2PRE_DIV3	(0x5 << 27)
#define STM32_RCC_CFGR_MCO2PRE_DIV4	(0x6 << 27)
#define STM32_RCC_CFGR_MCO2PRE_DIV5	(0x7 << 27)
/* MCO2 prescaler */
#define STM32_RCC_CFGR_MCO1PRE_BIT	(24)
#define STM32_RCC_CFGR_MCO1PRE_MASK	(0x7 << 24)
#define STM32_RCC_CFGR_MCO1PRE_DIVNO	(0x0 << 24)
#define STM32_RCC_CFGR_MCO1PRE_DIV2	(0x4 << 24)
#define STM32_RCC_CFGR_MCO1PRE_DIV3	(0x5 << 24)
#define STM32_RCC_CFGR_MCO1PRE_DIV4	(0x6 << 24)
#define STM32_RCC_CFGR_MCO1PRE_DIV5	(0x7 << 24)
/* I2S clock selection */
#define STM32_RCC_CFGR_I2SSRC_BIT	(23)
#define STM32_RCC_CFGR_I2SSRC_MASK	(0x1 << 23)
#define STM32_RCC_CFGR_I2SSRC_PLLI2S	(0x0 << 23)
#define STM32_RCC_CFGR_I2SSRC_I2SCKIN	(0x1 << 23)
/* MCO1 output */
#define STM32_RCC_CFGR_MCO1_BIT		(21)
#define STM32_RCC_CFGR_MCO1_MASK	(0x3 << 21)
#define STM32_RCC_CFGR_MCO1_HSI		(0x0 << 21)
#define STM32_RCC_CFGR_MCO1_LSE		(0x1 << 21)
#define STM32_RCC_CFGR_MCO1_HSE		(0x2 << 21)
#define STM32_RCC_CFGR_MCO1_PLL		(0x3 << 21)
/* RTC prescaler */
#define STM32_RCC_CFGR_RTCPRE_BIT	(16)
#define STM32_RCC_CFGR_RTCPRE_MASK	(0x1f << 16)
#define STM32_RCC_CFGR_RTCPRE_NOCLK	(0x0 << 16)
#define STM32_RCC_CFGR_RTCPRE_DIV2	(0x2 << 16)
/* TODO: add list of dividers: 0x4 - 0x1f (HSE/4 - HSE/31) */
/* APB2 prescaler */
#define STM32_RCC_CFGR_PPRE2_BIT	(13)
#define STM32_RCC_CFGR_PPRE2_MASK	(0x7 << 13)
#define STM32_RCC_CFGR_PPRE2_DIVNO	(0x0 << 13)
#define STM32_RCC_CFGR_PPRE2_DIV2	(0x4 << 13)
#define STM32_RCC_CFGR_PPRE2_DIV4	(0x5 << 13)
#define STM32_RCC_CFGR_PPRE2_DIV8	(0x6 << 13)
#define STM32_RCC_CFGR_PPRE2_DIV16	(0x7 << 13)
/* APB1 prescaler */
#define STM32_RCC_CFGR_PPRE1_BIT	(10)
#define STM32_RCC_CFGR_PPRE1_MASK	(0x7 << 10)
#define STM32_RCC_CFGR_PPRE1_DIVNO	(0x0 << 10)
#define STM32_RCC_CFGR_PPRE1_DIV2	(0x4 << 10)
#define STM32_RCC_CFGR_PPRE1_DIV4	(0x5 << 10)
#define STM32_RCC_CFGR_PPRE1_DIV8	(0x6 << 10)
#define STM32_RCC_CFGR_PPRE1_DIV16	(0x7 << 10)
/* AHB prescaler */
#define STM32_RCC_CFGR_HPRE_BIT		(4)
#define STM32_RCC_CFGR_HPRE_MASK	(0xF << 4)
#define STM32_RCC_CFGR_HPRE_DIVNO	(0x0 << 4)
#define STM32_RCC_CFGR_HPRE_DIV2	(0x8 << 4)
#define STM32_RCC_CFGR_HPRE_DIV4	(0x9 << 4)
#define STM32_RCC_CFGR_HPRE_DIV8	(0xA << 4)
#define STM32_RCC_CFGR_HPRE_DIV16	(0xB << 4)
#define STM32_RCC_CFGR_HPRE_DIV64	(0xC << 4)
#define STM32_RCC_CFGR_HPRE_DIV128	(0xD << 4)
#define STM32_RCC_CFGR_HPRE_DIV256	(0xE << 4)
#define STM32_RCC_CFGR_HPRE_DIV512	(0xF << 4)
/* System clock switch status */
#define STM32_RCC_CFGR_SWS_BIT		(2)
#define STM32_RCC_CFGR_SWS_MASK		(0x3 << 2)
#define STM32_RCC_CFGR_SWS_HSI		(0x0 << 2)
#define STM32_RCC_CFGR_SWS_HSE		(0x1 << 2)
#define STM32_RCC_CFGR_SWS_PLL		(0x2 << 2)
/* System clock switch */
#define STM32_RCC_CFGR_SW_BIT		(0)
#define STM32_RCC_CFGR_SW_MASK		(0x3)
#define STM32_RCC_CFGR_SW_HSI		(0x0)
#define STM32_RCC_CFGR_SW_HSE		(0x1)
#define STM32_RCC_CFGR_SW_PLL		(0x2)

#define STM32_RCC_CIR			(rcc_base_reg + 0x0C)
#define STM32_RCC_AHB1RSTR		(rcc_base_reg + 0x10)
#define STM32_RCC_AHB2RSTR		(rcc_base_reg + 0x14)
#define STM32_RCC_AHB3RSTR		(rcc_base_reg + 0x18)
#define STM32_RCC_APB1RSTR		(rcc_base_reg + 0x20)
#define STM32_RCC_APB2RSTR		(rcc_base_reg + 0x24)
#define STM32_RCC_AHB1ENR		(rcc_base_reg + 0x30)
#define STM32_RCC_AHB2ENR		(rcc_base_reg + 0x34)
#define STM32_RCC_AHB3ENR		(rcc_base_reg + 0x38)
#define STM32_RCC_APB1ENR		(rcc_base_reg + 0x40)
#define STM32_RCC_APB1ENR_PWREN		(0x1 << 28)
#define STM32_RCC_APB2ENR		(rcc_base_reg + 0x44)
#define STM32_RCC_AHB1LPENR		(rcc_base_reg + 0x50)
#define STM32_RCC_AHB2LPENR		(rcc_base_reg + 0x54)
#define STM32_RCC_AHB3LPENR		(rcc_base_reg + 0x58)
#define STM32_RCC_APB1LPENR		(rcc_base_reg + 0x60)
#define STM32_RCC_APB2LPENR		(rcc_base_reg + 0x64)
#define STM32_RCC_BDCR			(rcc_base_reg + 0x70)
#define STM32_RCC_CSR			(rcc_base_reg + 0x74)
#define STM32_RCC_SSCGR			(rcc_base_reg + 0x80)
#define STM32_RCC_PLLI2SCFGR		(rcc_base_reg + 0x84)
#define STM32_RCC_PLLSAICFGR		(rcc_base_reg + 0x88)
#define STM32_RCC_DCKCFGR		(rcc_base_reg + 0x8C)

enum clock {
	CLOCK_CORE,
	CLOCK_AHB,
	CLOCK_APB1,
	CLOCK_APB2,
	CLOCK_TIMER2,
	CLOCK_SYSTICK,
	CLOCK_MAX
};
static struct clk * clks[CLOCK_MAX];
static struct clk_onecell_data clk_data = {
	.clks = clks,
	.clk_num = CLOCK_MAX,
};

DEFINE_SPINLOCK(stm32_rcc_lock);

static int stm32_sysclk_setup(void)
{
	unsigned int v;

	pwr_base_reg = ioremap(0x40007000, 0x400);
	WARN_ON(!pwr_base_reg);

	/* set HSI as input */
	v = readl(STM32_RCC_CR);
	v |= STM32_RCC_CR_HSION;
	writel(v, STM32_RCC_CR);
	/* turn off HSE, CSS, PLL */
	v = readl(STM32_RCC_CR);
	v &= ~(STM32_RCC_CR_HSEON | STM32_RCC_CR_CSSON | STM32_RCC_CR_PLLON);
	writel(v, STM32_RCC_CR);

	/* Reset PLLCFGR, value from RM */
	writel(0x24003010, STM32_RCC_PLLCFGR);
	/* disable HSE bypass */
	v = readl(STM32_RCC_CR);
	v &= ~STM32_RCC_CR_HSEBYP;
	writel(v, STM32_RCC_CR);
	/* Disable all interrupts */
	writel(0, STM32_RCC_CIR);
	/* Reset CFGR */
	writel(0, STM32_RCC_CFGR);

	/* Configure for HSE+PLL operation */
	v = readl(STM32_RCC_CR);
	v &= ~(STM32_RCC_CR_HSION);
	v |= (STM32_RCC_CR_HSEON | STM32_RCC_CR_CSSON );
	writel(v, STM32_RCC_CR);
	while(!(readl(STM32_RCC_CR) & STM32_RCC_CR_HSERDY));

	/* Enable high performance mode, System frequency up to 168 MHz */
	v = readl(STM32_RCC_APB1ENR);
	v |= STM32_RCC_APB1ENR_PWREN;
	writel(v, STM32_RCC_APB1ENR);

	writel(STM32_PWR_CR_VOS_SCALE_MODE_1, STM32_PWR_CR);

	/* Count prescalers for the frequencies */
	/* now assume AHB=168MHz/APB1=42MHz/APB2=84MHz */
	v = readl(STM32_RCC_CFGR);
	v |= STM32_RCC_CFGR_HPRE_DIVNO;
	v |= STM32_RCC_CFGR_PPRE1_DIV4;
	v |= STM32_RCC_CFGR_PPRE2_DIV2;
	writel(v, STM32_RCC_CFGR);

	/* setup PLL */
	v = (8 << STM32_RCC_PLLCFGR_PLLM_BIT)
		| (336 << STM32_RCC_PLLCFGR_PLLN_BIT)
		| STM32_RCC_PLLCFGR_PLLP_DIV2
		| (7 << STM32_RCC_PLLCFGR_PLLQ_BIT);

	v |= STM32_RCC_PLLCFGR_PLLSRC_HSE;
	writel(v, STM32_RCC_PLLCFGR);

	/* enable PLL */
	v = readl(STM32_RCC_CR);
	v |= STM32_RCC_CR_PLLON;
	writel(v, STM32_RCC_CR);
	while(!((readl(STM32_RCC_CR)) & STM32_RCC_CR_PLLRDY));

	/* switch to PLL */
	v = readl(STM32_RCC_CFGR);
	v &= ~STM32_RCC_CFGR_SW_MASK;
	v |= STM32_RCC_CFGR_SW_PLL;
	writel(v, STM32_RCC_CFGR);

	while((readl(STM32_RCC_CFGR) & STM32_RCC_CFGR_SWS_MASK) != STM32_RCC_CFGR_SWS_PLL);

	return 0;
}

static unsigned long stm32_get_clock(enum clock clck)
{
	unsigned int sysclk = 0;
	unsigned int shift = 0;
	unsigned int v = 0;
	/* Prescaler table lookups for clock computation */
	unsigned char ahb_psc_table[16] =
	{0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
	unsigned char apb_psc_table[8] =
	{0, 0, 0, 0, 1, 2, 3, 4};

	v = readl(STM32_RCC_CFGR);
	if((v & STM32_RCC_CFGR_SWS_MASK) == STM32_RCC_CFGR_SWS_PLL) {
		u16 pllm, plln, pllp;
		pllm = (readl(STM32_RCC_PLLCFGR) & STM32_RCC_PLLCFGR_PLLM_MASK);
		plln = ((readl(STM32_RCC_PLLCFGR) & STM32_RCC_PLLCFGR_PLLN_MASK) >> 6);
		pllp = ((((readl(STM32_RCC_PLLCFGR) & STM32_RCC_PLLCFGR_PLLP_MASK) >> 16) + 1) << 1);
		sysclk = ((CONFIG_STM32_HSE_HZ / pllm) * plln) / pllp;
	}
	else{
		return 0;
	}
	switch(clck) {
		case CLOCK_CORE:
			return sysclk;
			break;
		case CLOCK_AHB:
			shift = ahb_psc_table[((readl(STM32_RCC_CFGR)  & STM32_RCC_CFGR_HPRE_MASK) >> 4)];
			return sysclk >>= shift;
			break;
		case CLOCK_APB1:
			shift = apb_psc_table[((readl(STM32_RCC_CFGR)  & STM32_RCC_CFGR_PPRE1_MASK) >> 10)];
			return sysclk >>= shift;
			break;
		case CLOCK_APB2:
			shift = apb_psc_table[((readl(STM32_RCC_CFGR)  & STM32_RCC_CFGR_PPRE2_MASK) >> 13)];
			return sysclk >>= shift ;
			break;
		case CLOCK_SYSTICK:
			return sysclk / 8;
			break;
		default:
			return 0;
			break;
	}
}

static void __init stm32_rcc_init(struct device_node *sysclk_node)
{
	struct device_node *np;

	np = sysclk_node;
	rcc_base_reg = of_iomap(np, 0);
	WARN_ON(!rcc_base_reg);

	stm32_sysclk_setup();

	clks[CLOCK_CORE] = clk_register_fixed_rate(NULL, "sysclk", NULL, CLK_IS_ROOT, stm32_get_clock(CLOCK_CORE));
	clk_prepare_enable(clks[CLOCK_CORE]);
	printk("%s: sysclk registered (%ldHz)\n",__func__, stm32_get_clock(CLOCK_CORE));

	clks[CLOCK_AHB] = clk_register_fixed_factor(NULL, "ahbclk", "sysclk", CLK_SET_RATE_PARENT, 1, 1);
	clk_prepare_enable(clks[CLOCK_AHB]);
	printk("%s: ahbclk registered (%ldHz)\n",__func__, stm32_get_clock(CLOCK_AHB));

	clks[CLOCK_APB1] = clk_register_fixed_factor(NULL, "apb1clk", "ahbclk", CLK_SET_RATE_PARENT, 1, 4);
	clk_prepare_enable(clks[CLOCK_APB1]);
	printk("%s: apb1clk registered (%ldHz)\n",__func__, stm32_get_clock(CLOCK_APB1));

	clks[CLOCK_APB2] = clk_register_fixed_factor(NULL, "apb2clk", "apb1clk", CLK_SET_RATE_PARENT, 2, 1);
	clk_prepare_enable(clks[CLOCK_APB2]);
	printk("%s: apb2clk registered (%ldHz)\n",__func__, stm32_get_clock(CLOCK_APB2));

	clks[CLOCK_TIMER2] = clk_register_gate(NULL, "timer2clk", "apb2clk", 0, rcc_base_reg + 0x40, 0, 0, &stm32_rcc_lock);
	of_clk_add_provider(np, of_clk_src_onecell_get, &clk_data);
}
CLK_OF_DECLARE(stm32_rcc, "stm32,rcc", stm32_rcc_init);

MODULE_AUTHOR("Rafal Fabich");
MODULE_DESCRIPTION("STM32 system clock driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
