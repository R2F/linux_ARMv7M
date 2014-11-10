#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/cpu.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/sched_clock.h>
#include <linux/types.h>

#include <asm/cputype.h>

#define STM32_TIM2_BASE	0x40000000 /* APB1 */

#define MIN_OSCR_DELTA 16

struct stm32_tim2_5 {
	uint32_t cr1;
	uint32_t cr2;
	uint32_t smcr;
	uint32_t dier;
	uint32_t sr;
	uint32_t egr;
	uint32_t ccmr1;
	uint32_t ccmr2;
	uint32_t ccer;
	uint32_t cnt;
	uint32_t psc;
	uint32_t arr;
	uint32_t reserved1;
	uint32_t ccr1;
	uint32_t ccr2;
	uint32_t ccr3;
	uint32_t ccr4;
	uint32_t reserved2;
	uint32_t dcr;
	uint32_t dmar;
	uint32_t or;
};

#define STM32_TIMER	((volatile struct stm32_tim2_5*)STM32_TIM2_BASE)


static cycle_t gt_clocksource_read(struct clocksource *cs)
{
	return STM32_TIMER->cnt;
}

static cycle_t gt_clocksource_read2(void)
{
	return STM32_TIMER->cnt;
}

static irqreturn_t stm32_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = (struct clock_event_device *)dev_id;

	STM32_TIMER->sr &= (~(1 << 1));
	STM32_TIMER->dier &= (~(1 << 1));

	//printk("timer_int\n");

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct clocksource gt_clocksource = {
	.name	= "stm32_timer",
	.rating	= 300,
	.read	= gt_clocksource_read,
	.mask	= CLOCKSOURCE_MASK(32),
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};

static void stm32_clkevt_mode(enum clock_event_mode mode,
			      struct clock_event_device *clk)
{
	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		printk("%s PERIODIC\n", __func__);
		STM32_TIMER->dier &= ~(1 << 1);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		printk("%s ONESHOT\n", __func__);
		STM32_TIMER->dier &= ~(1 << 1);
		break;
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	default:
		printk("%s default\n", __func__);
		STM32_TIMER->dier &= ~(1 << 1);
		break;
	}
}

static int stm32_clkevt_next_event(unsigned long evt,
				   struct clock_event_device *unused)
{
	unsigned long next, oscr;


	if(evt > 1000000) {
		evt = 1000000;
	}

	STM32_TIMER->dier = (1 << 1);
	next = STM32_TIMER->cnt + evt;
	STM32_TIMER->ccr1 = next;
	oscr = STM32_TIMER->cnt;

	//printk("evt :%d clkevt_next_event next %d  oscr %d  psc %d\n", evt, next, oscr, STM32_TIMER->psc);

	return (signed)(next - oscr) <= MIN_OSCR_DELTA ? -ETIME : 0;
}

static struct clock_event_device stm32_clockevent = {
	.name = "stm32_tick",
	.rating = 350,
	.features = CLOCK_EVT_FEAT_ONESHOT,
	.set_mode = stm32_clkevt_mode,
	.set_next_event = stm32_clkevt_next_event,
};

static struct irqaction stm32_timer_irq = {
	.name = "stm32_timer0",
	.flags = IRQF_TIMER | IRQF_IRQPOLL,
	.handler = stm32_timer_interrupt,
	.dev_id = &stm32_clockevent,
};

static void __init global_timer_of_register(struct device_node *np)
{
	struct clk *gt_clk;
	int err = 0;
	int irq;

	printk("Registering stm32 timer\n");

	irq = irq_of_parse_and_map(np, 0);
	if (!irq) {
		pr_err("failed to get irq for clockevent\n");
	}

	err = setup_irq(irq, &stm32_timer_irq);
	if (err)
		pr_warn("failed to setup irq %d\n", 44);

	clocksource_register_hz(&gt_clocksource, 84000000);
	sched_clock_register(gt_clocksource_read2, 32, 84000000);

	clockevents_config_and_register(&stm32_clockevent,
					84000000, MIN_OSCR_DELTA * 2, 0x7fffffff);

	STM32_TIMER->cr1 = 0;
	STM32_TIMER->arr = 0xFFFFFFFF - 1;
	STM32_TIMER->psc = 0;
	STM32_TIMER->cnt = 0;
	STM32_TIMER->egr |= 1;
	STM32_TIMER->cr1 |= 1;
	STM32_TIMER->egr |= 1;

	return;
}

CLOCKSOURCE_OF_DECLARE(stm32_tim, "stm32,timer",
			global_timer_of_register);
