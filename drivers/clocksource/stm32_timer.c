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

static void __iomem *timer_base;

#define MIN_CCR_DELTA 16

#define TIMx_CR1	0x00
#define TIMx_CR1_CEN	BIT(0)

#define TIMx_DIER	0x0C
#define TIMx_DIER_CC1IE	BIT(1)

#define TIMx_SR	0x10
#define TIMx_SR_CC1IF	BIT(1)

#define TIMx_EGR	0x14
#define TIMx_EGR_UG	BIT(0)

#define TIMx_CNT	0x24
#define TIMx_PSC	0x28
#define TIMx_ARR	0x2C
#define TIMx_CCR1	0x34

static u64 notrace stm32_timer_sched_read(void)
{
	return readl(timer_base + TIMx_CNT);
}

static irqreturn_t stm32_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = (struct clock_event_device *)dev_id;

	writel(readl(timer_base + TIMx_SR) & (~TIMx_SR_CC1IF),
			timer_base + TIMx_SR);
	writel(readl(timer_base + TIMx_DIER) & (~TIMx_DIER_CC1IE),
			timer_base + TIMx_DIER);

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static void stm32_clkevt_mode(enum clock_event_mode mode,
			      struct clock_event_device *clk)
{
	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		printk("%s PERIODIC\n", __func__);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		printk("%s ONESHOT\n", __func__);
		break;
	case CLOCK_EVT_MODE_UNUSED:
		printk("%s UNUSED\n", __func__);
		break;
	case CLOCK_EVT_MODE_SHUTDOWN:
		printk("%s SHUTDOWN\n", __func__);
		break;
	default:
		printk("%s default\n", __func__);
		break;
	}
}

static int stm32_clkevt_next_event(unsigned long evt,
				   struct clock_event_device *unused)
{
	unsigned long next;

	next = readl(timer_base + TIMx_CNT) + evt;
	writel(next, timer_base + TIMx_CCR1);
	writel(TIMx_DIER_CC1IE, timer_base + TIMx_DIER);

	return 0;
}

static struct clock_event_device stm32_clockevent = {
	.name = "stm32_tick",
	.rating = 200,
	.features = CLOCK_EVT_FEAT_ONESHOT,
	.set_mode = stm32_clkevt_mode,
	.set_next_event = stm32_clkevt_next_event,
};

static struct irqaction stm32_timer_irq = {
	.name = "stm32_timer2",
	.flags = IRQF_TIMER | IRQF_IRQPOLL,
	.handler = stm32_timer_interrupt,
	.dev_id = &stm32_clockevent,
};

static void __init global_timer_of_register(struct device_node *np)
{
	int irq;
	struct clk * clk;
	unsigned long clk_rate;

	timer_base = of_iomap(np, 0);
	if(!timer_base) {
		pr_err("failed to map registers for clocksource\n");
		goto err_iomap;
	}

	clk = of_clk_get(np, 0);
	if(!clk){
		pr_err("failed to get clock for clockevent\n");
		goto err_get_clk;
	}
	clk_prepare_enable(clk);
	clk_rate = clk_get_rate(clk);
	if(!clk_rate){
		pr_err("failed to get clock rate for clockevent\n");
		clk_disable_unprepare(clk);
		goto err_get_clk;
	}
	irq = irq_of_parse_and_map(np, 0);
	if(!irq) {
		pr_err("failed to get irq for clockevent\n");
		goto err_get_irq;
	}

	setup_irq(irq, &stm32_timer_irq);

	writel(0, timer_base + TIMx_CR1);
	writel(0xFFFFFFFF, timer_base + TIMx_ARR);
	writel(0, timer_base + TIMx_PSC);
	writel(0, timer_base + TIMx_CNT);

	writel(readl(timer_base + TIMx_CR1) | TIMx_CR1_CEN,
			timer_base + TIMx_CR1);

	writel(readl(timer_base + TIMx_EGR) | TIMx_EGR_UG,
			timer_base + TIMx_EGR);

	clocksource_mmio_init(timer_base + TIMx_CNT, "stm32 timer",
				clk_rate, 200, 32, clocksource_mmio_readl_up);

	sched_clock_register(stm32_timer_sched_read, 32, clk_rate);

	clockevents_config_and_register(&stm32_clockevent,
				clk_rate, MIN_CCR_DELTA, 0xFFFFFFFF);
err_get_clk:
err_get_irq:
	iounmap(timer_base);
err_iomap:
	return;
}

CLOCKSOURCE_OF_DECLARE(stm32_tim, "stm32,timer",
			global_timer_of_register);
