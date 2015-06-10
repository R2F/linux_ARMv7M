/*
 * STM32F429 pinctrl driver
 *
 * Copyright (C) 2015
 *
 * Author: Rafal Fabich <rafal.fabich@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

//#define DEBUG
#include <linux/device.h>

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
/* Since we request GPIOs from ourself */
#include <linux/pinctrl/consumer.h>


#include "core.h"
#include "stm32-pincfg.h"

#define MAX_GPIO_BANKS		11
#define MAX_NB_GPIO_PER_BANK	16

static void __iomem * g_exti_reg = NULL;
static void __iomem * g_syscfg_reg = NULL;

static int gpio_banks;
struct stm32_pinctrl_mux_ops;

struct stm32_gpio_chip {
	struct gpio_chip	chip;
	struct pinctrl_gpio_range range;
	struct stm32_gpio_chip	*next;		/* Bank sharing same clock */
	int			pioc_hwirq;	/* PIO bank interrupt identifier on AIC */
	int			pioc_virq;	/* PIO bank Linux virtual interrupt */
	int			pioc_idx;	/* PIO bank index */
	void __iomem		*regbase;	/* PIO bank virtual address */
//	void __iomem		*syscfg;	/* EXTI controller address */
//	void __iomem		*exti;	/* EXTI controller address */
	struct clk		*clock;		/* associated clock */
	struct stm32_pinctrl_mux_ops *ops;	/* ops */
};

#define to_stm32_gpio_chip(c) container_of(c, struct stm32_gpio_chip, chip)

static struct stm32_gpio_chip *gpio_chips[MAX_GPIO_BANKS];


struct stm32_pmx_func {
	const char	*name;
	const char	**groups;
	unsigned int	ngroups;
};

struct stm32_pmx_pin {
	uint32_t	bank;
	uint32_t	pin;
	enum stm32_pin_func	mux;
	unsigned long	conf;
	unsigned int use_exti_line;
};

struct stm32_pin_group {
	const char		*name;
	struct stm32_pmx_pin	*pins_conf;
	unsigned int		*pins;
	unsigned		npins;
};

struct stm32_pinctrl_mux_ops {
	enum stm32_pin_func (*get_periph)(void __iomem *pio, unsigned int pin);
	void (*set_periph)(void __iomem *pio, unsigned int pin, enum stm32_pin_func);
	enum stm32_pin_pupdr (*get_pullupdown)(void __iomem *pio, unsigned int pin);
	void (*set_pullupdown)(void __iomem *pio, unsigned int pin, enum stm32_pin_pupdr);
	enum stm32_pin_ospeedr (*get_speed)(void __iomem *pio, unsigned int pin);
	void (*set_speed)(void __iomem *pio, unsigned int pin, enum stm32_pin_ospeedr);
	enum stm32_pin_typer (*get_type)(void __iomem *pio, unsigned int pin);
	void (*set_type)(void __iomem *pio, unsigned int pin, enum stm32_pin_typer);
	enum stm32_pin_moder (*get_mode)(void __iomem *pio, unsigned int pin);
	void (*set_mode)(void __iomem *pio, unsigned int pin, enum stm32_pin_moder);

	/* irq */
	int (*irq_type)(struct irq_data *d, unsigned type);
};

struct stm32_pinctrl {
	struct device		*dev;
	struct pinctrl_dev	*pctl;
	int			nbanks;
//	uint32_t		*mux_mask;
//	int			nmux;
	struct stm32_pmx_func	*functions;
	int			nfunctions;
	struct stm32_pin_group	*groups;
	int			ngroups;

	struct stm32_pinctrl_mux_ops *ops;
};

static void __iomem *pin_to_controller(struct stm32_pinctrl *info,
				 unsigned int bank)
{
	return gpio_chips[bank]->regbase;
}

static inline int pin_to_bank(unsigned pin)
{
	return pin /= MAX_NB_GPIO_PER_BANK;
}

static unsigned pin_to_mask(unsigned int pin)
{
	return 1 << pin;
}

/* pin configuration ops */
enum stm32_pin_func stm32_get_periph(void __iomem *pio, unsigned int pin)
{
	unsigned int ret;

	if(pin < GPIO_PIN_8){
		ret = readl_relaxed(PIOOFFSET(pio,stm32_gpio_afrl));
		ret >>= (pin<<2);
	}
	else{
		ret = readl_relaxed(PIOOFFSET(pio,stm32_gpio_afrh));
		ret >>= ((pin-GPIO_PIN_8)<<2);
	}
	ret &= 0xF;
	return (enum stm32_pin_func)ret;
}

void stm32_set_periph(void __iomem *pio, unsigned int pin, enum stm32_pin_func func)
{
	unsigned int val;

	if(pin < GPIO_PIN_8){
		val = readl_relaxed(PIOOFFSET(pio,stm32_gpio_afrl));
		val |= (func << (pin<<2));
		writel_relaxed(val, PIOOFFSET(pio,stm32_gpio_afrl));
	}
	else{
		val = readl_relaxed(PIOOFFSET(pio,stm32_gpio_afrh));
		val |= (func << ((pin-GPIO_PIN_8)<<2));
		writel_relaxed(val, PIOOFFSET(pio,stm32_gpio_afrh));
	}
}

enum stm32_pin_pupdr stm32_get_pullupdown(void __iomem *pio, unsigned int pin)
{
	unsigned int ret;

	ret = readl_relaxed(PIOOFFSET(pio,stm32_gpio_pupdr));
	ret >>= (pin<<1);
	ret &= 0x3;

	return (enum stm32_pin_pupdr)ret;
}
void stm32_set_pullupdown(void __iomem *pio, unsigned int pin, enum stm32_pin_pupdr pupd)
{
	unsigned int val;

	val = readl_relaxed(PIOOFFSET(pio,stm32_gpio_pupdr));
	val |= (pupd << (pin<<1));
	writel_relaxed(val, PIOOFFSET(pio,stm32_gpio_pupdr));
}

enum stm32_pin_ospeedr stm32_get_speed(void __iomem *pio, unsigned int pin)
{
	unsigned int ret;

	ret = readl_relaxed(PIOOFFSET(pio,stm32_gpio_ospeedr));
	ret >>= (pin<<1);
	ret &= 0x3;

	return (enum stm32_pin_ospeedr)ret;
}
void stm32_set_speed(void __iomem *pio, unsigned int pin, enum stm32_pin_ospeedr speed)
{
	unsigned int val;

	val = readl_relaxed(PIOOFFSET(pio,stm32_gpio_ospeedr));
	val |= (speed << (pin<<1));
	writel_relaxed(val, PIOOFFSET(pio,stm32_gpio_ospeedr));
}

enum stm32_pin_typer stm32_get_type(void __iomem *pio, unsigned int pin)
{
	unsigned int ret;

	ret = readl_relaxed(PIOOFFSET(pio,stm32_gpio_otyper));
	ret >>= pin;
	ret &= 1;

	return (enum stm32_pin_typer)ret;
}
void stm32_set_type(void __iomem *pio, unsigned int pin, enum stm32_pin_typer type)
{
	unsigned int val;

	val = readl_relaxed(PIOOFFSET(pio,stm32_gpio_otyper));
	val |= (type << pin);
	writel_relaxed(val, PIOOFFSET(pio,stm32_gpio_otyper));
}

enum stm32_pin_moder stm32_get_mode(void __iomem *pio, unsigned int pin)
{
	unsigned int ret;

	ret = readl_relaxed(PIOOFFSET(pio,stm32_gpio_moder));
	ret >>= (pin<<1);
	ret &= 0x3;

	return (enum stm32_pin_moder)ret;
}
void stm32_set_mode(void __iomem *pio, unsigned int pin, enum stm32_pin_moder mode)
{
	unsigned int val;

	val = readl_relaxed(PIOOFFSET(pio,stm32_gpio_moder));
	val |= (mode << (pin<<1));
	writel_relaxed(val, PIOOFFSET(pio,stm32_gpio_moder));
}

static int alt_gpio_irq_type(struct irq_data *d, unsigned type)
{
	struct stm32_gpio_chip *stm32_gpio = irq_data_get_irq_chip_data(d);
	void __iomem	*pio = stm32_gpio->regbase;
	void __iomem	*exti = g_exti_reg;
	unsigned int	mask = 1 << d->hwirq;
	unsigned int val = 0;

//	printk("%s: d->hwirq = %x\n",__func__,d->hwirq);
	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		__irq_set_handler_locked(d->irq, handle_simple_irq);
		val = readl_relaxed(EXTIOFFSET(exti,stm32_exti_rtsr));
		val |= mask;
		writel_relaxed(mask, EXTIOFFSET(exti,stm32_exti_rtsr));

		val = readl_relaxed(EXTIOFFSET(exti,stm32_exti_ftsr));
		val &= ~mask;
		writel_relaxed(mask, EXTIOFFSET(exti,stm32_exti_ftsr));
		break;
	case IRQ_TYPE_EDGE_FALLING:
		__irq_set_handler_locked(d->irq, handle_simple_irq);
		val = readl_relaxed(EXTIOFFSET(exti,stm32_exti_ftsr));
		val |= mask;
		writel_relaxed(mask, EXTIOFFSET(exti,stm32_exti_ftsr));

		val = readl_relaxed(EXTIOFFSET(exti,stm32_exti_rtsr));
		val &= ~mask;
		writel_relaxed(mask, EXTIOFFSET(exti,stm32_exti_rtsr));
		break;

	case IRQ_TYPE_EDGE_BOTH:
		__irq_set_handler_locked(d->irq, handle_simple_irq);
		val = readl_relaxed(EXTIOFFSET(exti,stm32_exti_ftsr));
		val |= mask;
		writel_relaxed(mask, EXTIOFFSET(exti,stm32_exti_ftsr));

		val = readl_relaxed(EXTIOFFSET(exti,stm32_exti_rtsr));
		val |= mask;
		writel_relaxed(mask, EXTIOFFSET(exti,stm32_exti_rtsr));
		break;
	case IRQ_TYPE_LEVEL_LOW:
//		__irq_set_handler_locked(d->irq, handle_level_irq);
//		writel_relaxed(mask, pio + PIO_LSR);
//		writel_relaxed(mask, pio + PIO_FELLSR);
//		break;
	case IRQ_TYPE_LEVEL_HIGH:
//		__irq_set_handler_locked(d->irq, handle_level_irq);
//		writel_relaxed(mask, pio + PIO_LSR);
//		writel_relaxed(mask, pio + PIO_REHLSR);
//		break;
	case IRQ_TYPE_NONE:
	default:
		pr_warn("STM32: No type for irq %d\n", gpio_to_irq(d->irq));
		return -EINVAL;
	}

	/* enable additional interrupt modes */
//	writel_relaxed(mask, pio + PIO_AIMER);

	return 0;
}

static struct stm32_pinctrl_mux_ops stm32f429_ops = {
	.get_periph = stm32_get_periph,
	.set_periph = stm32_set_periph,
	.get_pullupdown = stm32_get_pullupdown,
	.set_pullupdown = stm32_set_pullupdown,
	.get_speed = stm32_get_speed,
	.set_speed = stm32_set_speed,
	.get_type = stm32_get_type,
	.set_type = stm32_set_type,
	.get_mode = stm32_get_mode,
	.set_mode = stm32_set_mode,
	.irq_type = alt_gpio_irq_type,
};


static struct of_device_id stm32_gpio_of_match[] = {
	{ .compatible = "stm32,stm32-gpio", .data = &stm32f429_ops},
	{ /* sentinel */ }
};
static const char *gpio_compat = "stm32,stm32-gpio";

static struct of_device_id stm32_pinctrl_of_match[] = {
	{ .compatible = "stm32,stm32-pinctrl", .data = &stm32f429_ops},
	{ /* sentinel */ }
};

//// GPIO functions
static int stm32_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	/*
	 * Map back to global GPIO space and request muxing, the direction
	 * parameter does not matter for this controller.
	 */
	int gpio = chip->base + offset;
	int bank = chip->base / chip->ngpio;

	dev_dbg(chip->dev, "%s:%d pio%c%d(%d)\n", __func__, __LINE__,
		 'A' + bank, offset, gpio);

	return pinctrl_request_gpio(gpio);
}

static void stm32_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	int gpio = chip->base + offset;

	pinctrl_free_gpio(gpio);
}

static int stm32_gpio_get_direction(struct gpio_chip *chip, unsigned offset)
{
	struct stm32_gpio_chip *stm32_gpio = to_stm32_gpio_chip(chip);
	void __iomem *pio = stm32_gpio->regbase;
	enum stm32_pin_moder mode;

	mode = stm32_get_mode(pio, offset);

	return (mode == GPIO_MODER_IN)?1:((mode==GPIO_MODER_OUT)?0:-1);
}

static int stm32_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct stm32_gpio_chip *stm32_gpio = to_stm32_gpio_chip(chip);
	void __iomem *pio = stm32_gpio->regbase;

	stm32_set_mode(pio, offset, GPIO_MODER_IN);
	return 0;
}

static int stm32_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct stm32_gpio_chip *stm32_gpio = to_stm32_gpio_chip(chip);
	void __iomem *pio = stm32_gpio->regbase;
	unsigned int val = 0;

	val = readl_relaxed(PIOOFFSET(pio,stm32_gpio_idr));
	val >>=offset;
	return (val & 1);
}

static void stm32_gpio_set(struct gpio_chip *chip, unsigned offset,
				int val)
{
	struct stm32_gpio_chip *stm32_gpio = to_stm32_gpio_chip(chip);
	void __iomem *pio = stm32_gpio->regbase;
	unsigned int reg = 0;

	reg = readl_relaxed(PIOOFFSET(pio,stm32_gpio_odr));

	if(val)
		reg |= (1 << offset);
	else
		reg &= ~(1 << offset);

	writel_relaxed(reg, PIOOFFSET(pio,stm32_gpio_odr));
}

static int stm32_gpio_direction_output(struct gpio_chip *chip, unsigned offset,
				int val)
{
	struct stm32_gpio_chip *stm32_gpio = to_stm32_gpio_chip(chip);
	void __iomem *pio = stm32_gpio->regbase;
	unsigned mask = 1 << offset;

	stm32_set_mode(pio, offset, GPIO_MODER_OUT);

	return 0;
}
#ifdef CONFIG_DEBUG_FS
static void stm32_gpio_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
	/*
	enum stm32_mux mode;
	int i;
	struct stm32_gpio_chip *stm32_gpio = to_stm32_gpio_chip(chip);
	void __iomem *pio = stm32_gpio->regbase;

	for (i = 0; i < chip->ngpio; i++) {
		unsigned mask = pin_to_mask(i);
		const char *gpio_label;
		u32 pdsr;

		gpio_label = gpiochip_is_requested(chip, i);
		if (!gpio_label)
			continue;
		mode = stm32_gpio->ops->get_periph(pio, mask);
		seq_printf(s, "[%s] GPIO%s%d: ",
			   gpio_label, chip->label, i);
		if (mode == AT91_MUX_GPIO) {
			pdsr = readl_relaxed(pio + PIO_PDSR);

			seq_printf(s, "[gpio] %s\n",
				   pdsr & mask ?
				   "set" : "clear");
		} else {
			seq_printf(s, "[periph %c]\n",
				   mode + 'A' - 1);
		}
	}
	*/
}
#else
#define stm32_gpio_dbg_show	NULL
#endif

static struct gpio_chip stm32_gpio_template = {
	.request		= stm32_gpio_request,
	.free			= stm32_gpio_free,
	.get_direction		= stm32_gpio_get_direction,
	.direction_input	= stm32_gpio_direction_input,
	.get			= stm32_gpio_get,
	.direction_output	= stm32_gpio_direction_output,
	.set			= stm32_gpio_set,
	.dbg_show		= stm32_gpio_dbg_show,
	.can_sleep		= false,
	.ngpio			= MAX_NB_GPIO_PER_BANK,
};


static void gpio_irq_mask(struct irq_data *d)
{
	struct stm32_gpio_chip *stm32_gpio = irq_data_get_irq_chip_data(d);
	void __iomem	*exti = g_exti_reg;
	unsigned int	mask = 1 << d->hwirq;

//	printk("%s: d->hwirq = %x\n",__func__,d->hwirq);
	if (exti){
		unsigned int val;
		val = readl_relaxed(EXTIOFFSET(exti,stm32_exti_imr));
		val &= ~mask;
		writel_relaxed(val, EXTIOFFSET(exti,stm32_exti_imr));
	}
}

static void gpio_irq_unmask(struct irq_data *d)
{
	struct stm32_gpio_chip *stm32_gpio = irq_data_get_irq_chip_data(d);
	void __iomem	*pio = stm32_gpio->regbase;
	void __iomem	*exti = g_exti_reg;
	unsigned	mask = 1 << d->hwirq;


//	printk("%s: d->hwirq = %x\n",__func__,d->hwirq);
	if (pio){
		unsigned int val;
		val = readl_relaxed(EXTIOFFSET(exti,stm32_exti_imr));
		val |= mask;
		writel_relaxed(val, EXTIOFFSET(exti,stm32_exti_imr));
	}
}



static void gpio_irq_ack(struct irq_data *d)
{
	/* the interrupt is already cleared before by reading ISR */
//	printk("%s: \n",__func__);
}


//static int setup_exti_line(struct stm32_gpio_chip *stm32_gpio, unsigned pin)
static int setup_exti_line(struct stm32_pinctrl *info, unsigned int pin)
{
	unsigned int bank = info->groups->pins_conf->bank;
	unsigned int new_val;
	unsigned int val;

	if(pin < 4){
		//exti0-3;
		new_val = (bank)<<(4*(pin%4));
		val = readl_relaxed(SYSCFGOFFSET(g_syscfg_reg,stm32_syscfg_exticr1));
		if(val & new_val)
			return -1;
		val |= new_val;
		writel_relaxed(val,SYSCFGOFFSET(g_syscfg_reg,stm32_syscfg_exticr1));
	}
	else if(pin < 8){
		//exti4-7
		new_val = (bank)<<(4*(pin%4));
		val = readl_relaxed(SYSCFGOFFSET(g_syscfg_reg,stm32_syscfg_exticr2));
		if(val & new_val)
			return -1;
		val |= new_val;
		writel_relaxed(val,SYSCFGOFFSET(g_syscfg_reg,stm32_syscfg_exticr2));
	}
	else if(pin < 12){
		//exti8-11
		new_val = (bank)<<(4*(pin%4));
		val = readl_relaxed(SYSCFGOFFSET(g_syscfg_reg,stm32_syscfg_exticr3));
		if(val & new_val)
			return -1;
		val |= new_val;
		writel_relaxed(val,SYSCFGOFFSET(g_syscfg_reg,stm32_syscfg_exticr3));
	}
	else if(pin < 15){
		//exti12-15
		new_val = (bank)<<(4*(pin%4));
		val = readl_relaxed(SYSCFGOFFSET(g_syscfg_reg,stm32_syscfg_exticr4));
		if(val & new_val)
			return -1;
		val |= new_val;
		writel_relaxed(val,SYSCFGOFFSET(g_syscfg_reg,stm32_syscfg_exticr4));
	}
	else
		return -2;
	return 0;
}


static unsigned int gpio_irq_startup(struct irq_data *d)
{
	struct stm32_gpio_chip *stm32_gpio = irq_data_get_irq_chip_data(d);
//	void __iomem	*exti = g_exti_reg;
	unsigned	pin = d->hwirq;
	int ret;


//	printk("%s: d->hwirq = %x\n",__func__,d->hwirq);

	ret = gpio_lock_as_irq(&stm32_gpio->chip, pin);
	if (ret) {
		dev_err(stm32_gpio->chip.dev, "unable to lock pind %lu IRQ\n",
			d->hwirq);
		return ret;
	}
	gpio_irq_unmask(d);
	return 0;
}

static void gpio_irq_shutdown(struct irq_data *d)
{
	struct stm32_gpio_chip *stm32_gpio = irq_data_get_irq_chip_data(d);
	unsigned	pin = d->hwirq;


//	printk("%s: d->hwirq = %x\n",__func__,d->hwirq);
	gpio_irq_mask(d);
	gpio_unlock_as_irq(&stm32_gpio->chip, pin);
}

static struct irq_chip gpio_irqchip = {
	.name		= "GPIO",
	.irq_ack	= gpio_irq_ack,
	.irq_startup	= gpio_irq_startup,
	.irq_shutdown	= gpio_irq_shutdown,
	.irq_disable	= gpio_irq_mask,
	.irq_mask	= gpio_irq_mask,
	.irq_unmask	= gpio_irq_unmask,
	/* .irq_set_type is set dynamically */
	.irq_set_wake	= NULL,//gpio_irq_set_wake,
};

static void gpio_irq_handler(unsigned irq, struct irq_desc *desc)
{
	struct irq_chip *chip = irq_get_chip(irq);
	struct gpio_chip *gpio_chip = irq_desc_get_handler_data(desc);
	struct stm32_gpio_chip *stm32_gpio = container_of(gpio_chip, struct stm32_gpio_chip, chip);

	void __iomem	*pio = stm32_gpio->regbase;
	void __iomem	*exti = g_exti_reg;
	unsigned long	isr;
	int		n;

	chained_irq_enter(chip, desc);
	for (;;) {
		/* Reading ISR acks pending (edge triggered) GPIO interrupts.
		 * When there are none pending, we're finished unless we need
		 * to process multiple banks (like ID_PIOCDE on sam9263).
		 */
		isr = readl_relaxed(EXTIOFFSET(exti,stm32_exti_pr)) & readl_relaxed(EXTIOFFSET(exti,stm32_exti_imr));
		if (!isr) {
			if (!stm32_gpio->next)
				break;
			stm32_gpio = stm32_gpio->next;
			pio = stm32_gpio->regbase;
			gpio_chip = &stm32_gpio->chip;
			printk("%s: isr==0 \n",__func__);
			continue;
		}
		else{
			printk("%s: isr!=0 \n",__func__);
		}

		for_each_set_bit(n, &isr, BITS_PER_LONG) {
			generic_handle_irq(irq_find_mapping(gpio_chip->irqdomain, n));
		}
	}
	chained_irq_exit(chip, desc);
	/* now it may re-trigger */
}

static int stm32_gpio_of_irq_setup(struct device_node *node,
				  struct stm32_gpio_chip *stm32_gpio)
{
	struct stm32_gpio_chip   *prev = NULL;
	struct irq_data		*d = irq_get_irq_data(stm32_gpio->pioc_virq);
	int ret;
//	void __iomem	*exti = stm32_gpio->exti;

	stm32_gpio->pioc_hwirq = irqd_to_hwirq(d);

//	printk("%s: stm32_gpio->pioc_hwirq = %x\n",__func__,stm32_gpio->pioc_hwirq);
	/* Setup proper .irq_set_type function */
	gpio_irqchip.irq_set_type = stm32_gpio->ops->irq_type;

	/* Disable irqs of this PIO controller */
//	writel_relaxed(~0, at91_gpio->regbase + PIO_IDR);

	/*
	 * Let the generic code handle this edge IRQ, the the chained
	 * handler will perform the actual work of handling the parent
	 * interrupt.
	 */
	ret = gpiochip_irqchip_add(&stm32_gpio->chip,
				   &gpio_irqchip,
				   0,
				   handle_edge_irq,
				   IRQ_TYPE_EDGE_BOTH);
	if (ret)
		panic("stm32_gpio.%d: couldn't allocate irq domain (DT).\n",
				stm32_gpio->pioc_idx);

	/* Setup chained handler */
	if (stm32_gpio->pioc_idx)
		prev = gpio_chips[stm32_gpio->pioc_idx - 1];

	/* The top level handler handles one bank of GPIOs, except
	 * on some SoC it can handle up to three...
	 * We only set up the handler for the first of the list.
	 */
	if (prev && prev->next == stm32_gpio)
		return 0;

	/* Then register the chain on the parent IRQ */
	gpiochip_set_chained_irqchip(&stm32_gpio->chip,
				     &gpio_irqchip,
				     stm32_gpio->pioc_virq,
				     gpio_irq_handler);

	return 0;
}


static void stm32_gpio_probe_fixup(void)
{
	unsigned i;
	struct stm32_gpio_chip *stm32_gpio, *last = NULL;

	for (i = 0; i < gpio_banks; i++) {
		stm32_gpio = gpio_chips[i];
		/*
		 * GPIO controller are grouped on some SoC:
		 * PIOC, PIOD and PIOE can share the same IRQ line
		 */
		if (last && last->pioc_virq == stm32_gpio->pioc_virq)
			last->next = stm32_gpio;
		last = stm32_gpio;
	}
}

inline static void stm32_gpio_core_interrupt_ctrl_config(void)
{
	g_syscfg_reg = ioremap(STM32_SYSCFG_REGBASE, STM32_SYSCFG_MEM_SIZE);
	g_exti_reg = ioremap(STM32_EXTI_REGBASE, STM32_EXTI_MEM_SIZE);
}

static int stm32_gpio_core_probe(struct platform_device *pdev)
{
	struct device_node *np = NULL;
	struct resource *res;
	struct stm32_gpio_chip *stm32_chip = NULL;
	struct gpio_chip *chip;
	struct pinctrl_gpio_range *range;
	int ret = 0;
	int irq, i;
	int alias_idx;
	uint32_t ngpio;
	char **names;

	if(pdev)
		np = pdev->dev.of_node;
	if(np)
		alias_idx = of_alias_get_id(np, "gpio");

	dev_dbg(&pdev->dev,"%s: \n",__func__);
	BUG_ON(alias_idx >= ARRAY_SIZE(gpio_chips));
	if (gpio_chips[alias_idx]) {
		ret = -EBUSY;
		goto err;
	}

	irq = platform_get_irq(pdev, 0);

	if (irq < 0) {
		ret = irq;
		goto err;
	}

	stm32_chip = devm_kzalloc(&pdev->dev, sizeof(*stm32_chip), GFP_KERNEL);
	if (!stm32_chip) {
		ret = -ENOMEM;
		goto err;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	stm32_chip->regbase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(stm32_chip->regbase)) {
		ret = PTR_ERR(stm32_chip->regbase);
		goto err;
	}

	dev_dbg(&pdev->dev, "at address %p; syscfg: %p; exti: %p\n", stm32_chip->regbase,g_syscfg_reg,g_exti_reg);

	stm32_chip->ops = (struct stm32_pinctrl_mux_ops *)
		of_match_device(stm32_gpio_of_match, &pdev->dev)->data;
stm32_chip->pioc_virq = irq;
	stm32_chip->pioc_idx = alias_idx;

	stm32_chip->clock = clk_get(&pdev->dev, NULL);
	if (IS_ERR(stm32_chip->clock)) {
		dev_err(&pdev->dev, "failed to get clock, ignoring.\n");
		goto err;
	}

	if (clk_prepare(stm32_chip->clock))
		goto clk_prep_err;

	/* enable PIO controller's clock */
	if (clk_enable(stm32_chip->clock)) {
		dev_err(&pdev->dev, "failed to enable clock, ignoring.\n");
		goto clk_err;
	}

	stm32_chip->chip = stm32_gpio_template;

	chip = &stm32_chip->chip;
	chip->of_node = np;
	chip->label = dev_name(&pdev->dev);
	chip->dev = &pdev->dev;
	chip->owner = THIS_MODULE;
	chip->base = alias_idx * MAX_NB_GPIO_PER_BANK;

	if (!of_property_read_u32(np, "#gpio-lines", &ngpio)) {
		if (ngpio >= MAX_NB_GPIO_PER_BANK)
			pr_err("stm32_gpio.%d, gpio-nb >= %d failback to %d\n",
			       alias_idx, MAX_NB_GPIO_PER_BANK, MAX_NB_GPIO_PER_BANK);
		else
			chip->ngpio = ngpio;
	}

	names = devm_kzalloc(&pdev->dev, sizeof(char *) * chip->ngpio,
			     GFP_KERNEL);

	if (!names) {
		ret = -ENOMEM;
		goto clk_err;
	}

	for (i = 0; i < chip->ngpio; i++)
		names[i] = kasprintf(GFP_KERNEL, "pio%c%d", alias_idx + 'A', i);

	chip->names = (const char *const *)names;

	range = &stm32_chip->range;
	range->name = chip->label;
	range->id = alias_idx;
	range->pin_base = range->base = range->id * MAX_NB_GPIO_PER_BANK;

	range->npins = chip->ngpio;
	range->gc = chip;

	ret = gpiochip_add(chip);
	if (ret)
		goto clk_err;

	gpio_chips[alias_idx] = stm32_chip;
	gpio_banks = max(gpio_banks, alias_idx + 1);

//	stm32_gpio_probe_fixup();

	stm32_gpio_of_irq_setup(np, stm32_chip);

	dev_info(&pdev->dev, "at address %p\n", stm32_chip->regbase);

	return 0;

clk_err:
	clk_unprepare(stm32_chip->clock);
clk_prep_err:
	clk_put(stm32_chip->clock);
err:
	dev_err(&pdev->dev, "Failure %i for GPIO %i\n", ret, alias_idx);

	return ret;
}

static int stm32_gpio_probe(struct platform_device *pdev)
{
	dev_dbg(&pdev->dev,"%s: \n",__func__);
	return stm32_gpio_core_probe(pdev);
}


//// PINCTRL functions/ops

static int pin_check_config(struct stm32_pinctrl *info, const char *name,
			    int index, const struct stm32_pmx_pin *pin)
{
	int mux;

	/* check if it's a valid config */
	if (pin->bank >= info->nbanks) {
		dev_err(info->dev, "%s: pin conf %d bank_id %d >= nbanks %d\n",
			name, index, pin->bank, info->nbanks);
		return -EINVAL;
	}

	if (pin->pin >= MAX_NB_GPIO_PER_BANK) {
		dev_err(info->dev, "%s: pin conf %d pin_bank_id %d >= %d\n",
			name, index, pin->pin, MAX_NB_GPIO_PER_BANK);
		return -EINVAL;
	}

	if (!pin->mux)
		return 0;
/*
	mux = pin->mux - 1;

	if (mux >= info->nmux) {
		dev_err(info->dev, "%s: pin conf %d mux_id %d >= nmux %d\n",
			name, index, mux, info->nmux);
		return -EINVAL;
	}

	if (!(info->mux_mask[pin->bank * info->nmux + mux] & 1 << pin->pin)) {
		dev_err(info->dev, "%s: pin conf %d mux_id %d not supported for pio%c%d\n",
			name, index, mux, pin->bank + 'A', pin->pin);
		return -EINVAL;
	}
*/
	return 0;
}

static void stm32_pin_dbg(const struct device *dev, const struct stm32_pmx_pin *pin)
{
	if (pin->mux) {
		dev_dbg(dev, "pio%c%d configured as periph %c with conf = 0x%lx\n",
			pin->bank + 'A', pin->pin, pin->mux, pin->conf);
	} else {
		dev_dbg(dev, "pio%c%d configured as gpio with conf = 0x%lx\n",
			pin->bank + 'A', pin->pin, pin->conf);
	}
}

static int stm32_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct stm32_pinctrl *info = pinctrl_dev_get_drvdata(pctldev);

	return info->ngroups;
}

static const char *stm32_get_group_name(struct pinctrl_dev *pctldev,
				       unsigned selector)
{
	struct stm32_pinctrl *info = pinctrl_dev_get_drvdata(pctldev);

	return info->groups[selector].name;
}

static int stm32_get_group_pins(struct pinctrl_dev *pctldev, unsigned selector,
			       const unsigned **pins,
			       unsigned *npins)
{
	struct stm32_pinctrl *info = pinctrl_dev_get_drvdata(pctldev);

	if (selector >= info->ngroups)
		return -EINVAL;

	*pins = info->groups[selector].pins;
	*npins = info->groups[selector].npins;

	return 0;
}

static void stm32_pin_dbg_show(struct pinctrl_dev *pctldev, struct seq_file *s,
		   unsigned offset)
{
	seq_printf(s, "%s", dev_name(pctldev->dev));
}

static const inline struct stm32_pin_group *stm32_pinctrl_find_group_by_name(
				const struct stm32_pinctrl *info,
				const char *name)
{
	const struct stm32_pin_group *grp = NULL;
	int i;

	for (i = 0; i < info->ngroups; i++) {
		if (strcmp(info->groups[i].name, name))
			continue;

		grp = &info->groups[i];
		dev_dbg(info->dev, "%s: %d 0:%d\n", name, grp->npins, grp->pins[0]);
		break;
	}

	return grp;
}

static int stm32_dt_node_to_map(struct pinctrl_dev *pctldev,
			struct device_node *np,
			struct pinctrl_map **map, unsigned *num_maps)
{
	struct stm32_pinctrl *info = pinctrl_dev_get_drvdata(pctldev);
	const struct stm32_pin_group *grp;
	struct pinctrl_map *new_map;
	struct device_node *parent;
	int map_num = 1;
	int i;

	/*
	 * first find the group of this node and check if we need to create
	 * config maps for pins
	 */
	grp = stm32_pinctrl_find_group_by_name(info, np->name);
	if (!grp) {
		dev_err(info->dev, "unable to find group for node %s\n",
			np->name);
		return -EINVAL;
	}

	map_num += grp->npins;
	new_map = devm_kzalloc(pctldev->dev, sizeof(*new_map) * map_num, GFP_KERNEL);
	if (!new_map)
		return -ENOMEM;

	*map = new_map;
	*num_maps = map_num;

	/* create mux map */
	parent = of_get_parent(np);
	if (!parent) {
		devm_kfree(pctldev->dev, new_map);
		return -EINVAL;
	}
	new_map[0].type = PIN_MAP_TYPE_MUX_GROUP;
	new_map[0].data.mux.function = parent->name;
	new_map[0].data.mux.group = np->name;
	of_node_put(parent);

	/* create config map */
	new_map++;
	for (i = 0; i < grp->npins; i++) {
		new_map[i].type = PIN_MAP_TYPE_CONFIGS_PIN;
		new_map[i].data.configs.group_or_pin =
				pin_get_name(pctldev, grp->pins[i]);
		new_map[i].data.configs.configs = &grp->pins_conf[i].conf;
		new_map[i].data.configs.num_configs = 1;
	}

	dev_dbg(pctldev->dev, "maps: function %s group %s num %d\n",
		(*map)->data.mux.function, (*map)->data.mux.group, map_num);

	return 0;
}

static void stm32_dt_free_map(struct pinctrl_dev *pctldev,
				struct pinctrl_map *map, unsigned num_maps)
{
}

static const struct pinctrl_ops stm32_pctrl_ops = {
	.get_groups_count	= stm32_get_groups_count,
	.get_group_name		= stm32_get_group_name,
	.get_group_pins		= stm32_get_group_pins,
	.pin_dbg_show		= stm32_pin_dbg_show,
	.dt_node_to_map		= stm32_dt_node_to_map,
	.dt_free_map		= stm32_dt_free_map,
};

static void stm32_mux_disable_interrupt(void __iomem *pio, unsigned mask)
{
	//writel_relaxed(mask, pio + PIO_IDR);
}

static int stm32_pmx_get_funcs_count(struct pinctrl_dev *pctldev)
{
	struct stm32_pinctrl *info = pinctrl_dev_get_drvdata(pctldev);

	return info->nfunctions;
}

static const char *stm32_pmx_get_func_name(struct pinctrl_dev *pctldev,
					  unsigned selector)
{
	struct stm32_pinctrl *info = pinctrl_dev_get_drvdata(pctldev);

	return info->functions[selector].name;
}

static int stm32_pmx_get_groups(struct pinctrl_dev *pctldev, unsigned selector,
			       const char * const **groups,
			       unsigned * const num_groups)
{
	struct stm32_pinctrl *info = pinctrl_dev_get_drvdata(pctldev);

	*groups = info->functions[selector].groups;
	*num_groups = info->functions[selector].ngroups;

	return 0;
}

static int stm32_pmx_enable(struct pinctrl_dev *pctldev, unsigned selector,
			   unsigned group)
{
	struct stm32_pinctrl *info = pinctrl_dev_get_drvdata(pctldev);
	const struct stm32_pmx_pin *pins_conf = info->groups[group].pins_conf;
	const struct stm32_pmx_pin *pin;
	uint32_t npins = info->groups[group].npins;
	int i, ret;
	unsigned mask;
	void __iomem *pio;

	dev_dbg(info->dev, "enable function %s group %s\n",
		info->functions[selector].name, info->groups[group].name);

	/* first check that all the pins of the group are valid with a valid
	 * parameter */
	for (i = 0; i < npins; i++) {
		pin = &pins_conf[i];
		ret = pin_check_config(info, info->groups[group].name, i, pin);
		if (ret)
			return ret;
	}

	for (i = 0; i < npins; i++) {
		pin = &pins_conf[i];
		stm32_pin_dbg(info->dev, pin);
		pio = pin_to_controller(info, pin->bank);

		//mask = pin_to_mask(pin->pin);
		//stm32_mux_disable_interrupt(pio, mask);

		if(!info->ops->set_periph)
			return -EINVAL;

		info->ops->set_periph(pio, pin->pin, pin->mux);


/*
		if (pin->mux)
			at91_mux_gpio_disable(pio, mask);
*/
	}

	return 0;
}

static int stm32_gpio_request_enable(struct pinctrl_dev *pctldev,
				    struct pinctrl_gpio_range *range,
				    unsigned offset)
{
	struct stm32_pinctrl *npct = pinctrl_dev_get_drvdata(pctldev);
	struct stm32_gpio_chip *stm32_chip;
	struct gpio_chip *chip;
	unsigned mask;

	if (!range) {
		dev_err(npct->dev, "invalid range\n");
		return -EINVAL;
	}
	if (!range->gc) {
		dev_err(npct->dev, "missing GPIO chip in range\n");
		return -EINVAL;
	}
	chip = range->gc;
	stm32_chip = container_of(chip, struct stm32_gpio_chip, chip);

	dev_dbg(npct->dev, "enable pin %u as GPIO\n", offset);

	mask = 1 << (offset - chip->base);

	dev_dbg(npct->dev, "enable pin %u as PIO%c%d 0x%x\n",
		offset, 'A' + range->id, offset - chip->base, mask);

//	writel_relaxed(mask, stm32_chip->regbase + PIO_PER);

	return 0;
}

static void stm32_gpio_disable_free(struct pinctrl_dev *pctldev,
				   struct pinctrl_gpio_range *range,
				   unsigned offset)
{
	struct stm32_pinctrl *npct = pinctrl_dev_get_drvdata(pctldev);

	dev_dbg(npct->dev, "disable pin %u as GPIO\n", offset);
	/* Set the pin to some default state, GPIO is usually default */
}

static const struct pinmux_ops stm32_pmx_ops = {
	.get_functions_count	= stm32_pmx_get_funcs_count,
	.get_function_name	= stm32_pmx_get_func_name,
	.get_function_groups	= stm32_pmx_get_groups,
	.enable			= stm32_pmx_enable,
	.gpio_request_enable	= stm32_gpio_request_enable,
	.gpio_disable_free	= stm32_gpio_disable_free,
};

static int stm32_pinconf_get(struct pinctrl_dev *pctldev,
			     unsigned pin_id, unsigned long *config)
{
	struct stm32_pinctrl *info = pinctrl_dev_get_drvdata(pctldev);
	void __iomem *pio;
	unsigned pin;
	int div;

	enum stm32_pin_moder mode = 0;
	enum stm32_pin_typer otype = 0;
	enum stm32_pin_ospeedr ospeed = 0;
	enum stm32_pin_pupdr pupd = 0;

	*config = 0;
	dev_dbg(info->dev, "%s:%d, pin_id=%d", __func__, __LINE__, pin_id);
	pio = pin_to_controller(info, pin_to_bank(pin_id));
	pin = pin_id % MAX_NB_GPIO_PER_BANK;


	if(info->ops->set_mode){
		mode = info->ops->get_mode(pio, pin);
	}
	if(info->ops->set_type){
		otype = info->ops->get_type(pio, pin);
	}
	if(info->ops->set_speed){
		ospeed = info->ops->get_speed(pio, pin);
	}
	if(info->ops->set_pullupdown){
		pupd = info->ops->get_pullupdown(pio, pin);
	}

	*config = STM32_GPIO_PIN_CONFIG(info->groups->pins_conf->use_exti_line,mode,otype,ospeed,pupd);

	return 0;
}

static int stm32_pinconf_set(struct pinctrl_dev *pctldev,
			     unsigned pin_id, unsigned long *configs,
			     unsigned num_configs)
{
	struct stm32_pinctrl *info = pinctrl_dev_get_drvdata(pctldev);
	unsigned int pin;
	void __iomem *pio;
	int i;
	unsigned long config;

	for (i = 0; i < num_configs; i++) {
		config = configs[i];

		dev_dbg(info->dev,
			"%s:%d, pin_id=%d, config=0x%lx",
			__func__, __LINE__, pin_id, config);
		pio = pin_to_controller(info, pin_to_bank(pin_id));
		pin = pin_id % MAX_NB_GPIO_PER_BANK;

		if(info->ops->set_mode){
			info->ops->set_mode(pio, pin, STM32_GPIO_PIN_GET_MODER(config));
		}
		if(info->ops->set_pullupdown){
			info->ops->set_pullupdown(pio, pin, STM32_GPIO_PIN_GET_PUPDR(config));
		}
		if(info->ops->set_speed){
			info->ops->set_speed(pio, pin, STM32_GPIO_PIN_GET_OSPEEDR(config));
		}
		if(info->ops->set_type){
			info->ops->set_type(pio, pin, STM32_GPIO_PIN_GET_OTYPER(config));
		}

	} /* for each config */

	return 0;
}

static void stm32_pinconf_dbg_show(struct pinctrl_dev *pctldev,
				   struct seq_file *s, unsigned pin_id)
{
	unsigned long config;
	int val, num_conf = 0;

//	stm32_pinconf_get(pctldev, pin_id, &config);

	return;
}

static void stm32_pinconf_group_dbg_show(struct pinctrl_dev *pctldev,
					 struct seq_file *s, unsigned group)
{

}

static const struct pinconf_ops stm32_pinconf_ops = {
	.pin_config_get			= stm32_pinconf_get,
	.pin_config_set			= stm32_pinconf_set,
	.pin_config_dbg_show		= stm32_pinconf_dbg_show,
	.pin_config_group_dbg_show	= stm32_pinconf_group_dbg_show,
};

static struct pinctrl_desc stm32_pinctrl_desc = {
	.pctlops	= &stm32_pctrl_ops,
	.pmxops		= &stm32_pmx_ops,
	.confops	= &stm32_pinconf_ops,
	.owner		= THIS_MODULE,
};

static void stm32_pinctrl_child_count(struct stm32_pinctrl *info,
				     struct device_node *np)
{
	struct device_node *child;

	for_each_child_of_node(np, child) {
		if (of_device_is_compatible(child, gpio_compat)) {
			info->nbanks++;
		} else {
			info->nfunctions++;
			info->ngroups += of_get_child_count(child);
		}
	}
}
/*
static int stm32_pinctrl_mux_mask(struct stm32_pinctrl *info,
				 struct device_node *np)
{
	int ret = 0;
	int size;
	const __be32 *list;

	list = of_get_property(np, "stm32,mux-mask", &size);
	if (!list) {
		dev_err(info->dev, "can not read the mux-mask of %d\n", size);
		return -EINVAL;
	}

	size /= sizeof(*list);
	if (!size || size % info->nbanks) {
		dev_err(info->dev, "wrong mux mask array should be by %d\n", info->nbanks);
		return -EINVAL;
	}
	info->nmux = size / info->nbanks;

	info->mux_mask = devm_kzalloc(info->dev, sizeof(unsigned int) * size, GFP_KERNEL);
	if (!info->mux_mask) {
		dev_err(info->dev, "could not alloc mux_mask\n");
		return -ENOMEM;
	}

	ret = of_property_read_u32_array(np, "stm32,mux-mask", info->mux_mask, size);
	if (ret)
		dev_err(info->dev, "can not read the mux-mask of %d\n", size);
	return ret;
}
*/

static int stm32_pinctrl_parse_groups(struct device_node *np,
				     struct stm32_pin_group *grp,
				     struct stm32_pinctrl *info, u32 index)
{
	struct stm32_pmx_pin *pin;
	int size;
	const __be32 *list;
	int i, j;

	dev_dbg(info->dev, "group(%d): %s\n", index, np->name);

	/* Initialise group */
	grp->name = np->name;

	/*
	 * the binding format is stm32,pins = <bank pin mux CONFIG ...>,
	 * do sanity check and calculate pins number
	 */
	list = of_get_property(np, "stm32,pins", &size);
	/* we do not check return since it's safe node passed down */
	size /= sizeof(*list);
	if (!size || size % 4) {
		dev_err(info->dev, "wrong pins number or pins and configs should be by 4\n");
		return -EINVAL;
	}

	grp->npins = size / 4;
	pin = grp->pins_conf = devm_kzalloc(info->dev, grp->npins * sizeof(struct stm32_pmx_pin),
				GFP_KERNEL);
	grp->pins = devm_kzalloc(info->dev, grp->npins * sizeof(unsigned int),
				GFP_KERNEL);
	if (!grp->pins_conf || !grp->pins)
		return -ENOMEM;

	for (i = 0, j = 0; i < size; i += 4, j++) {
		pin->bank = be32_to_cpu(*list++);
		pin->pin = be32_to_cpu(*list++);
		grp->pins[j] = pin->bank * MAX_NB_GPIO_PER_BANK + pin->pin;
		pin->mux = be32_to_cpu(*list++);
		pin->conf = be32_to_cpu(*list++);

		if(STM32_GPIO_PIN_GET_EXTI_IS_SET(pin->conf))
		{
			int ret;

			if(pin->use_exti_line){
				printk("%s: EXTI already set\n",__func__);
				goto exit;
			}
			pin->use_exti_line = 1;

			ret = setup_exti_line(info,pin->pin);
			if(ret == -1){
				dev_dbg(info->dev,"%s: line for pin%c-%d taken already\n",__func__,pin->bank+'A',pin->pin);
			}
			else if(ret < 0){
				dev_dbg(info->dev,"%s: error in specifying EXTI line for pin%c-%d n",__func__,pin->bank+'A',pin->pin);
			}
			else{
				dev_dbg(info->dev,"%s: EXTI line for pin%c-%d set up\n",__func__,pin->bank+'A',pin->pin);
			}
		}
exit:
		stm32_pin_dbg(info->dev, pin);
		pin++;
	}

	return 0;
}

static int stm32_pinctrl_parse_functions(struct device_node *np,
					struct stm32_pinctrl *info, unsigned int index)
{
	struct device_node *child;
	struct stm32_pmx_func *func;
	struct stm32_pin_group *grp;
	int ret;
	static unsigned int grp_index;
	unsigned int i = 0;

	dev_dbg(info->dev, "parse function(%d): %s\n", index, np->name);

	func = &info->functions[index];

	/* Initialise function */
	func->name = np->name;
	func->ngroups = of_get_child_count(np);
	if (func->ngroups == 0) {
		dev_err(info->dev, "no groups defined\n");
		return -EINVAL;
	}
	func->groups = devm_kzalloc(info->dev,
			func->ngroups * sizeof(char *), GFP_KERNEL);
	if (!func->groups)
		return -ENOMEM;

	for_each_child_of_node(np, child) {
		func->groups[i] = child->name;
		grp = &info->groups[grp_index++];
		ret = stm32_pinctrl_parse_groups(child, grp, info, i++);
		if (ret)
			return ret;
	}

	return 0;
}


static int stm32_pinctrl_probe_dt(struct platform_device *pdev,
				 struct stm32_pinctrl *info)
{
	int ret = 0;
	int i, j;
	uint32_t *tmp;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *child;

	if (!np)
		return -ENODEV;

	info->dev = &pdev->dev;
	info->ops = (struct stm32_pinctrl_mux_ops *)of_match_device(stm32_pinctrl_of_match, &pdev->dev)->data;
	stm32_pinctrl_child_count(info, np);

	if (info->nbanks < 1) {
		dev_err(&pdev->dev, "you need to specify at least one gpio-controller\n");
		return -EINVAL;
	}
/*
	ret = stm32_pinctrl_mux_mask(info, np);
	if (ret)
		return ret;

	dev_dbg(&pdev->dev, "nmux = %d\n", info->nmux);

	dev_dbg(&pdev->dev, "mux-mask\n");
	tmp = info->mux_mask;
	for (i = 0; i < info->nbanks; i++) {
		for (j = 0; j < info->nmux; j++, tmp++) {
			dev_dbg(&pdev->dev, "%d:%d\t0x%x\n", i, j, tmp[0]);
		}
	}
*/
	//dev_dbg(&pdev->dev, "nfunctions = %d\n", info->nfunctions);
	//dev_dbg(&pdev->dev, "ngroups = %d\n", info->ngroups);
	info->functions = devm_kzalloc(&pdev->dev, info->nfunctions * sizeof(struct stm32_pmx_func),
					GFP_KERNEL);
	if (!info->functions)
		return -ENOMEM;

	info->groups = devm_kzalloc(&pdev->dev, info->ngroups * sizeof(struct stm32_pin_group),
					GFP_KERNEL);
	if (!info->groups)
		return -ENOMEM;

	//dev_dbg(&pdev->dev, "nbanks = %d\n", info->nbanks);
	//dev_dbg(&pdev->dev, "nfunctions = %d\n", info->nfunctions);
	//dev_dbg(&pdev->dev, "ngroups = %d\n", info->ngroups);

	i = 0;

	for_each_child_of_node(np, child) {
		if (of_device_is_compatible(child, gpio_compat))
			continue;
		ret = stm32_pinctrl_parse_functions(child, info, i++);
		if (ret) {
			dev_err(&pdev->dev, "failed to parse function\n");
			return ret;
		}
	}

	return 0;
}


static int stm32_pinctrl_core_probe(struct platform_device *pdev)
{
	struct stm32_pinctrl *info;
	struct pinctrl_pin_desc *pdesc;
	int ret, i, j, k;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;


	stm32_gpio_core_interrupt_ctrl_config();

	ret = stm32_pinctrl_probe_dt(pdev, info);
	if (ret)
		return ret;

	/*
	 * We need all the GPIO drivers to probe FIRST, or we will not be able
	 * to obtain references to the struct gpio_chip * for them, and we
	 * need this to proceed.
	 */
	for (i = 0; i < info->nbanks; i++) {
		if (!gpio_chips[i]) {
			dev_warn(&pdev->dev, "GPIO chip %d not registered yet\n", i);
			devm_kfree(&pdev->dev, info);
			return -EPROBE_DEFER;
		}
	}

	stm32_pinctrl_desc.name = dev_name(&pdev->dev);
	stm32_pinctrl_desc.npins = info->nbanks * MAX_NB_GPIO_PER_BANK;
	stm32_pinctrl_desc.pins = pdesc =
		devm_kzalloc(&pdev->dev, sizeof(*pdesc) * stm32_pinctrl_desc.npins, GFP_KERNEL);

	if (!stm32_pinctrl_desc.pins)
		return -ENOMEM;

	for (i = 0 , k = 0; i < info->nbanks; i++) {
		for (j = 0; j < MAX_NB_GPIO_PER_BANK; j++, k++) {
			pdesc->number = k;
			pdesc->name = kasprintf(GFP_KERNEL, "pio%c%d", i + 'A', j);
			pdesc++;
		}
	}

	platform_set_drvdata(pdev, info);
	info->pctl = pinctrl_register(&stm32_pinctrl_desc, &pdev->dev, info);

	if (!info->pctl) {
		dev_err(&pdev->dev, "could not register STM32 pinctrl driver\n");
		ret = -EINVAL;
		goto err;
	}

	/* We will handle a range of GPIO pins */
	for (i = 0; i < info->nbanks; i++)
		pinctrl_add_gpio_range(info->pctl, &gpio_chips[i]->range);

	dev_info(&pdev->dev, "initialized STM32 pinctrl driver\n");

	return 0;

err:
	return ret;
}

static int stm32_pinctrl_core_remove(struct platform_device *pdev)
{
	struct stm32_pinctrl *info = platform_get_drvdata(pdev);
	pinctrl_unregister(info->pctl);

	return 0;
}

///
static int stm32_pinctrl_probe(struct platform_device *pdev)
{
	return stm32_pinctrl_core_probe(pdev);
}

static int stm32_pinctrl_remove(struct platform_device *pdev)
{
	return stm32_pinctrl_core_remove(pdev);
}

static struct platform_driver stm32_gpio_driver = {
	.driver = {
		.name = "stm32-gpio",
		.owner = THIS_MODULE,
		.of_match_table = stm32_gpio_of_match,
	},
	.probe = stm32_gpio_probe,
};

static struct platform_driver stm32_pinctrl_driver = {
	.driver = {
		.name = "stm32-pinctrl",
		.owner = THIS_MODULE,
		.of_match_table = stm32_pinctrl_of_match,
	},
	.probe = stm32_pinctrl_probe,
	.remove = stm32_pinctrl_remove,
};

static int __init stm32_pinctrl_init(void)
{
	int ret;

	ret = platform_driver_register(&stm32_gpio_driver);
	if (ret)
		return ret;
	ret = platform_driver_register(&stm32_pinctrl_driver);

	return ret;
}
arch_initcall(stm32_pinctrl_init);

static void __exit stm32_pinctrl_exit(void)
{
	platform_driver_unregister(&stm32_pinctrl_driver);
}
module_exit(stm32_pinctrl_exit);
MODULE_AUTHOR("Rafal Fabich <rafal.fabich@gmail.com>");
MODULE_DESCRIPTION("STM32F429 pinctrl driver");
MODULE_LICENSE("GPL v2");
