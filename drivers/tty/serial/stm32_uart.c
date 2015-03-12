#include <linux/module.h>
#include <linux/tty.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/tty_flip.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <asm/io.h>

#define DRIVER_NAME "stm32-usart"
#define DEV_NAME "ttystm"

#define USART_SR	0x00
#define USART_SR_RXNE	BIT(5)
#define USART_SR_TXE	BIT(7)

#define USART_DR	0x04
#define USART_BRR	0x08

#define USART_CR1	0x0C
#define USART_CR1_RXNEIE	BIT(5)
#define USART_CR1_TXEIE	BIT(7)

#define USART_CR2	0x10
#define USART_CR3	0x14
#define USART_GTPR	0x18

struct stm32_uart_port {
	struct uart_port	port;
};

#define to_stm_port(_port) container_of(_port, struct stm32_uart_port, port)

static void stm32_uart_write32(struct stm32_uart_port *stm_port,
		u32 value, unsigned offset)
{
	writel_relaxed(value, stm_port->port.membase + offset);
}

static u32 stm32_uart_read32(struct stm32_uart_port *stm_port,
		unsigned offset)
{
	return readl_relaxed(stm_port->port.membase + offset);
}

static unsigned int stm32_uart_tx_empty(struct uart_port *port)
{
	struct stm32_uart_port *stm_port = to_stm_port(port);
	return(stm32_uart_read32(stm_port, USART_SR) & USART_SR_TXE);
}

static unsigned int stm32_uart_get_mctrl(struct uart_port *port)
{
	return 0;
}

static void stm32_uart_set_mctrl(struct uart_port *port, unsigned int sigs)
{
}

static void stm32_uart_start_tx(struct uart_port *port)
{
	struct stm32_uart_port *stm_port = to_stm_port(port);
	u32 val;

	val = stm32_uart_read32(stm_port, USART_CR1);
	stm32_uart_write32(stm_port, val | USART_CR1_TXEIE, USART_CR1);
}

static void stm32_uart_stop_tx(struct uart_port *port)
{
	struct stm32_uart_port *stm_port = to_stm_port(port);
	u32 val;

	val = stm32_uart_read32(stm_port, USART_CR1);
	stm32_uart_write32(stm_port, val & (~USART_CR1_TXEIE), USART_CR1);
}

static void stm32_uart_stop_rx(struct uart_port *port)
{
}

static void stm32_uart_break_ctl(struct uart_port *port, int break_state)
{
}

static void stm32_uart_rx_chars(struct stm32_uart_port *stm_port)
{
	struct uart_port *port = &stm_port->port;
	u32 status;

	status = stm32_uart_read32(stm_port, USART_SR);

	while(status & USART_SR_RXNE) {
		u32 rxdata = stm32_uart_read32(stm_port, USART_DR);
		int flag = 0;

		port->icount.rx++;

		tty_insert_flip_char(&port->state->port,
				rxdata, flag);
		status = stm32_uart_read32(stm_port, USART_SR);
	}
}

static void stm32_uart_tx_chars(struct stm32_uart_port *stm_port)
{
	struct uart_port *port = &stm_port->port;
	struct circ_buf *xmit = &port->state->xmit;
	unsigned int pending, tx;

	pending = uart_circ_chars_pending(xmit);

	for(tx = 0; tx < pending; tx++) {
		while(!(stm32_uart_read32(stm_port, USART_SR) & USART_SR_TXE));
		stm32_uart_write32(stm_port, xmit->buf[xmit->tail], USART_DR);
		port->icount.tx++;
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
	}

	/* disable tx interrupts if nothing more to send */
	if (uart_circ_empty(xmit))
		stm32_uart_stop_tx(port);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);
}

static irqreturn_t stm32_uart_interrupt(int irq, void *data)
{
	struct stm32_uart_port *port = data;
	struct tty_port *tport = &port->port.state->port;
	int handled = IRQ_NONE;
	u32 status;

	spin_lock(&port->port.lock);

	status = stm32_uart_read32(port, USART_SR);

	if ((status & USART_SR_RXNE)) {
		stm32_uart_rx_chars(port);
		spin_unlock(&port->port.lock);
		tty_flip_buffer_push(tport);
		handled = IRQ_HANDLED;
		goto handled;
	}

	if ((status & USART_SR_TXE)) {
		stm32_uart_tx_chars(port);
		handled = IRQ_HANDLED;
	}

	spin_unlock(&port->port.lock);

handled:
	return handled;
}

static void stm32_uart_set_termios(struct uart_port *port,
				    struct ktermios *termios,
				    struct ktermios *old)
{
}

static int stm32_uart_request_port(struct uart_port *port)
{
	struct stm32_uart_port *stm_port = to_stm_port(port);
	int ret = 0;

	port->membase = ioremap(port->mapbase, 0x400);
	if(!stm_port->port.membase)
		ret = -ENOMEM;

	return ret;
}

static void stm32_uart_config_port(struct uart_port *port, int type)
{
	if((type & UART_CONFIG_TYPE) &&
			!stm32_uart_request_port(port))
		port->type = PORT_STM32;
}

static int stm32_uart_startup(struct uart_port *port)
{
	struct stm32_uart_port *stm_port = to_stm_port(port);
	u32 val;
	int ret = 0;

	ret = request_irq(port->irq, stm32_uart_interrupt, 0,
			DRIVER_NAME, port);
	if(ret) {
		printk("%s failed to register uart irq\n", __func__);
	} else {
		val = stm32_uart_read32(stm_port, USART_CR1);
		stm32_uart_write32(stm_port, val | USART_CR1_RXNEIE, USART_CR1);
	}

	return ret;
}

static void stm32_uart_shutdown(struct uart_port *port)
{
	struct stm32_uart_port *stm_port = to_stm_port(port);
	u32 val;

	val = stm32_uart_read32(stm_port, USART_CR1);
	stm32_uart_write32(stm_port, val & (~USART_CR1_RXNEIE), USART_CR1);
	free_irq(port->irq, stm_port);
}

static const char *stm32_uart_type(struct uart_port *port)
{
	return port->type == PORT_STM32 ? "stm32-usart" : NULL;
}

static void stm32_uart_release_port(struct uart_port *port)
{
	iounmap(port->membase);
}

static int stm32_uart_verify_port(struct uart_port *port,
				   struct serial_struct *ser)
{
	int ret = 0;

	if(ser->type != PORT_STM32)
		ret = -EINVAL;

	return ret;
}

#ifdef CONFIG_CONSOLE_POLL
static int stm32_uart_poll_get_char(struct uart_port *port)
{
	struct stm32_uart_port *stm_port = to_stm_port(port);

	while((stm32_uart_read32(stm_port, USART_SR) & USART_SR_RXNE) == 0) {
		cpu_relax();
	}

	return stm32_uart_read32(stm_port, USART_DR);
}

static void stm32_uart_poll_put_char(struct uart_port *port, unsigned char c)
{
	struct stm32_uart_port *stm_port = to_stm_port(port);

	while((stm32_uart_read32(stm_port, USART_SR) & USART_SR_TXE) == 0) {
		cpu_relax();
	}

	stm32_uart_write32(stm_port, c, USART_DR);
}
#endif

static struct uart_ops stm32_uart_ops = {
	.tx_empty	= stm32_uart_tx_empty,
	.get_mctrl	= stm32_uart_get_mctrl,
	.set_mctrl	= stm32_uart_set_mctrl,
	.start_tx	= stm32_uart_start_tx,
	.stop_tx	= stm32_uart_stop_tx,
	.stop_rx	= stm32_uart_stop_rx,
	.break_ctl	= stm32_uart_break_ctl,
	.startup	= stm32_uart_startup,
	.shutdown	= stm32_uart_shutdown,
	.set_termios	= stm32_uart_set_termios,
	.type		= stm32_uart_type,
	.request_port	= stm32_uart_request_port,
	.release_port	= stm32_uart_release_port,
	.config_port	= stm32_uart_config_port,
	.verify_port	= stm32_uart_verify_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char	= stm32_uart_poll_get_char,
	.poll_put_char	= stm32_uart_poll_put_char,
#endif
};

static struct stm32_uart_port *stm32_uart_ports[1];

#ifdef CONFIG_SERIAL_STM32_CONSOLE

static void stm32_console_write(struct console *co, const char *s, u_int count)
{
	struct stm32_uart_port *stm_port = stm32_uart_ports[co->index];
	int i = 0;
	u32 status;

	for(i = 0; i < count; i++) {
		do {
			status = stm32_uart_read32(stm_port, USART_SR);
		} while(!(status & USART_SR_TXE));
		stm32_uart_write32(stm_port, s[i], USART_DR);
	}
}

static int __init stm32_console_setup(struct console *co, char *options)
{
	struct stm32_uart_port *stm_port;

	if (co->index < 0 || co->index >= ARRAY_SIZE(stm32_uart_ports)) {
			unsigned i;
			for (i = 0; i < ARRAY_SIZE(stm32_uart_ports); ++i) {
				if (stm32_uart_ports[i]) {
					pr_warn("stm32-console: fall back to console index %u (from %hhi)\n",
							i, co->index);
					co->index = i;
					break;
				}
			}
		}

		stm_port = stm32_uart_ports[co->index];
		if (!stm_port) {
			pr_warn("stm32-console: No port at %d\n", co->index);
			return -ENODEV;
		}

	return 0;
}

static struct uart_driver stm32_uart;

static struct console stm32_console = {
	.name		= DEV_NAME,
	.write		= stm32_console_write,
	.device		= uart_console_device,
	.setup		= stm32_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &stm32_uart,
};

#endif /* CONFIG_SERIAL_STM32_CONSOLE*/

#if defined(CONFIG_OF)
static const struct of_device_id stm32_serial_dt_ids[] = {
	{ .compatible = "stm32,stm32-usart" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, stm32_serial_dt_ids);
#endif

static struct uart_driver stm32_uart = {
	.owner		= THIS_MODULE,
	.driver_name	= DRIVER_NAME,
	.dev_name	= DEV_NAME,
	.nr		= ARRAY_SIZE(stm32_uart_ports),
	.cons		= &stm32_console,
};

static int stm32_serial_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	return 0;
}

static int stm32_serial_resume(struct platform_device *pdev)
{
	return 0;
}

static int stm32_serial_probe(struct platform_device *pdev)
{
	struct stm32_uart_port *stm_port;
	struct resource *res;
	unsigned int line;
	int ret;

	stm_port = kzalloc(sizeof(*stm_port), GFP_KERNEL);
	if (!stm_port) {
		dev_printk(KERN_ERR, &pdev->dev, "failed to allocate private data\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -ENODEV;
		dev_printk(KERN_ERR, &pdev->dev, "failed to determine base address\n");
		goto err_get_base;
	}

	ret = platform_get_irq(pdev, 0);
	if (ret <= 0) {
		dev_printk(KERN_ERR, &pdev->dev, "failed to get uart irq\n");
		return ret;
	}

	stm_port->port.irq = ret;

	stm_port->port.dev = &pdev->dev;
	stm_port->port.mapbase = res->start;
	stm_port->port.type = PORT_STM32;
	stm_port->port.iotype = UPIO_MEM32;
	stm_port->port.fifosize = 64;
	stm_port->port.ops = &stm32_uart_ops;
	stm_port->port.flags = UPF_BOOT_AUTOCONF;

	/* TODO: get line number from DT */
	stm_port->port.line = 0;

	line = stm_port->port.line;

	if (line >= 0 && line < ARRAY_SIZE(stm32_uart_ports))
		stm32_uart_ports[line] = stm_port;

	ret = uart_add_one_port(&stm32_uart, &stm_port->port);
	if (ret) {
		dev_printk(KERN_ERR, &pdev->dev, "failed to add port: %d\n", ret);

		if (line >= 0 && line < ARRAY_SIZE(stm32_uart_ports))
			stm32_uart_ports[line] = NULL;
err_get_base:
		kfree(stm_port);
	} else {
		platform_set_drvdata(pdev, stm_port);
	}

	return ret;
}

static int stm32_serial_remove(struct platform_device *pdev)
{
	struct stm32_uart_port *stm_port = platform_get_drvdata(pdev);
	unsigned int line = stm_port->port.line;

	uart_remove_one_port(&stm32_uart, &stm_port->port);

	if (line >= 0 && line < ARRAY_SIZE(stm32_uart_ports))
		stm32_uart_ports[line] = NULL;

	kfree(stm_port);

	return 0;
}

static struct platform_driver stm32_serial_driver = {
	.probe		= stm32_serial_probe,
	.remove		= stm32_serial_remove,
	.suspend	= stm32_serial_suspend,
	.resume		= stm32_serial_resume,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table	= of_match_ptr(stm32_serial_dt_ids),
	},
};

static int __init stm32_serial_init(void)
{
	int ret;

	ret = uart_register_driver(&stm32_uart);
	if (ret)
		return ret;

	ret = platform_driver_register(&stm32_serial_driver);
	if (ret)
		uart_unregister_driver(&stm32_uart);

	return ret;
}

static void __exit stm32_serial_exit(void)
{
	platform_driver_unregister(&stm32_serial_driver);
	uart_unregister_driver(&stm32_uart);
}

module_init(stm32_serial_init);
module_exit(stm32_serial_exit);

MODULE_AUTHOR("Kamil Lulko");
MODULE_DESCRIPTION("STM32 serial port driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
