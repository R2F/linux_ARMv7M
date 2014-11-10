#include <linux/module.h>
#include <linux/tty.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/serial.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/tty_flip.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>
#include <linux/timer.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/err.h>
#include <linux/irq.h>

#include <asm/io.h>
#include <asm/ioctls.h>

#include <linux/serial_core.h>

#define DRIVER_NAME "stm32-uart"
#define DEV_NAME "ttystm"

#define STM32_USART1_BASE	0x40011000 /* APB2 */

#define USART_SR_FLAG_RXNE	0x20
#define USART_SR_FLAG_TXE	0x80

struct stm32_usart_struct {
	uint32_t USART_SR;
	uint32_t USART_DR;
	uint32_t USART_BRR;
	uint32_t USART_CR1;
	uint32_t USART_CR2;
	uint32_t USART_CR3;
	uint32_t USART_GTPR;
};

#define STM32_USART	((volatile struct stm32_usart_struct*)STM32_USART1_BASE)

static struct console stm32_console;

struct uart_stm32_port {
	struct uart_port	port;
};

static unsigned int stm32_uart_tx_empty(struct uart_port *port)
{
	printk("%s\n", __func__);
	return(STM32_USART->USART_SR & USART_SR_FLAG_TXE);
}

static unsigned int stm32_uart_get_mctrl(struct uart_port *port)
{
	printk("%s\n", __func__);
	return 0;
}

static void stm32_uart_set_mctrl(struct uart_port *port, unsigned int sigs)
{
	printk("%s\n", __func__);
}

static void stm32_uart_start_tx(struct uart_port *port)
{
	//printk("%s\n", __func__);
	STM32_USART->USART_CR1 |= (1 << 7);
}

static void stm32_uart_stop_tx(struct uart_port *port)
{
	//printk("%s\n", __func__);
	STM32_USART->USART_CR1 &= ~(1 << 7);
}

static void stm32_uart_stop_rx(struct uart_port *port)
{
	printk("%s\n", __func__);
}

static void stm32_uart_break_ctl(struct uart_port *port, int break_state)
{
	printk("%s\n", __func__);
}

static void stm32_uart_set_termios(struct uart_port *port,
				    struct ktermios *termios,
				    struct ktermios *old)
{
	printk("%s\n", __func__);
}

static void stm32_uart_config_port(struct uart_port *port, int flags)
{
	printk("%s\n", __func__);
}

static int stm32_uart_startup(struct uart_port *port)
{
	printk("%s\n", __func__);
	return 0;
}

static void stm32_uart_shutdown(struct uart_port *port)
{
	printk("%s\n", __func__);
}

static const char *stm32_uart_type(struct uart_port *port)
{
	printk("%s\n", __func__);
	return "stm32_serial";
}

static int stm32_uart_request_port(struct uart_port *port)
{
	printk("%s\n", __func__);
	return 0;
}

static void stm32_uart_release_port(struct uart_port *port)
{
	printk("%s\n", __func__);
}

static int stm32_uart_verify_port(struct uart_port *port,
				   struct serial_struct *ser)
{
	printk("%s\n", __func__);
	return 0;
}

#ifdef CONFIG_CONSOLE_POLL
static int stm32_uart_poll_get_char(struct uart_port *port)
{
	while((STM32_USART->USART_SR & USART_SR_FLAG_RXNE) == 0) {
		cpu_relax();
	}

	return STM32_USART->USART_DR;
}

static void stm32_uart_poll_put_char(struct uart_port *port, unsigned char c)
{
	while((STM32_USART->USART_SR & USART_SR_FLAG_TXE) == 0); {
		cpu_relax();
	}

	STM32_USART->USART_DR = c;
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

static struct uart_stm32_port stm32_ports[1];

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
	.major		= TTY_MAJOR,
	.minor		= 64,
	.nr		= 1,
	.cons		= &stm32_console,
};

#ifdef CONFIG_SERIAL_STM32_CONSOLE

static void stm32_console_write(struct console *co, const char *s, u_int count)
{
	int i = 0;

	//printk("%s\n", __func__);
	for(i = 0; i < count; i++) {
		while((STM32_USART->USART_SR & USART_SR_FLAG_TXE) == 0);
		STM32_USART->USART_DR = s[i];
	}
}

static int __init stm32_console_setup(struct console *co, char *options)
{
	printk("%s\n", __func__);
	return 0;
}

static void stm32_uart_rx_chars(struct uart_stm32_port *stm_port)
{
	struct uart_port *port = &stm_port->port;

	//printk("%s\n", __func__);

	u32 rxdata = STM32_USART->USART_DR;
	int flag = 0;

	port->icount.rx++;

	tty_insert_flip_char(&port->state->port,
			rxdata, flag);
}

static void stm32_uart_tx_chars(struct uart_stm32_port *stm_port)
{
	struct uart_port *port = &stm_port->port;
	struct circ_buf *xmit = &port->state->xmit;

	//printk("%s\n", __func__);
	while(!uart_circ_empty(xmit)) {
		while((STM32_USART->USART_SR & USART_SR_FLAG_TXE) == 0);

		if (port->x_char) {
			port->icount.tx++;
			STM32_USART->USART_DR = port->x_char;
			port->x_char = 0;
			continue;
		}
		if (!uart_circ_empty(xmit) && !uart_tx_stopped(port)) {
			port->icount.tx++;
			STM32_USART->USART_DR = xmit->buf[xmit->tail];
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		} else
			break;

		if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
			uart_write_wakeup(port);
	}
	stm32_uart_stop_tx(port);
}

static irqreturn_t stm32_uart_interrupt(int irq, void *data)
{
	struct uart_stm32_port *port = data;
	struct tty_port *tport = &port->port.state->port;
	int handled = IRQ_NONE;

	spin_lock(&port->port.lock);
	//printk("%s\n", __func__);

	if ((STM32_USART->USART_SR & USART_SR_FLAG_TXE) != 0) {
		stm32_uart_tx_chars(port);
		STM32_USART->USART_SR &= (~(USART_SR_FLAG_TXE));
		handled = IRQ_HANDLED;
	}

	if ((STM32_USART->USART_SR & USART_SR_FLAG_RXNE) != 0) {
		stm32_uart_rx_chars(port);
		//tty_insert_flip_char(tport, 0, TTY_OVERRUN);
		STM32_USART->USART_SR &= (~(USART_SR_FLAG_RXNE));
		handled = IRQ_HANDLED;
	}

	tty_flip_buffer_push(tport);
	spin_unlock(&port->port.lock);

	return handled;
}

static struct console stm32_console = {
	.name		= DEV_NAME,
	.write		= stm32_console_write,
	.device		= uart_console_device,
	.setup		= stm32_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &stm32_uart,
};

static int __init stm32_uart_console_init(void)
{
	printk("%s\n", __func__);
	register_console(&stm32_console);
	return 0;
}

console_initcall(stm32_uart_console_init);

#endif /* CONFIG_SERIAL_STM32_CONSOLE*/

static int stm32_serial_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	printk("%s\n", __func__);
}

static int stm32_serial_resume(struct platform_device *pdev)
{
	printk("%s\n", __func__);
}

static int stm32_serial_probe(struct platform_device *pdev)
{
	int ret;
	struct uart_stm32_port *port;

	printk("stm32 serial probe\n");

	port = &stm32_ports[0];

	port->port.dev = &pdev->dev;
	port->port.line = 0;
	port->port.type = PORT_STM32;
	port->port.fifosize = 2;
	port->port.iotype = SERIAL_IO_MEM;
	port->port.ops = &stm32_uart_ops;
	port->port.flags = UPF_BOOT_AUTOCONF;

	ret = platform_get_irq(pdev, 0);
	if (ret <= 0) {
		dev_dbg(&pdev->dev, "failed to get uart irq\n");
		return ret;
	}

	port->port.irq = ret;

	ret = request_irq(port->port.irq, stm32_uart_interrupt, 0,
			DRIVER_NAME, port);
	if (ret) {
		dev_dbg(&pdev->dev, "failed to register uart irq\n");
		return ret;
	}

	STM32_USART->USART_CR1 |= ((1 << 5));

	ret = uart_add_one_port(&stm32_uart, &port->port);

	return 0;
}

static int stm32_serial_remove(struct platform_device *pdev)
{
	printk("%s\n", __func__);
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

	printk("%s\n", __func__);

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
	printk("%s\n", __func__);
	platform_driver_unregister(&stm32_serial_driver);
	uart_unregister_driver(&stm32_uart);
}

module_init(stm32_serial_init);
module_exit(stm32_serial_exit);

MODULE_AUTHOR("Kamil Lulko");
MODULE_DESCRIPTION("STM32 serial port driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
