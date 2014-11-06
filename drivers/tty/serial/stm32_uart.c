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
	.driver_name	= "stm32_serial",
	.dev_name	= "ttyS",
	.major		= TTY_MAJOR,
	.minor		= 64,
	.nr		= 1,
	.cons		= &stm32_console,
};

#ifdef CONFIG_SERIAL_STM32_CONSOLE

static void stm32_console_write(struct console *co, const char *s, u_int count)
{
	int i = 0;
	for(i = 0; i < count; i++) {
		while((STM32_USART->USART_SR & USART_SR_FLAG_TXE) == 0);
		STM32_USART->USART_DR = s[i];
	}
}

static int __init stm32_console_setup(struct console *co, char *options)
{
	return 0;
}

static struct console stm32_console = {
	.name		= "ttyS",
	.write		= stm32_console_write,
	.device		= uart_console_device,
	.setup		= stm32_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &stm32_uart,
};

/*
 * Early console initialization (before VM subsystem initialized).
 */
static int __init stm32_console_init(void)
{
	int ret;

	add_preferred_console("ttyS", 0, NULL);
	register_console(&stm32_console);

	return 0;
}

console_initcall(stm32_console_init);

/*
 * Late console initialization.
 */
static int __init stm32_late_console_init(void)
{
	register_console(&stm32_console);

	return 0;
}

core_initcall(stm32_late_console_init);

#endif /* CONFIG_SERIAL_STM32_CONSOLE*/

static int stm32_serial_suspend(struct platform_device *pdev,
				pm_message_t state)
{

}

static int stm32_serial_resume(struct platform_device *pdev)
{

}

static int stm32_serial_probe(struct platform_device *pdev)
{
	int ret;
	struct uart_stm32_port *port;

	printk("stm32 serial probe\n");

	port = &stm32_ports[0];

	ret = uart_add_one_port(&stm32_uart, &port->port);

	return 0;
}

static int stm32_serial_remove(struct platform_device *pdev)
{

}

static struct platform_driver stm32_serial_driver = {
	.probe		= stm32_serial_probe,
	.remove		= stm32_serial_remove,
	.suspend	= stm32_serial_suspend,
	.resume		= stm32_serial_resume,
	.driver		= {
		.name	= "stm32_usart",
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
MODULE_ALIAS("platform:stm32_usart");
