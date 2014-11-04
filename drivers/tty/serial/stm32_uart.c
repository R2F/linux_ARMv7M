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
#include <linux/atmel_pdc.h>
#include <linux/atmel_serial.h>
#include <linux/uaccess.h>
#include <linux/platform_data/atmel.h>
#include <linux/timer.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/err.h>
#include <linux/irq.h>

#include <asm/io.h>
#include <asm/ioctls.h>

#include <linux/serial_core.h>

#define ATMEL_CONSOLE_DEVICE	NULL

static struct uart_driver stm32_uart = {
	.owner		= THIS_MODULE,
	.driver_name	= "stm32_serial",
	.dev_name	= "ttyS",
	.major		= TTY_MAJOR,
	.minor		= 64,
	.nr		= 1,
	.cons		= ATMEL_CONSOLE_DEVICE,
};

static int stm32_serial_suspend(struct platform_device *pdev,
				pm_message_t state)
{

}

static int stm32_serial_resume(struct platform_device *pdev)
{

}

static int stm32_serial_probe(struct platform_device *pdev)
{

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
