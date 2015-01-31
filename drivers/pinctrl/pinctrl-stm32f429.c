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

#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinctrl.h>

#include "stm32-pincfg.h"

static struct of_device_id stm32_pinctrl_of_match[] = {
	{ .compatible = "stm32,stm32-pinctrl", .data = NULL},//&stm32f429_ops},
	{ /* sentinel */ }
};

static int stm32_pinctrl_probe(struct platform_device *pdev)
{
	//return imx1_pinctrl_core_probe(pdev, &stm32_pinctrl_info);
	printk("%s: \n",__func__);
	return 0;
}

static int stm32_pinctrl_remove(struct platform_device *pdev)
{
	//return imx1_pinctrl_core_remove(pdev, &stm32_pinctrl_info);
	printk("%s: \n",__func__);
	return 0;
}

static struct platform_driver stm32_pinctrl_driver = {
	.driver = {
		.name = "stm32-pinctrl",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(stm32_pinctrl_of_match),
	},
	.probe = stm32_pinctrl_probe,
	.remove = stm32_pinctrl_remove,
};

static int __init stm32_pinctrl_init(void)
{
	printk("%s: \n",__func__);
	return platform_driver_register(&stm32_pinctrl_driver);
}
arch_initcall(stm32_pinctrl_init);

static void __exit stm32_pinctrl_exit(void)
{
	printk("%s: \n",__func__);
	platform_driver_unregister(&stm32_pinctrl_driver);
}
module_exit(stm32_pinctrl_exit);
MODULE_AUTHOR("Rafal Fabich <rafal.fabich@gmail.com>");
MODULE_DESCRIPTION("STM32F429 pinctrl driver");
MODULE_LICENSE("GPL v2");
