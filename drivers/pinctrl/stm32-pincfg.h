/*
 * stm32-pincfg.h
 *
 *  Created on: Jan 31, 2015
 *      Author: rafalf
 */

#ifndef STM32_PINCFG_H_
#define STM32_PINCFG_H_


#define STM32_GPIO_CONTROLLER_MEM_SIZE	0x3FF

struct stm32_gpio_regs {
	unsigned int stm32_gpio_moder;
	unsigned int stm32_gpio_otyper;
	unsigned int stm32_gpio_ospeedr;
	unsigned int stm32_gpio_pupdr;
	unsigned int stm32_gpio_idr;
	unsigned int stm32_gpio_odr;
	unsigned int stm32_gpio_bssr;
	unsigned int stm32_gpio_lckr;
	unsigned int stm32_gpio_afrl;
	unsigned int stm32_gpio_afrh;
	/* keep the rest of the controller memory reserved */
	unsigned char reserved[STM32_GPIO_CONTROLLER_MEM_SIZE-(10*sizeof(unsigned int))];
};

#define STM32_GPIO_MODER_IN	0
#define STM32_GPIO_MODER_OUT	1
#define STM32_GPIO_MODER_AF	2
#define STM32_GPIO_MODER_ANALOG	3
#define STM32_GPIO_MODER_MASK	3

enum stm32_pin_moder{
	GPIO_MODER_IN,
	GPIO_MODER_OUT,
	GPIO_MODER_AF,
	GPIO_MODER_ANALOG,
};

#define STM32_GPIO_OTYPER_PP	0
#define STM32_GPIO_OTYPER_OD	1
#define STM32_GPIO_OTYPER_MASK	1

enum stm32_pin_typer{
	GPIO_OTYPER_PP,
	GPIO_OTYPER_OD,
};

#define SMT32_GPIO_OSPEEDR_LS	0
#define STM32_GPIO_OSPEEDR_MS	1
#define STM32_GPIO_OSPEEDR_FS	2
//#define STM32_GPIO_OSPEEDR_LS	3
#define STM32_GPIO_OSPEEDR_MASK	3

enum stm32_pin_ospeedr{
	GPIO_OSPEEDR_LS,
	GPIO_OSPEEDR_MS,
	GPIO_OSPEEDR_FS,
};

#define STM32_GPIO_PUPDR_NONE	0
#define	STM32_GPIO_PUPDR_PUP	1
#define	STM32_GPIO_PUPDR_PDOWN	2
#define	STM32_GPIO_PUPDR_MASK	3

enum stm32_pin_pupdr{
	GPIO_PUPDR_NONE,
	GPIO_PUPDR_PUP,
	GPIO_PUPDR_PDOWN,
};

#define STM32_GPIO_PIN_CONFIG(m,t,sp,pu) ((m<<5)|(t<<4)|(sp<<2)|(pu))
#define STM32_GPIO_PIN_GET_MODER(v) ((v>>5)&STM32_GPIO_MODER_MASK)
#define STM32_GPIO_PIN_GET_OTYPER(v) ((v>>4)&STM32_GPIO_OTYPER_MASK)
#define STM32_GPIO_PIN_GET_OSPEEDR(v) ((v>>2)&STM32_GPIO_OSPEEDR_MASK)
#define STM32_GPIO_PIN_GET_PUPDR(v) ((v)&STM32_GPIO_PUPDR_MASK)


#define STM32_GPIO_IDR	0
#define STM32_GPIO_ODR	0
#define STM32_GPIO_BSSR_PIN_RESET(n)	((0x1<<(n))<<16)
#define STM32_GPIO_BSSR_PIN_SET(n)	(0x1<<(n))
#define STM32_GPIO_LCKR_LCKK	(0x1<<15)
#define	STM32_GPIO_LCKR_PIN_LOCK(n) (0x1<<(n))


enum stm32_port {
	GPIO_PORT_A,
	GPIO_PORT_B,
	GPIO_PORT_C,
	GPIO_PORT_D,
	GPIO_PORT_E,
	GPIO_PORT_F,
	GPIO_PORT_G,
	GPIO_PORT_H,
	GPIO_PORT_I,
	GPIO_PORT_J,
	GPIO_PORT_K,
};

enum stm32_pin{
	GPIO_PIN_0,
	GPIO_PIN_1,
	GPIO_PIN_2,
	GPIO_PIN_3,
	GPIO_PIN_4,
	GPIO_PIN_5,
	GPIO_PIN_6,
	GPIO_PIN_7,
	GPIO_PIN_8,
	GPIO_PIN_9,
	GPIO_PIN_10,
	GPIO_PIN_11,
	GPIO_PIN_12,
	GPIO_PIN_13,
	GPIO_PIN_14,
	GPIO_PIN_15,
};

enum stm32_pin_func{
	GPIO_ALT_SYSTEM,
	GPIO_ALT_TIM_1_2,
	GPIO_ALT_TIM_3_5,
	GPIO_ALT_TIM_8_11,
	GPIO_ALT_I2C_1_3,
	GPIO_ALT_SPI_1_6,
	GPIO_ALT_SPI_2_3_SAI_1,
	GPIO_ALT_USART_1_3,
	GPIO_ALT_USART_4_8,
	GPIO_ALT_CAN_TIM_12_14,
	GPIO_ALT_OTG_FS_HS,
	GPIO_ALT_ETM,
	GPIO_ALT_SDIO_OTG_HS,
	GPIO_ALT_DCMI,
	GPIO_ALT_LTDC,
	GPIO_ALT_EVENTOUT,
};



#endif /* STM32_PINCFG_H_ */
