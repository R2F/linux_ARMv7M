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

#define STM32_GPIO_OTYPER_PP	0
#define STM32_GPIO_OTYPER_OD	1
#define STM32_GPIO_OTYPER_MASK	1

#define SMT32_GPIO_OSPEEDR_LS	0
#define STM32_GPIO_OSPEEDR_MS	1
#define STM32_GPIO_OSPEEDR_FS	2
#define STM32_GPIO_OSPEEDR_LS	3
#define STM32_GPIO_OSPEEDR_MASK	3

#define STM32_GPIO_PUPDR_NONE	0
#define	STM32_GPIO_PUPDR_PUP	1
#define	STM32_GPIO_PUPDR_PDOWN	2
#define	STM32_GPIO_PUPDR_MASK	3

#define STM32_GPIO_PIN_CONFIG(m,t,sp,pu) ((m<<5)|(t<<4)|(sp<<2)|(pu))

#define STM32_GPIO_IDR	0
#define STM32_GPIO_ODR	0
#define STM32_GPIO_BSSR_PIN_RESET(n)	((0x1<<(n))<<16)
#define STM32_GPIO_BSSR_PIN_SET(n)	(0x1<<(n))
#define STM32_GPIO_LCKR_LCKK	(0x1<<15)
#define	STM32_GPIO_LCKR_PIN_LOCK(n) (0x1<<(n))


enum stm32_port {
	PORTA,
	PORTB,
	PORTC,
	PORTD,
	PORTE,
	PORTF,
	PORTG,
	PORTH,
	PORTI,
	PORTJ,
	PORTK,
};

enum stm32_pin{
	PIN0,
	PIN1,
	PIN2,
	PIN3,
	PIN4,
	PIN5,
	PIN6,
	PIN7,
	PIN8,
	PIN9,
	PIN10,
	PIN11,
	PIN12,
	PIN13,
	PIN14,
	PIN15,
};

enum stm32_pin_func{
	AF0,
	AF1,
	AF2,
	AF3,
	AF4,
	AF5,
	AF6,
	AF7,
	AF8,
	AF9,
	AF10,
	AF11,
	AF12,
	AF13,
	AF14,
	AF15,
};



#endif /* STM32_PINCFG_H_ */
