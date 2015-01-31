/*
 * stm32-pincfg.h
 *
 *  Created on: Jan 31, 2015
 *      Author: rafalf
 */

#ifndef STM32_PINFUNC_H_
#define STM32_PINFUNC_H_

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


#endif /* STM32_PINFUNC_H_ */
