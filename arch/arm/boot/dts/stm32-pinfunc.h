/*
 * stm32-pincfg.h
 *
 *  Created on: Jan 31, 2015
 *      Author: rafalf
 */

#define GPIO_PORT_A	0
#define GPIO_PORT_B	1
#define GPIO_PORT_C	2
#define GPIO_PORT_D	3
#define GPIO_PORT_E	4
#define GPIO_PORT_F	5
#define GPIO_PORT_G	6
#define GPIO_PORT_H	7
#define GPIO_PORT_I	8
#define GPIO_PORT_J	9
#define GPIO_PORT_K	10

#define GPIO_PIN_0	0
#define GPIO_PIN_1	1
#define GPIO_PIN_2	2
#define GPIO_PIN_3	3
#define GPIO_PIN_4	4
#define GPIO_PIN_5	5
#define GPIO_PIN_6	6
#define GPIO_PIN_7	7
#define GPIO_PIN_8	8
#define GPIO_PIN_9	9
#define GPIO_PIN_10	10
#define GPIO_PIN_11	11
#define GPIO_PIN_12	12
#define GPIO_PIN_13	13
#define GPIO_PIN_14	14
#define GPIO_PIN_15	15

#define EXTI_IRQ_OFF	0
#define EXTI_IRQ_ON	1

#define EXTI_0	0
#define EXTI_1	1
#define EXTI_2	2
#define EXTI_3	3
#define EXTI_4	4
#define EXTI_5	5
#define EXTI_6	6
#define EXTI_7	7
#define EXTI_8	8
#define EXTI_9	9
#define EXTI_10	10
#define EXTI_11	11
#define EXTI_12	12
#define EXTI_13	13
#define EXTI_14	14
#define EXTI_15	15

#define GPIO_ALT_SYSTEM		0
#define GPIO_ALT_TIM_1_2	1
#define GPIO_ALT_TIM_3_5	2
#define GPIO_ALT_TIM_8_11	3
#define GPIO_ALT_I2C_1_3	4
#define GPIO_ALT_SPI_1_6	5
#define GPIO_ALT_SPI_2_3_SAI_1	6
#define GPIO_ALT_USART_1_3	7
#define GPIO_ALT_USART_4_8	8
#define GPIO_ALT_CAN_TIM_12_14	9
#define GPIO_ALT_OTG_FS_HS	10
#define GPIO_ALT_ETM		11
#define GPIO_ALT_SDIO_OTG_HS	12
#define GPIO_ALT_DCMI		13
#define GPIO_ALT_LTDC		14
#define GPIO_ALT_EVENTOUT	15

#define GPIO_MODER_IN	0
#define GPIO_MODER_OUT	1
#define GPIO_MODER_AF	2
#define GPIO_MODER_ANALOG	3
#define GPIO_MODER_MASK	3

#define GPIO_OTYPER_PP	0
#define GPIO_OTYPER_OD	1
#define GPIO_OTYPER_MASK	1

#define GPIO_OSPEEDR_LS	0
#define GPIO_OSPEEDR_MS	1
#define GPIO_OSPEEDR_FS	2
//#define GPIO_OSPEEDR_LS	3
#define GPIO_OSPEEDR_MASK	3

#define GPIO_PUPDR_NONE	0
#define	GPIO_PUPDR_PUP	1
#define	GPIO_PUPDR_PDOWN	2
#define	GPIO_PUPDR_MASK	3

#define GPIO_PIN_CONFIG(i,m,t,sp,pu) ((i<<11)|(m<<5)|(t<<4)|(sp<<2)|(pu))
#define PIO_PULL_NONE	0
#define PIO_PULL_UP	1
#define PIO_PULL_DOWN	2
#define PIO_PUSHPULL	0
#define PIO_OPENDRAIN	1
/* PIO config set: <PORT PIN MODE TYPE SPEED PULLUP>
*/
#define STM32_GPIO_IDR	0
#define STM32_GPIO_ODR	0
#define STM32_GPIO_BSSR_PIN_RESET(n)	((0x1<<(n))<<16)
#define STM32_GPIO_BSSR_PIN_SET(n)	(0x1<<(n))
#define STM32_GPIO_LCKR_LCKK	(0x1<<15)
#define	STM32_GPIO_LCKR_PIN_LOCK(n) (0x1<<(n))

