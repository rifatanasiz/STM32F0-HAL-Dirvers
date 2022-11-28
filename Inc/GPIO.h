/*
 * GPIO.h
 *
 *  Created on: Jul 29, 2021
 *      Author: Rifat Anasýz
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include "stm32f051xx.h"

/*
 * @def_group GPIO_Pins
 *
 */

#define GPIO_PIN_0        (uint16_t)(0x0001)//0000 0000 0000 0001   GPIO Pin 0 Selected
#define GPIO_PIN_1        (uint16_t)(0x0002)//0000 0000 0000 0010	GPIO Pin 1 Selected
#define GPIO_PIN_2        (uint16_t)(0x0004)//0000 0000 0000 0100	GPIO Pin 2 Selected
#define GPIO_PIN_3        (uint16_t)(0x0008)//0000 0000 0000 1000
#define GPIO_PIN_4        (uint16_t)(0x0010)//0000 0000 0001 0000
#define GPIO_PIN_5        (uint16_t)(0x0020)//0000 0000 0010 0000
#define GPIO_PIN_6        (uint16_t)(0x0040)//0000 0000 0010 0000
#define GPIO_PIN_7        (uint16_t)(0x0080)//0000 0000 0010 0000
#define GPIO_PIN_8        (uint16_t)(0x0100)//0000 0000 0010 0000
#define GPIO_PIN_9        (uint16_t)(0x0200)//0000 0000 0010 0000
#define GPIO_PIN_10       (uint16_t)(0x0400)//0000 0000 0010 0000
#define GPIO_PIN_11       (uint16_t)(0x0800)//0000 0000 0010 0000
#define GPIO_PIN_12       (uint16_t)(0x1000)//0000 0000 0010 0000
#define GPIO_PIN_13       (uint16_t)(0x2000)//0000 0000 0010 0000
#define GPIO_PIN_14       (uint16_t)(0x4000)//0000 0000 0010 0000
#define GPIO_PIN_15       (uint16_t)(0x8000)//0000 0000 0010 0000
#define GPIO_PIN_All      (uint16_t)(0xFFFF)//0000 0000 0010 0000	GPIO Pin All Selected

/*
 * @def_group GPIO_Pin_Modes
 *
 */
#define GPIO_MODE_INPUT		(0x0U)
#define GPIO_MODE_OUTPUT	(0x1U)
#define GPIO_MODE_AF		(0x2U)
#define GPIO_MODE_ANALOG	(0x3U)

/*
 * @def_group GPIO_Otype_Modes
 *
 */
#define GPIO_OTYPE_PP		(0x0U)
#define GPIO_OTYPE_OD		(0x1U)

/*
 * @def_group GPIO_Speed_Modes
 *
 */
#define GPIO_SPEED_LOW  	(0x0U)
#define GPIO_SPEED_MEDIUM	(0x1U)
#define GPIO_SPEED_HIGH  	(0x2U)
#define GPIO_SPEED_VERY  	(0x3U)

/*
 * @def_group GPIO_PuPd_Modes
 *
 */
#define GPIO_PUPD_NOPULL     (0X0U)
#define GPIO_PUPD_PULLUP     (0X1U)
#define GPIO_PUPD_PULLDOWN   (0X2U)


typedef enum
{
	GPIO_Pin_Reset = 0x0U,
	GPIO_Pin_Set = !GPIO_Pin_Reset

}GPIO_PinState_t;

typedef struct
{
	uint32_t pinNumber;  /* GPIO Pin Numbers @def_group GPIO_Pins */
	uint32_t Mode;		 /* GPIO Pin Numbers @def_group GPIO_Pin_Modes */
	uint32_t Otype;		 /* GPIO Pin Numbers @def_group GPIO_Otype_Modes */
	uint32_t speed;		 /* GPIO Pin Numbers @def_group GPIO_Speed_Modes */
	uint32_t PuPd;		 /* GPIO Pin Numbers @def_group GPIO_PuPd_Modes */
	uint32_t Alternate;

}GPIO_InitTypeDef_t;

void GPIO_Init( GPIO_TypeDef_t *GPIOx, GPIO_InitTypeDef_t *GPIO_ConfigStruct);

void GPIO_WritePin( GPIO_TypeDef_t *GPIOx, uint16_t pinNumber, GPIO_PinState_t pinState );

GPIO_PinState_t GPIO_ReadPin(GPIO_TypeDef_t *GPIOx, uint16_t pinNumber);

void GPIO_LockPin(GPIO_TypeDef_t *GPIOx,uint16_t pinNumber );

void GPIO_TogglePin(GPIO_TypeDef_t *GPIOx,uint16_t pinNumber);


#endif /* INC_GPIO_H_ */
