/*
 * GPIO.c
 *
 *  Created on: Jul 29, 2021
 *      Author: Rifat Anasýz
 */

#include "GPIO.h"
/**
  * @brief  GPIO_Init = Configures the port and pin
  * @param  GPIOx= GPIO port Base Address
  *
  * @param  GPIO_InitTypeDef_t = users config structures
  *
  * @retval Void
  */

void GPIO_Init( GPIO_TypeDef_t *GPIOx, GPIO_InitTypeDef_t *GPIO_ConfigStruct)
{

	uint32_t position;
	uint32_t fakePosition=0;
	uint32_t lastPosition=0;

	for(position=0;position<16;position++)
	{
		fakePosition=(0x1<<position);
		lastPosition=(uint32_t)(GPIO_ConfigStruct->pinNumber) & fakePosition;

		if(fakePosition == lastPosition)
		{
			/* Mode Config */
			uint32_t tempValue = GPIOx -> MODER;

			tempValue &= ~(0x3U << (position*2));
			tempValue |= (GPIO_ConfigStruct->Mode << (position*2) );

			GPIOx -> MODER = tempValue;

			if(GPIO_ConfigStruct->Mode == GPIO_MODE_OUTPUT || GPIO_ConfigStruct -> Mode == GPIO_MODE_AF)
			{
				/*Output type Config*/
				tempValue = GPIOx->OTYPER;
				tempValue &= ~(0x1U<<position);
				tempValue |= (GPIO_ConfigStruct->Otype<<position);
				GPIOx->OTYPER=tempValue;

				/* Output speed Config*/
				tempValue = GPIOx->OSPEEDR;
				tempValue &= ~(0x3U << (position*2));
				tempValue |= (GPIO_ConfigStruct->speed << (position*2) );
				GPIOx->OSPEEDR = tempValue;
			}
			/* Push Pull Config*/
			tempValue = GPIOx->PUPDR;
			tempValue &= ~(0x3U << (position*2));
			tempValue |= (GPIO_ConfigStruct->PuPd << (position*2) );
			GPIOx->PUPDR=tempValue;

		}


	}





}

/**
  * @brief  GPIO_Write_Pin = makes pin High or Low
  * @param  GPIOx= GPIO port Base Address
  *
  * @param  pinNumber=GPIO_PIN_x    numbers 0 - 15
  *
  * @param  pinState = GPIO_Pin_Set  or GPIO_Pin_Reset
  * @retval Void
  */

void GPIO_WritePin( GPIO_TypeDef_t *GPIOx, uint16_t pinNumber, GPIO_PinState_t pinState )
{
	if (pinState == GPIO_Pin_Set)
	{
		GPIOx->BSRR = pinNumber;
	}
	else
	{
		GPIOx->BSRR = (pinNumber << 16U );
	}
}

/**
  * @brief  GPIO_ReadPin = reads the pin of GPIO Port
  * @param  GPIOx= GPIO port Base Address
  *
  * @param  pinNumber=GPIO_PIN_x    numbers 0 - 15
  *
  * @retval GPIO_PinState_t
  */

GPIO_PinState_t GPIO_ReadPin(GPIO_TypeDef_t *GPIOx, uint16_t pinNumber)
{
	GPIO_PinState_t bitStatus = GPIO_Pin_Reset;

	if ((GPIOx->IDR & pinNumber) != GPIO_Pin_Reset)
	{
		bitStatus = GPIO_Pin_Set;
	}

	return bitStatus;
}

/**
  * @brief  GPIO_LockPin = Lock the pin of GPIO Port
  * @param  GPIOx= GPIO port Base Address
  *
  * @param  pinNumber=GPIO_PIN_x    numbers 0 - 15
  *
  * @retval void
  */

void GPIO_LockPin(GPIO_TypeDef_t *GPIOx,uint16_t pinNumber )
{
	uint32_t tempValue = (0x1U << 16U ) | pinNumber;

	GPIOx->LCKR = tempValue;    // LCKR [16] = '1'  LCKR[15:0] =Data
	GPIOx->LCKR = pinNumber;	// LCKR [16] = '0'  LCKR[15:0] =Data
	GPIOx->LCKR = tempValue;    // LCKR [16] = '1'  LCKR[15:0] =Data
	tempValue = GPIOx->LCKR;    // Read lock register
}

/**
  * @brief  GPIO_tOGGLE_Pin = TOGGLE the pin of GPIO Port
  * @param  GPIOx= GPIO port Base Address
  *
  * @param  pinNumber=GPIO_PIN_x    numbers 0 - 15
  *
  * @retval void
  */
void GPIO_TogglePin(GPIO_TypeDef_t *GPIOx,uint16_t pinNumber)
{
	uint32_t tempODRRegister = GPIOx->ODR;
	GPIOx->BSRR = ((tempODRRegister & pinNumber) << 16U) | (~tempODRRegister & pinNumber);

}

