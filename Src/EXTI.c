/*
 * EXTI.c
 *
 *  Created on: 5 Ağu 2021
 *      Author: Rifat Anasız
 */


#include "EXTI.h"

/**
  * @brief  EXTI_Init for valid GPIO port and Line Number
  * @param  EXTI_InitStruct= user config structer
  *
  * @retval Void
  */
void EXTI_Init(EXTI_InitTypeDef_t *EXTI_InitStruct)
{
	uint32_t tempValue = 0 ;

	tempValue=(uint32_t)EXTI_BASE_ADDR;

	EXTI->IMR &= ~(0x1U <<EXTI_InitStruct->EXTI_LineNumber);
	EXTI->EMR &= ~(0x1U <<EXTI_InitStruct->EXTI_LineNumber);

	if(EXTI_InitStruct->EXTI_LineCmd != DISABLE)
	{
		tempValue += EXTI_InitStruct->EXTI_Mode;

		*((__IO uint32_t *)tempValue) |= (0x1U << EXTI_InitStruct->EXTI_LineNumber);

		tempValue= (uint32_t)EXTI_BASE_ADDR;
		EXTI->RTSR &= ~(0x1U <<EXTI_InitStruct->EXTI_LineNumber);
		EXTI->FTSR &= ~(0x1U <<EXTI_InitStruct->EXTI_LineNumber);

		if(EXTI_InitStruct->TriggerSelection==EXTI_Trigger_RF)
		{
			EXTI->RTSR |= (0x1U <<EXTI_InitStruct->EXTI_LineNumber);
			EXTI->FTSR |= (0x1U <<EXTI_InitStruct->EXTI_LineNumber);

		}
		else
		{
			tempValue += EXTI_InitStruct->TriggerSelection;

			*((__IO uint32_t *)tempValue) |= (0x1U << EXTI_InitStruct->EXTI_LineNumber);

		}

	}
	else
	{
		tempValue= (uint32_t)EXTI_BASE_ADDR;

		tempValue += EXTI_InitStruct->EXTI_Mode;

		*((__IO uint32_t *)tempValue) &= ~(0x1U << EXTI_InitStruct->EXTI_LineNumber);
	}

}


/**
  * @brief  EXTI_LineConfig = Configures the port and pin for SYSCFG
  * @param  PortSource= Port value A-F @def_group PORT_Values
  *
  * @param  XTI_LineSource = Pin numbers & Line numbers @def_group EXTI_Line_Values
  *
  * @retval Void
  */
void EXTI_LineConfig(uint8_t PortSource,uint8_t EXTI_LineSource )
{
	uint32_t tempValue;

	tempValue = SYSCFG->EXTICR[EXTI_LineSource>>2U];
	tempValue &=~(0xFU<< (EXTI_LineSource & 0x3U) * 4);
	tempValue=(PortSource<<(EXTI_LineSource& 0x3U)* 4);
	SYSCFG->EXTICR[EXTI_LineSource>>2U]=tempValue;

}

/**
  * @brief  NVIC_EnableInterrupt
  *
  * @param  IRQNumber= IRQ number of line
  * @retval Void
  */
void NVIC_EnableInterrupt(IRQNumber_Type_t IRQNumber)
{
	uint32_t tempValue= 0;

	tempValue=*((IRQNumber >> 5U) + NVIC_ISER0);
	tempValue &=~(0x1U <<(IRQNumber & 0x1FU ) );
	tempValue |= (0x1U <<(IRQNumber & 0x1FU ) );
	*((IRQNumber >> 5U) + NVIC_ISER0)=tempValue;

}
