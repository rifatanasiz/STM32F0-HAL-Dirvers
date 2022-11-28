/*
 * SPI.c
 *
 *  Created on: 19 Ağu 2021
 *      Author: Rifat Anasız
 */

#include "SPI.h"

void SPI_Init(SPI_HandleTypeDef_t *SPI_Handle)
{
	uint32_t tempValue = 0;

	tempValue=SPI_Handle->Instance->CR1;

	tempValue |= (SPI_Handle->Init.BaundRate) | (SPI_Handle->Init.CPHA) | (SPI_Handle->Init.CPOL)| (SPI_Handle->Init.Mode)| (SPI_Handle->Init.FrameFormat);


}
