/*
 * SPI.h
 *
 *  Created on: 19 Ağu 2021
 *      Author: Rifat Anasız
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_

#include "stm32f051xx.h"
/*
 * @def_group SPI_Baundrates
 */
#define SPI_BAUDRATE_DIV2		((uint32_t)(0x00000000))
#define SPI_BAUDRATE_DIV4		((uint32_t)(0x00000008))
#define SPI_BAUDRATE_DIV8		((uint32_t)(0x00000010))
#define SPI_BAUDRATE_DIV16		((uint32_t)(0x00000018))
#define SPI_BAUDRATE_DIV32		((uint32_t)(0x00000020))
#define SPI_BAUDRATE_DIV64		((uint32_t)(0x00000028))
#define SPI_BAUDRATE_DIV128		((uint32_t)(0x00000030))
#define SPI_BAUDRATE_DIV256		((uint32_t)(0x00000038))

/*
 * @def_group CPHA_Values
 */
#define SPI_CPHA_FIRST			((uint32_t)(0x00000000))
#define SPI_CPHA_SECOND			((uint32_t)(0x00000001))
/*
 * @def_group CPOL_Values
 */
#define SPI_CPOL_LOW			((uint32_t)(0x00000000))
#define SPI_CPOL_HIGH			((uint32_t)(0x00000002))
/*
 * @def_group DFF_Values
 */
#define SPI_DFF_8BIT		((uint32_t)(0x00000000))
#define SPI_DFF_16BIT		((uint32_t)(0x00000000))
#define SPI_DFF_8BIT		((uint32_t)(0x00000000))

/*
 * @def_group Mode
 */
#define SPI_MODE_MASTER			((uint32_t)(0x00000004))
#define SPI_MODE_SLAVE			((uint32_t)(0x00000000))

/*
 * @def_group _SPI_FREME_FORMAT
 */
#define SPI_FRAMEFORMAT_MSB			((uint32_t)(0x00000000))
#define SPI_FRAMEFORMAT_LSB			((uint32_t)(0x00000080))
typedef struct
{
	uint32_t Mode;		/*!  Mode for SPI @def_group Mode      */
	uint32_t CPHA;		 /*!  CPHA for SPI @def_group CPHA_Values      */
	uint32_t CPOL;		/*!  CPOL for SPI @def_group CPOL_Values      */
	uint32_t BaundRate;  /*!  Baundrates for SPI @def_group SPI_Baundrates      */
	uint32_t SSM_Cmd;
	uint32_t DFF_Format;/*! ???? DFF for SPI @def_group DFF_Values      */
	uint32_t CRCEN;
	uint32_t BusConfig;
	uint32_t FrameFormat;/*!  FRAME FORMAT  for SPI @def_group _SPI_FREME_FORMAT      */

}SPI_InitTypeDef_t;


typedef struct
{
	SPI_TypeDef_t *Instance;
	SPI_InitTypeDef_t Init;

}SPI_HandleTypeDef_t;

void SPI_Init(SPI_HandleTypeDef_t *SPI_Handle);

#endif /* INC_SPI_H_ */
