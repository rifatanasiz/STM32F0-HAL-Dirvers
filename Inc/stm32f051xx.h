/*
 * stm32f051xx.h
 *
 *  Created on: Jul 28, 2021
 *      Author: Rifat Anasiz
 */

#ifndef INC_STM32F051XX_H_
#define INC_STM32F051XX_H_

#include <stdint.h>
#include <string.h>
/*
 *
 * Microprocessor Defines
 */
#define NVIC_ISER0			( (uint32_t*)(0xE000E100UL) )

#define __IO volatile

#define SET_BIT(REG, BIT)					((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)					( (REG) &= ~(BIT) )
#define READ_BIT(REG, BIT)					( (REG) & (BIT) )
#define UNUSED(x)            				(void)x

typedef enum
{
	DISABLE = 0x0U,
	ENABLE=!DISABLE
}FunctionalState_t;


/*!< Interrupt Number Definition */
typedef enum
{
/******  Cortex-M0 Processor Exceptions Numbers **************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                        */
  HardFault_IRQn              = -13,    /*!< 3 Cortex-M0 Hard Fault Interrupt                                */
  SVC_IRQn                    = -5,     /*!< 11 Cortex-M0 SV Call Interrupt                                  */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M0 Pend SV Interrupt                                  */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M0 System Tick Interrupt                              */

/******  STM32F0 specific Interrupt Numbers ******************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                               */
  PVD_IRQn                    = 1,      /*!< PVD Interrupt through EXTI Lines 16                             */
  RTC_IRQn                    = 2,      /*!< RTC Interrupt through EXTI Lines 17, 19 and 20                  */
  FLASH_IRQn                  = 3,      /*!< FLASH global Interrupt                                          */
  RCC_IRQn                    = 4,      /*!< RCC global Interrupt                                            */
  EXTI0_1_IRQn                = 5,      /*!< EXTI Line 0 and 1 Interrupt                                     */
  EXTI2_3_IRQn                = 6,      /*!< EXTI Line 2 and 3 Interrupt                                     */
  EXTI4_15_IRQn               = 7,      /*!< EXTI Line 4 to 15 Interrupt                                     */
  TSC_IRQn                    = 8,      /*!< Touch Sensing Controller Interrupts                             */
  DMA1_Channel1_IRQn          = 9,      /*!< DMA1 Channel 1 Interrupt                                        */
  DMA1_Channel2_3_IRQn        = 10,     /*!< DMA1 Channel 2 and Channel 3 Interrupt                          */
  DMA1_Channel4_5_IRQn        = 11,     /*!< DMA1 Channel 4 and Channel 5 Interrupt                          */
  ADC1_COMP_IRQn              = 12,     /*!< ADC1 and COMP interrupts (ADC interrupt combined with EXTI Lines 21 and 22 */
  TIM1_BRK_UP_TRG_COM_IRQn    = 13,     /*!< TIM1 Break, Update, Trigger and Commutation Interrupt           */
  TIM1_CC_IRQn                = 14,     /*!< TIM1 Capture Compare Interrupt                                  */
  TIM2_IRQn                   = 15,     /*!< TIM2 global Interrupt                                           */
  TIM3_IRQn                   = 16,     /*!< TIM3 global Interrupt                                           */
  TIM6_DAC_IRQn               = 17,     /*!< TIM6 global and DAC channel underrun error Interrupt            */
  TIM14_IRQn                  = 19,     /*!< TIM14 global Interrupt                                          */
  TIM15_IRQn                  = 20,     /*!< TIM15 global Interrupt                                          */
  TIM16_IRQn                  = 21,     /*!< TIM16 global Interrupt                                          */
  TIM17_IRQn                  = 22,     /*!< TIM17 global Interrupt                                          */
  I2C1_IRQn                   = 23,     /*!< I2C1 Event Interrupt & EXTI Line23 Interrupt (I2C1 wakeup)      */
  I2C2_IRQn                   = 24,     /*!< I2C2 Event Interrupt                                            */
  SPI1_IRQn                   = 25,     /*!< SPI1 global Interrupt                                           */
  SPI2_IRQn                   = 26,     /*!< SPI2 global Interrupt                                           */
  USART1_IRQn                 = 27,     /*!< USART1 global Interrupt & EXTI Line25 Interrupt (USART1 wakeup) */
  USART2_IRQn                 = 28,     /*!< USART2 global Interrupt                                         */
  CEC_CAN_IRQn                = 30      /*!< CEC and CAN global Interrupts & EXTI Line27 Interrupt           */
} IRQNumber_Type_t;






/*
 * Memory Base Address
 *
 */

#define FLASH_BASE_ADDR						(0x08000000UL) /* Flash Base Address (up to 256 KB ) 	*/
#define SRAM1_BASE_ADDR						(0x20000000UL) /* Sram1 Base Address  8 KB 			    */


/*
 * Peripheral Base Addresses
 *
 */

#define PERIPH_BASE_ADDR					(0x40000000UL)					 /* Base Address for All peripherals       */

#define APB_BASE_ADDR						PERIPH_BASE_ADDR				 /* APB Bus Domain Address 				   */
#define AHB1_BASE_ADDR      			   (PERIPH_BASE_ADDR + 0x00020000UL) /* AHB1 Bus Domain Address 			   */
#define AHB2_BASE_ADDR          	       (PERIPH_BASE_ADDR + 0x08000000UL) /* AHB2 Bus Domain Address 			   */


/*
 * APB Peripheral Base Addresses
 *
 */

#define TIM2_BASE_ADDR				          (APB_BASE_ADDR + 0x0000UL)		 /* Timer 2 Base Address 			*/
#define TIM3_BASE_ADDR				          (APB_BASE_ADDR + 0x0400UL)	     /* Timer 3 Base Address 			*/
#define TIM6_BASE_ADDR          		      (APB_BASE_ADDR + 0x00001000UL)     /* Timer 6 Base Address 			*/
#define TIM7_BASE_ADDR          		      (APB_BASE_ADDR + 0x00001400UL)     /* Timer 6 Base Address 			*/
#define TIM14_BASE_ADDR                       (APB_BASE_ADDR + 0x00002000UL)     /* Timer 14 Base Address 		    */
#define RTC_BASE_ADDR           			  (APB_BASE_ADDR + 0x00002800UL)     /* RTC Base Address 				*/
#define WWDG_BASE_ADDR                        (APB_BASE_ADDR + 0x00002C00UL)     /* WWDG Base Address 				*/
#define IWDG_BASE_ADDR                        (APB_BASE_ADDR + 0x00003000UL)     /* IWDG Base Address 				*/

#define SPI2_BASE_ADDR                        (APB_BASE_ADDR + 0x00003800UL)     /* SPI 2 Base Address 				*/

#define USART2_BASE_ADDR                      (APB_BASE_ADDR + 0x00004400UL)     /* USART 2 Base Address 		    */
#define USART3_BASE_ADDR                      (APB_BASE_ADDR + 0x00004800UL)     /* USART 3 Base Address 		    */
#define USART4_BASE_ADDR                      (APB_BASE_ADDR + 0x00004C00UL)     /* USART 4 Base Address 		    */
#define USART5_BASE_ADDR                      (APB_BASE_ADDR + 0x00005000UL)     /* USART 5 Base Address 		    */

#define I2C1_BASE_ADDR                        (APB_BASE_ADDR + 0x00005400UL)     /* I2C1 Base Address 				*/
#define I2C2_BASE_ADDR                        (APB_BASE_ADDR + 0x00005800UL)     /* I2C2 Base Address 				*/

#define PWR_BASE_ADDR                         (APB_BASE_ADDR + 0x00007000UL)     /* PWR Base Address 				*/
#define DAC_BASE_ADDR                         (APB_BASE_ADDR + 0x00007400UL)     /* DAC Base Address 				*/

#define CEC_BASE_ADDR                         (APB_BASE_ADDR + 0x00007800UL)     /* CEC Base Address 				*/

#define SYSCFG_BASE_ADDR                      (APB_BASE_ADDR + 0x00010000UL)     /* SYSCFG Base Address 			*/
#define COMP_BASE_ADDR                        (APB_BASE_ADDR + 0x0001001CUL)     /* COMP Base Address 				*/

#define EXTI_BASE_ADDR                        (APB_BASE_ADDR + 0x00010400UL)     /* EXTI Base Address 				*/

#define ADC1_BASE_ADDR                        (APB_BASE_ADDR + 0x00012400UL)     /* ADC1 Base Address 				*/
#define ADC_BASE_ADDR                         (APB_BASE_ADDR + 0x00012708UL)     /* ADC Base Address 				*/

#define TIM1_BASE_ADDR                        (APB_BASE_ADDR + 0x00012C00UL)     /* TIM1 Base Address 				*/

#define SPI1_BASE_ADDR                        (APB_BASE_ADDR + 0x00013000UL)     /* SPI1 Base Address 				*/

#define USART1_BASE_ADDR                      (APB_BASE_ADDR + 0x00013800UL)     /* USART1 Base Address 			*/

#define TIM15_BASE_ADDR                       (APB_BASE_ADDR + 0x00014000UL)     /* TIM15 Base Address 				*/
#define TIM16_BASE_ADDR                       (APB_BASE_ADDR + 0x00014400UL)     /* TIM16 Base Address 				*/
#define TIM17_BASE_ADDR                       (APB_BASE_ADDR + 0x00014800UL)     /* TIM17 Base Address 				*/

#define DBGMCU_BASE_ADDR                      (APB_BASE_ADDR + 0x00015800UL)     /* TIM17 Base Address 				*/


/*
 * AHB1 Peripheral Base Addresses
 *
 */

#define DMA1_BASE_ADDR              (AHB1_BASE_ADDR + 0x00000000UL)     /* DMA1 Base Address 				*/
#define DMA2_BASE_ADDR              (AHB1_BASE_ADDR + 0x00000400UL)     /* DMA2 Base Address 				*/

#define RCC_BASE_ADDR               (AHB1_BASE_ADDR + 0x00001000UL)     /* RCC Base Address 				*/

#define FLASH_R_BASE_ADDR           (AHB1_BASE_ADDR + 0x00002000UL)     /* FLASH Base Address 				*/
#define OB_BASE_ADDR                0x1FFFF800UL       /*!< FLASH Option Bytes base address */
#define FLASHSIZE_BASE_ADDR         0x1FFFF7CCUL       /*!< FLASH Size register base address */
#define UID_BASE_ADDR               0x1FFFF7ACUL       /*!< Unique device ID register base address */
#define CRC_BASE_ADDR               (AHB1_BASE_ADDR + 0x00003000UL)     /* CRC Base Address 				*/
#define TSC_BASE_ADDR               (AHB1_BASE_ADDR + 0x00004000UL)     /* TSC Base Address 				*/


/*
 * AHB2 Peripheral Base Addresses
 *
 */

#define GPIOA_BASE_ADDR             (AHB2_BASE_ADDR + 0x00000000UL)     /* GPIOA Base Address 				*/
#define GPIOB_BASE_ADDR             (AHB2_BASE_ADDR + 0x00000400UL)     /* GPIOB Base Address 				*/
#define GPIOC_BASE_ADDR             (AHB2_BASE_ADDR + 0x00000800UL)     /* GPIOC Base Address 				*/
#define GPIOD_BASE_ADDR             (AHB2_BASE_ADDR + 0x00000C00UL)     /* GPIOD Base Address 				*/
#define GPIOE_BASE_ADDR             (AHB2_BASE_ADDR + 0x00001000UL)     /* GPIOE Base Address 				*/
#define GPIOF_BASE_ADDR             (AHB2_BASE_ADDR + 0x00001400UL)     /* GPIOF Base Address 				*/


/*
 * Peripheral Structure Definitions
 *
 */

typedef struct
{
	__IO uint32_t MODER;        /*!< GPIO port mode register,                     Address offset: 0x00      */
	__IO uint32_t OTYPER;       /*!< GPIO port output type register,              Address offset: 0x04      */
	__IO uint32_t OSPEEDR;      /*!< GPIO port output speed register,             Address offset: 0x08      */
	__IO uint32_t PUPDR;        /*!< GPIO port pull-up/pull-down register,        Address offset: 0x0C     */
	__IO uint32_t IDR;          /*!< GPIO port input data register,               Address offset: 0x10      */
	__IO uint32_t ODR;          /*!< GPIO port output data register,              Address offset: 0x14      */
	__IO uint32_t BSRR;         /*!< GPIO port bit set/reset register,      Address offset: 0x1A */
	__IO uint32_t LCKR;         /*!< GPIO port configuration lock register,       Address offset: 0x1C      */
	__IO uint32_t AFR[2];       /*!< GPIO alternate function low register,  Address offset: 0x20-0x24 */
	__IO uint32_t BRR;          /*!< GPIO bit reset register,                     Address offset: 0x28      */

}GPIO_TypeDef_t;

/**
  * @brief Reset and Clock Control
  */

typedef struct
{
  __IO uint32_t CR;         /*!< RCC clock control register,                                  Address offset: 0x00 */
  __IO uint32_t CFGR;       /*!< RCC clock configuration register,                            Address offset: 0x04 */
  __IO uint32_t CIR;        /*!< RCC clock interrupt register,                                Address offset: 0x08 */
  __IO uint32_t APB2RSTR;   /*!< RCC APB2 peripheral reset register,                          Address offset: 0x0C */
  __IO uint32_t APB1RSTR;   /*!< RCC APB1 peripheral reset register,                          Address offset: 0x10 */
  __IO uint32_t AHBENR;     /*!< RCC AHB peripheral clock register,                           Address offset: 0x14 */
  __IO uint32_t APB2ENR;    /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x18 */
  __IO uint32_t APB1ENR;    /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x1C */
  __IO uint32_t BDCR;       /*!< RCC Backup domain control register,                          Address offset: 0x20 */
  __IO uint32_t CSR;        /*!< RCC clock control & status register,                         Address offset: 0x24 */
  __IO uint32_t AHBRSTR;    /*!< RCC AHB peripheral reset register,                           Address offset: 0x28 */
  __IO uint32_t CFGR2;      /*!< RCC clock configuration register 2,                          Address offset: 0x2C */
  __IO uint32_t CFGR3;      /*!< RCC clock configuration register 3,                          Address offset: 0x30 */
  __IO uint32_t CR2;        /*!< RCC clock control register 2,                                Address offset: 0x34 */

} RCC_TypeDef_t;

typedef struct
{
  __IO uint32_t CFGR1;       /*!< SYSCFG configuration register 1,                           Address offset: 0x00 */
       uint32_t RESERVED;    /*!< Reserved,                                                                  0x04 */
  __IO uint32_t EXTICR[4];   /*!< SYSCFG external interrupt configuration register,     Address offset: 0x14-0x08 */
  __IO uint32_t CFGR2;       /*!< SYSCFG configuration register 2,                           Address offset: 0x18 */

} SYSCFG_TypeDef_t;

typedef struct
{
  __IO uint32_t IMR;          /*!<EXTI Interrupt mask register,                 Address offset: 0x00 */
  __IO uint32_t EMR;          /*!<EXTI Event mask register,                     Address offset: 0x04 */
  __IO uint32_t RTSR;         /*!<EXTI Rising trigger selection register ,      Address offset: 0x08 */
  __IO uint32_t FTSR;         /*!<EXTI Falling trigger selection register,      Address offset: 0x0C */
  __IO uint32_t SWIER;        /*!<EXTI Software interrupt event register,       Address offset: 0x10 */
  __IO uint32_t PR;           /*!<EXTI Pending register,                        Address offset: 0x14 */

} EXTI_TypeDef_t;


typedef struct
{
  __IO uint32_t CR1;          /*!<SPI control register 1,                     Address offset: 0x00 */
  __IO uint32_t CR2;          /*!<SPI control register 2,                     Address offset: 0x04 */
  __IO uint32_t SR;       	  /*!<SPI status register ,                       Address offset: 0x08 */
  __IO uint32_t DR;        	  /*!<SPI data register,                          Address offset: 0x0C */
  __IO uint32_t CRCPR;        /*!<SPI CRC polynomial register,                Address offset: 0x10 */
  __IO uint32_t RXCRCR;       /*!<SPI Rx CRC register,                        Address offset: 0x14 */
  __IO uint32_t TXCRCR;		  /*!<SPI Tx CRC register,                        Address offset: 0x18 */
  __IO uint32_t I2SCFGR;      /*!<SPIx_I2S configuration register,            Address offset: 0x1C */
  __IO uint32_t I2SPR;		  /*!<SPIx_I2S prescaler register,                Address offset: 0x20 */

} SPI_TypeDef_t;

#define GPIOA                      ( (GPIO_TypeDef_t *)(GPIOA_BASE_ADDR) )
#define GPIOB                      ( (GPIO_TypeDef_t *)(GPIOB_BASE_ADDR) )
#define GPIOC                      ( (GPIO_TypeDef_t *)(GPIOC_BASE_ADDR) )
#define GPIOD                      ( (GPIO_TypeDef_t *)(GPIOD_BASE_ADDR) )
#define GPIOE                      ( (GPIO_TypeDef_t *)(GPIOE_BASE_ADDR) )
#define GPIOF                      ( (GPIO_TypeDef_t *)(GPIOF_BASE_ADDR) )


#define RCC						   ( (RCC_TypeDef_t *)(RCC_BASE_ADDR   ) )

#define SYSCFG					   ( (SYSCFG_TypeDef_t *)(SYSCFG_BASE_ADDR ) )

#define EXTI 						( (EXTI_TypeDef_t *)(EXTI_BASE_ADDR ) )

#define SPI1 						( (SPI_TypeDef_t *)(SPI1_BASE_ADDR ) )
#define SPI2 						( (SPI_TypeDef_t *)(SPI2_BASE_ADDR ) )

/*
 * Bit Definition
 *
 */


#define RCC_AHBENR_GPIOAEN_Pos                   (17U)                         		  /*RCC AHB Register GPIOAEN Bit position*/
#define RCC_AHBENR_GPIOAEN_Msk                   (0x1UL << RCC_AHBENR_GPIOAEN_Pos)    /*!<RCC AHB Register GPIOAEN Bit Mask */
#define RCC_AHBENR_GPIOAEN                       RCC_AHBENR_GPIOAEN_Msk               /*!<RCC AHB Register GPIOAEN Macro*/

#define RCC_AHBENR_GPIOBEN_Pos                   (18U)                         		  /*RCC AHB Register GPIOBEN Bit position*/
#define RCC_AHBENR_GPIOBEN_Msk                   (0x1UL << RCC_AHBENR_GPIOBEN_Pos)    /*!<RCC AHB Register GPIOBEN Bit Mask */
#define RCC_AHBENR_GPIOBEN                       RCC_AHBENR_GPIOBEN_Msk               /*!<RCC AHB Register GPIOBEN Macro*/

#define RCC_AHBENR_GPIOCEN_Pos                   (19U)                         		  /*RCC AHB Register GPIOCEN Bit position*/
#define RCC_AHBENR_GPIOCEN_Msk                   (0x1UL << RCC_AHBENR_GPIOCEN_Pos)    /*!<RCC AHB Register GPIOCEN Bit Mask */
#define RCC_AHBENR_GPIOCEN                       RCC_AHBENR_GPIOCEN_Msk               /*!<RCC AHB Register GPIOCEN Macro*/

#define RCC_AHBENR_GPIODEN_Pos                   (20U)
#define RCC_AHBENR_GPIODEN_Msk                   (0x1UL << RCC_AHBENR_GPIODEN_Pos) /*!< 0x00100000 */
#define RCC_AHBENR_GPIODEN                       RCC_AHBENR_GPIODEN_Msk        /*!< GPIOD clock enable */

#define RCC_AHBENR_GPIOFEN_Pos                   (22U)
#define RCC_AHBENR_GPIOFEN_Msk                   (0x1UL << RCC_AHBENR_GPIOFEN_Pos) /*!< 0x00400000 */
#define RCC_AHBENR_GPIOFEN                       RCC_AHBENR_GPIOFEN_Msk        /*!< GPIOF clock enable */

#define RCC_APB2ENR_SYSCFGEN_Pos                 (0U)
#define RCC_APB2ENR_SYSCFGEN_Msk                 (0x1UL << RCC_APB2ENR_SYSCFGEN_Pos)
#define APB2ENR_SYSCFGEN                         RCC_APB2ENR_SYSCFGEN_Msk

#define RCC_APB2ENR_SPI1EN_Pos                  (12U)
#define RCC_APB2ENR_SPI1EN_Msk                 (0x1UL << RCC_APB2ENR_SPI1EN_Pos)
#define RCC_APB2ENR_SPI1EN                     RCC_APB2ENR_SPI1EN_Msk


#define RCC_APB1ENR_SPI2EN_Pos                 (14U)
#define RCC_APB1ENR_SPI2EN_Msk                 (0x1UL << RCC_APB1ENR_SPI2EN_Pos)
#define RCC_APB1ENR_SPI2EN                     RCC_APB1ENR_SPI2EN_Msk


#include "RCC.h"
#include "GPIO.h"
#include "EXTI.h"
#include "SPI.h"

#endif /* INC_STM32F051XX_H_ */
