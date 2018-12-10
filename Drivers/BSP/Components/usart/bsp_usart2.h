#ifndef __USART2_H
#define	__USART2_H

#include "stm32l072xx.h"
#include <stdio.h>
#include "stm32l0xx_hal.h"
#include "stm32l0xx_nucleo.h"
#include <stdint.h>
#include "gps.h"

#ifdef __cplusplus
 extern "C" {
#endif
 
extern  UART_HandleTypeDef uart1;
	 
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */ 
/** 
* @brief  Init the VCOM.
* @param  None
* @return None
*/
void Error_Handler(void) ;
	 
	 
void usart1_Init(void);

   /** 
* @brief  DeInit the VCOM.
* @param  None
* @return None
*/
void usart1_DeInit(void);

   /** 
* @brief  Init the VCOM IOs.
* @param  None
* @return None
*/
void usart1_IoInit(void);
  
   /** 
* @brief  DeInit the VCOM IOs.
* @param  None
* @return None
*/
void usart1_IoDeInit(void);
  
/** 
* @brief  Records string on circular Buffer and set SW interrupt
* @note   Set NVIC to call vcom_Send
* @param  string
* @return None
*/
void usart1_Send( char *format, ... );

/** 
* @brief  Sends circular Buffer on com port in IT mode
* @note   called from low Priority interrupt
* @param  None
* @return None
*/
void usart1_Print( void);

void usart1_IRQHandler(UART_HandleTypeDef *huart);

/** 
* @brief  Records string on circular Buffer
* @note   To be called only from critical section from low power section
*         Other wise use vcom_Send
* @param  string
* @return None
*/
void vcom_Send_Lp( char *format, ... );

/* Exported macros -----------------------------------------------------------*/
#if 1
#define printf(...)            vcom_Send(__VA_ARGS__)
#else
#define printf(...)
#endif


#ifdef __cplusplus
}
#endif

#endif /* __VCOM_H__*/
