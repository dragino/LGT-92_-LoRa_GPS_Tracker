 /******************************************************************************
  * @file    tfmini_plus.c
  * @author  MCD Application Team
  * @version V1.1.2
  * @date    30-Novermber-2018
  * @brief   manages the sensors on the application
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
  
  /* Includes ------------------------------------------------------------------*/

#include "tfmini_plus.h"
#include "timeServer.h"
#include "delay.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* I2C handler declaration */
extern bool debug_flags;
extern UART_HandleTypeDef UartHandle1;
extern uint8_t aRxBuffer[1];

void BSP_tfmini_Init(void)
{
	uint8_t txdisoutput[5]={0x5A,0x05,0x07,0x00,0x66};   
	uint8_t txlowpower[6] ={0x5A,0x06,0x35,0x01,0x00,0x96};  //1HZ
	uint8_t txsave[4]     ={0x5A,0x04,0x11,0x6f};

  uart1_init_uart1();
	uart1_IoInit();	
  HAL_UART_Transmit(&UartHandle1, txdisoutput, 5, 0xFFFF);	
	DelayMs(50);
  HAL_UART_Transmit(&UartHandle1, txlowpower, 6, 0xFFFF);
	DelayMs(50);	
  HAL_UART_Transmit(&UartHandle1, txsave, 4, 0xFFFF);		
	DelayMs(50);	
	HAL_UART_Receive_IT(&UartHandle1, (uint8_t *)aRxBuffer,1);
	DelayMs(500); 
  uart1_IoDeInit();	
}

void tfmini_read_distance(tfmini_reading_t *tfmini_reading)
{
  uint8_t rxdata[7] ={0x00,0x00,0x00,0x00,0x00,0x00,0x00};

	uart1_IoInit();
	at_tfmini_data_receive(rxdata);
  uart1_IoDeInit();
	
	if ((rxdata[0] == 0x00)&&(rxdata[1] == 0x00))
	{
		tfmini_reading->distance_mm = 4095;
		if(debug_flags==1)
		{			
			PPRINTF("\r\n");					
			PPRINTF("Reading out of LIDAR range \r\n");	
		}			
	} 
	else 
	{
		tfmini_reading->distance_mm = (int)(((rxdata[1]<<8)+rxdata[0])*10);
		tfmini_reading->distance_signal_strengh = (int)((rxdata[3]<<8)+rxdata[2]);
		tfmini_reading->temperature = (int)((rxdata[5]*256+rxdata[4])/8-256);
		if(debug_flags==1)
		{			
			PPRINTF("\r\n");		
			PPRINTF("Dist:%dcm,Strength:%d,Temp:%d\r\n",tfmini_reading->distance_mm/10,tfmini_reading->distance_signal_strengh,tfmini_reading->temperature);
		}
	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

