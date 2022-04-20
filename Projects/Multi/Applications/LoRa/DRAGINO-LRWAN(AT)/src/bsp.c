 /******************************************************************************
  * @file    bsp.c
  * @author  MCD Application Team
  * @version V1.1.4
  * @date    08-January-2018
  * @brief   manages the sensors on the application
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
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
#include <string.h>
#include <stdlib.h>
#include "hw.h"
#include "timeServer.h"
#include "bsp.h"
#include "lora.h"
#include "delay.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

extern uint32_t LON ;
extern uint16_t batteryLevel_ref; // ADC reference

void BSP_sensor_Read( sensor_t *sensor_data)
{
  uint16_t adc;
  uint16_t pin_voltage; // mV on ADC

  HAL_GPIO_WritePin(battery_CONTROL_PORT, battery_CONTROL_PIN, GPIO_PIN_RESET);
  adc = HW_AdcReadChannel(ADC_Channel_battery);
  HAL_GPIO_WritePin(battery_CONTROL_PORT, battery_CONTROL_PIN, GPIO_PIN_SET);

  HW_GetBatteryLevel(); // calculates batteryLevel_ref
  pin_voltage = adc * batteryLevel_ref / 4095;
  sensor_data->bat_mv = pin_voltage * (47 + 10) / 47; // resistor divider
}

void  BSP_sensor_Init( void  )
{
	  GPIO_InitTypeDef GPIO_InitStructure; 
	  BSP_battery_Init();
	  GPS_Init();
  	LED_CLK_ENABLE();  
	
	  GPIO_InitStructure.Pin =   LED1_PIN | LED0_PIN | LED3_PIN  ;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull  = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(LED3_PORT , &GPIO_InitStructure);  
	
}

void  BSP_sensor_DeInit( void  )
{
	  GPIO_InitTypeDef GPIO_InitStructure; 
	
  	__GPIOA_CLK_ENABLE();

    GPIO_InitStructure.Pin =  LED1_PIN | LED0_PIN | LED3_PIN  ;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
//    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);     
	
}
void  BSP_powerLED_Init( void  )
{
	  GPIO_InitTypeDef GPIO_InitStructure; 

  	LED_CLK_ENABLE();
	
	  GPIO_InitStructure.Pin =   LED1_PIN | LED0_PIN | LED3_PIN  ;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull  = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(LED3_PORT , &GPIO_InitStructure);  
	
}

void  GPS_Init( void  )
{
	  GPIO_InitTypeDef GPIO_InitStructure; 
	  __GPIOB_CLK_ENABLE();
	  GPIO_InitStructure.Pin   = GPIO_PIN_5   ;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull  = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB , &GPIO_InitStructure);  
	
}

void  BSP_powerLED_DeInit( void  )
{
	   GPIO_InitTypeDef GPIO_InitStructure; 
	
	  __GPIOA_CLK_ENABLE();
	
	  GPIO_InitStructure.Pin =  LED1_PIN | LED0_PIN | LED3_PIN  ;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
//    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);  
	  
	
}

void powerLED(void)
{

	BSP_powerLED_Init();
	LED0_1 ;
	DelayMs(200);
  LED0_0;
	LED1_1 ;
	DelayMs(200);
  LED1_0;	
	LED3_1 ;
	DelayMs(200);
  LED3_0;		

}
void  BSP_battery_Init( void )
{
	GPIO_InitTypeDef GPIO_InitStruct={0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin = battery_CONTROL_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(battery_CONTROL_PORT, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(battery_CONTROL_PORT,battery_CONTROL_PIN,GPIO_PIN_SET);
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
