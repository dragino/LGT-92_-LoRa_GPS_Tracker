
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EXTI_WAKEUP_H__
#define __EXTI_WAKEUP_H__

#ifdef __cplusplus
 extern "C" {
#endif
	 
	 
/**
  * @brief  System Power Configuration
  *         The system Power is configured as follow : 
  *            + Regulator in LP mode
  *            + VREFINT OFF, with fast wakeup enabled
  *            + MSI as SysClk after Wake Up
  *            + No IWDG
  *            + Wakeup using EXTI Line (User push-button PC.13)
  * @param  None
  * @retval None
  */	 
void SystemPower_Config(void);
/**
  * @brief  Configures EXTI lines 4 to 15 (connected to PC.13 pin) in interrupt mode
  * @param  None
  * @retval None
  */
void EXTI4_15_IRQHandler_Config(void);
/**
  * @brief GPIO EXTI callback
  * @param None
  * @retval None
  */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#ifdef __cplusplus
}
 
//void send(void);
#endif

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

