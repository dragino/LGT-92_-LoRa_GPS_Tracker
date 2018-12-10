#include "hw.h"
#include "exti_wakeup.h"

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
void SystemPower_Config(void)
{

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* Enable Ultra low power mode */
  HAL_PWREx_EnableUltraLowPower();
  
  /* Enable the fast wake up from Ultra low power mode */
  HAL_PWREx_EnableFastWakeUp();

  /* Select MSI as system clock source after Wake Up from Stop mode */
  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI);
  
}
/**
  * @brief  Configures EXTI lines 4 to 15 (connected to PC.13 pin) in interrupt mode
  * @param  None
  * @retval None
  */
void EXTI4_15_IRQHandler_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOB clock */
  __HAL_RCC_GPIOB_CLK_ENABLE(); 

  /* Configure PB.14 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING_FALLING ;
  GPIO_InitStructure.Pull = GPIO_PULLDOWN ;
  GPIO_InitStructure.Pin = GPIO_PIN_14;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Enable and set EXTI lines 4 to 15 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}
/**
  * @brief GPIO EXTI callback
  * @param None
  * @retval None
  */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//  /* Clear Wake Up Flag */
////	send();
//  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
//}


