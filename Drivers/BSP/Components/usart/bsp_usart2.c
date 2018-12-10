
#include "bsp_usart2.h"
#include "vcom.h"
#include <stdarg.h>


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BUFSIZE 256
#define BUFFSIZE 256
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t aRxBuffer[BUFFSIZE];  
/* buffer */
static char buff[BUFSIZE];
/* buffer write index*/
__IO uint16_t iw1=0;
/* buffer read index*/
//static uint16_t ir=0;

 UART_HandleTypeDef uart1;

/* Private function prototypes -----------------------------------------------*/
/* Functions Definition ------------------------------------------------------*/

void usart1_Init(void)
{
  /*## Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART1 configured as follow:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = ODD parity
      - BaudRate = 921600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  uart1.Instance        = USART1;
  
  uart1.Init.BaudRate   = 9600;
  uart1.Init.WordLength = UART_WORDLENGTH_8B;
  uart1.Init.StopBits   = UART_STOPBITS_1;
  uart1.Init.Parity     = UART_PARITY_NONE;
  uart1.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  uart1.Init.Mode       = UART_MODE_TX_RX;
  
  if(HAL_UART_Init(&uart1) != HAL_OK)
  {
    /* Initialization Error */
    //Error_Handler(); 
  } 
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
	__HAL_UART_ENABLE_IT(&uart1,UART_IT_RXNE);//??????
}

void usart1_DeInit(void)
{
#if 1
  HAL_UART_DeInit(&uart1);
#endif
}

//void uart2_Send( char *format, ... )
//{
//  va_list args;
//  va_start(args, format);
//  
//  /*convert into string at buff[0] of length iw*/
//  iw1= vsprintf(&buff[0], format, args);
//  
//  HAL_UART_Transmit(&uart2,(uint8_t *)&buff[0], iw1, 300);
// // HAL_NVIC_SetPendingIRQ(USART2_IRQn);
//  va_end(args);
//}

void usart1_IoInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct={0};
    /* Enable GPIO TX/RX clock */
  __GPIOB_CLK_ENABLE();
    /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = GPIO_PIN_6;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_USART1;

  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Alternate = GPIO_AF0_USART1;

  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
}


void usart1_IoDeInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure={0};
  
 __GPIOB_CLK_ENABLE();

  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  
  GPIO_InitStructure.Pin =  GPIO_PIN_6 ;
  HAL_GPIO_Init(  GPIOB, &GPIO_InitStructure );
  
  GPIO_InitStructure.Pin = GPIO_PIN_7 ;
  HAL_GPIO_Init(  GPIOB, &GPIO_InitStructure ); 
}


/**
  * @brief UART MSP DeInit
  * @param huart: uart handle
  * @retval None
  */
void usart1_IRQHandler(UART_HandleTypeDef *huart)
{
	int rx_ready = 0;
  uint8_t rx;
	uint32_t isrflags   = READ_REG(huart->Instance->ISR);
  uint32_t cr1its     = READ_REG(huart->Instance->CR1);
  uint32_t cr3its = READ_REG(huart->Instance->CR3);;
  uint32_t errorflags;
	
	    /* UART wakeup from Stop mode interrupt occurred ---------------------------*/
//    if(((isrflags & USART_ISR_WUF) != RESET) && ((cr3its & USART_CR3_WUFIE) != RESET))
//    {
//      __HAL_UART_CLEAR_IT(huart, UART_CLEAR_WUF);
//      
//       /* forbid stop mode */
//       //LowPower_Disable(e_LOW_POWER_UART);  
//       
//      /* Enable the UART Data Register not empty Interrupts */
//      SET_BIT(huart->Instance->CR1, USART_CR1_RXNEIE);
//    
//      /* Set the UART state ready to be able to start again the process */
//      huart->gState  = HAL_UART_STATE_READY;
//      huart->RxState = HAL_UART_STATE_READY;

//    }
		
		/* UART in mode Receiver ---------------------------------------------------*/
    if(((isrflags & USART_ISR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
    {
		/* Check that a Rx process is ongoing */
//		if(huart->RxState == HAL_UART_STATE_BUSY_RX)
//		{
		        /*RXNE flag is auto cleared by reading the data*/
                        rx= (uint8_t)READ_REG(huart->Instance->RDR);	
                        
                        /* allow stop mode*/
                        //LowPower_Enable(e_LOW_POWER_UART);
                        
			
		rx_ready = 1;
	//	}	
			  /* If error occurs */
     errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE));
     if (errorflags != RESET)
     {
	   /* Error on receiving */ 
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_PEF);	  
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_FEF);
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_OREF);     
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_NEF);
//	   *((huart->pRxBuffPtr)-1) = 0x01;           /*we skip the overrun case*/
	   rx_ready = 1;
	   }
	 }
	if(rx_ready)
	{
		GPS_usart(rx);
//   	PRINTF("RX :%c",rx);
	}
}





/*********************************************END OF FILE**********************/
