 /******************************************************************************
  * @file    sht31.c
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

#include "bh1750.h"
#include "delay.h"

bool iic_noack=0;
extern bool debug_flags;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/
float bh1750_read(void)
{
	uint8_t rxdata[2];	
	uint16_t AD_code;
	float luminance;	

	DelayMs(10);//Required	
	IIC_Write_Byte(0x23,0x01);  //power on
	IIC_Write_Byte(0x23,0x20);  //read data
  DelayMs(200);	
  IIC_Read_Len(0x23,2,rxdata);
	IIC_Write_Byte(0x23,0x07);	//clear

	if(iic_noack==1)
	{
		luminance=65535;
	}
	else
	{
		AD_code=(rxdata[0]<<8)+rxdata[1];
		luminance=AD_code/1.2;
	}
	
	iic_noack=0;	
  if(debug_flags==1)
	{		
		PPRINTF("\r\n");			
		PPRINTF("Luminance:%d lux\r\n",(int)luminance);
	}
	return luminance;
}

void I2C_IoInit(void)
{
	static GPIO_InitTypeDef  GPIO_InitStruct;
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
   
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Pin  =IIC_SCL_PIN | IIC_SDA_PIN;

  HAL_GPIO_Init(GPIO_PORT_IIC, &GPIO_InitStruct); 	
}

void I2C_DoInit(void)
{
	static GPIO_InitTypeDef  GPIO_InitStruct;

  GPIO_InitStruct.Pull  = GPIO_PULLUP;
	GPIO_InitStruct.Pin  =IIC_SCL_PIN | IIC_SDA_PIN;

  HAL_GPIO_Init(GPIO_PORT_IIC, &GPIO_InitStruct); 	
}

void SDA_IN(void)
{
	static GPIO_InitTypeDef  GPIO_InitStruct;
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
   
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Pin  = IIC_SDA_PIN;

  HAL_GPIO_Init(GPIO_PORT_IIC, &GPIO_InitStruct); 
}

void SDA_OUT(void)
{
	static GPIO_InitTypeDef  GPIO_InitStruct;
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
   
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Pin  = IIC_SDA_PIN;

  HAL_GPIO_Init(GPIO_PORT_IIC, &GPIO_InitStruct); 
}

void IIC_Delay(void)
{
    volatile int i = 14;//?????????,????
    while (i) {
        i--;
    }
}

void IIC_Start(void)
{
	SDA_OUT();	/* 当SCL高电平时，SDA出现一个下跳沿表示IIC总线启动信号 */
	IIC_SDA_1;
	IIC_SCL_1;
	IIC_Delay();
	IIC_SDA_0;
	IIC_Delay();
	IIC_SCL_0;
	IIC_Delay();
}	
void IIC_Stop(void)
{
	SDA_OUT();/* 当SCL高电平时，SDA出现一个上跳沿表示IIC总线停止信号 */
	IIC_SCL_0;
	IIC_SDA_0;
	IIC_Delay();
	IIC_SCL_1;
	IIC_SDA_1;
	IIC_Delay();
}
uint8_t IIC_WaitAck(void)//0:ACK 1:no ACK
{
	uint8_t ucErrTime=0;
  SDA_IN();
	IIC_SDA_1;	/* CPU释放SDA总线 */
	IIC_Delay();
	IIC_SCL_1;	/* CPU驱动SCL = 1, 此时器件会返回ACK应答 */
	IIC_Delay();
	while (IIC_SDA_READ())	/* CPU读取SDA口线状态 */
	{
			ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}

	IIC_SCL_0;
	IIC_Delay();
	return 0;
}
void IIC_SendByte(uint8_t Byte)
{
	uint8_t i;
	SDA_OUT();
	IIC_SCL_0;
	for (i = 0; i < 8; i++)
	{		
		if (Byte & 0x80)
		{
			IIC_SDA_1;
		}
		else
		{
			IIC_SDA_0;
		}
		IIC_Delay();
		IIC_SCL_1;
		IIC_Delay();	
		IIC_SCL_0;
		IIC_SDA_1; // 释放总线
		Byte <<= 1;	/* 左移一个bit */
		IIC_Delay();
	}
}
uint8_t IIC_ReadByte(unsigned char ack)
{
	uint8_t i;
	uint8_t value;
  SDA_IN();
	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		IIC_SCL_0;
		IIC_Delay();
		IIC_SCL_1;
		
		if (IIC_SDA_READ())
		{
			value++;
		}
		IIC_Delay();
	}
	
	  if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK  
	return value;
}	

void IIC_Ack(void)
{
	IIC_SCL_0;
	SDA_OUT();
	IIC_SDA_0;
	IIC_Delay();
	IIC_SCL_1;
	IIC_Delay();
	IIC_SCL_0;
}

void IIC_NAck(void)
{
	IIC_SCL_0;
	SDA_OUT();
	IIC_SDA_1;
	IIC_Delay();
	IIC_SCL_1;
	IIC_Delay();
	IIC_SCL_0;	
}
//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
uint8_t IIC_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    uint8_t i;
    IIC_Start();
     IIC_SendByte((addr<<1)|0); //发送器件地址+写命令
    if(IIC_WaitAck())          //等待应答
    {
        IIC_Stop();
        return 1;
    }
     IIC_SendByte(reg);         //写寄存器地址
    IIC_WaitAck();             //等待应答
    for(i=0;i<len;i++)
    {
         IIC_SendByte(buf[i]);  //发送数据
        if(IIC_WaitAck())      //等待ACK
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_Stop();
    return 0;
} 

//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
uint8_t IIC_Read_Len(uint8_t addr,uint8_t len,uint8_t *buf)
{ 
    IIC_Start();
    IIC_SendByte((addr<<1)|0); //发送器件地址+写命令
    if(IIC_WaitAck())          //等待应答
    {
        IIC_Stop();
        return 1;
    }
	  IIC_Start();                
    IIC_SendByte((addr<<1)|1); //发送器件地址+读命令
    IIC_WaitAck();             //等待应答
    while(len)
    {
        if(len==1)*buf=IIC_ReadByte(0);//读数据,发送nACK 
		else *buf=IIC_ReadByte(1);		//读数据,发送ACK  
		len--;
		buf++;  
    }
    IIC_Stop();                 //产生一个停止条件
    return 0;       
}

//IIC写一个字节 
//devaddr:器件IIC地址
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
uint8_t IIC_Write_Byte(uint8_t addr,uint8_t data)
{
    IIC_Start();
    IIC_SendByte((addr<<1)|0); //发送器件地址+写命令
    if(IIC_WaitAck())          //等待应答
    {
        IIC_Stop();
				iic_noack=1;
        return 1;
    }
    IIC_SendByte(data);        //发送数据
    if(IIC_WaitAck())          //等待ACK
    {
        IIC_Stop();
			  iic_noack=1;
        return 1;
    }
    IIC_Stop();
    return 0;
}

//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
uint8_t IIC_Read_Byte(uint8_t addr,uint8_t reg)
{
    uint8_t res;
    IIC_Start();
    IIC_SendByte((addr<<1)|0); //发送器件地址+写命令
    IIC_WaitAck();             //等待应答
    IIC_SendByte(reg);         //写寄存器地址
    IIC_WaitAck();             //等待应答
		IIC_Start();                
    IIC_SendByte((addr<<1)|1); //发送器件地址+读命令
    IIC_WaitAck();             //等待应答
    res=IIC_ReadByte(0);		//读数据,发送nACK  
    IIC_Stop();                 //产生一个停止条件
    return res;  
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
