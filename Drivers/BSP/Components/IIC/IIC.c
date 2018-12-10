/**
  ******************************************************************************
  * @file    Project/User/IIC_OLED.c
  * @author  Dragino
  * @version 
  * @date    8-May-2017
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "IIC.h"

#define GPIO_PORT_IIC	 GPIOA		
#define IIC_SCL_PIN		 GPIO_PIN_9		
#define IIC_SDA_PIN		 GPIO_PIN_10

#define IIC_SCL_1  HAL_GPIO_WritePin(GPIO_PORT_IIC, IIC_SCL_PIN, GPIO_PIN_SET)		
#define IIC_SCL_0  HAL_GPIO_WritePin(GPIO_PORT_IIC, IIC_SCL_PIN, GPIO_PIN_RESET)	
	
#define IIC_SDA_1  HAL_GPIO_WritePin(GPIO_PORT_IIC, IIC_SDA_PIN, GPIO_PIN_SET)		/* SDA = 1 */
#define IIC_SDA_0  HAL_GPIO_WritePin(GPIO_PORT_IIC, IIC_SDA_PIN, GPIO_PIN_RESET)		/* SDA = 0 */
	
#define IIC_SDA_READ()  HAL_GPIO_ReadPin(GPIO_PORT_IIC, IIC_SDA_PIN)	

void IIC_GPIO_MODE_Config(void)
{
	static GPIO_InitTypeDef  GPIO_InitStruct;
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
   
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
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
//	uint8_t i;

	/*　
	 	下面的时间是通过安富莱AX-Pro逻辑分析仪测试得到的。
		CPU主频72MHz时，在内部Flash运行, MDK工程不优化
		循环次数为10时，SCL频率 = 205KHz 
		循环次数为7时，SCL频率 = 347KHz， SCL高电平时间1.5us，SCL低电平时间2.87us 
	 	循环次数为5时，SCL频率 = 421KHz， SCL高电平时间1.25us，SCL低电平时间2.375us 
        
    IAR工程编译效率高，不能设置为7
	*/
  volatile int i = 7;
    while (i)

    i--;
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

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
