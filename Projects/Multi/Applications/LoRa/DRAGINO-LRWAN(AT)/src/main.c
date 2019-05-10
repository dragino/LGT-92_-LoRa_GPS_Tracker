/******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.1.4
  * @date    08-January-2018
  * @brief   this is the main!
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
#include "hw.h"
#include "low_power_manager.h"
#include "lora.h"
#include "bsp.h"
#include "timeServer.h"
#include "vcom.h"
#include "version.h"
#include "command.h"
#include "at.h"
#include "GPS.h"  
#include "bsp_usart2.h"
#include "exti_wakeup.h"
#include "IIC.h"
#include "mpu9250.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/*!
 * CAYENNE_LPP is myDevices Application server.
 */
//#define CAYENNE_LPP
#define LPP_DATATYPE_DIGITAL_INPUT  0x0
#define LPP_DATATYPE_ANOLOG_INPUT   0x02
#define LPP_DATATYPE_HUMIDITY       0x68
#define LPP_DATATYPE_TEMPERATURE    0x67

#define LPP_APP_PORT 99
/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
uint32_t APP_TX_DUTYCYCLE=300000;

uint32_t Server_TX_DUTYCYCLE=300000;

uint32_t Alarm_TX_DUTYCYCLE=60000;

uint32_t GPS_ALARM=0;

extern uint32_t set_sgm;

extern uint32_t s_gm;
extern uint8_t Restart;

int ALARM = 0;
/*!
 * LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE LORAWAN_ADR_ON
/*!
 * LoRaWAN Default data Rate Data Rate
 * @note Please note that LORAWAN_DEFAULT_DATA_RATE is used only when ADR is disabled 
 */
#define LORAWAN_DEFAULT_DATA_RATE DR_0
/*!
 * LoRaWAN application port
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_APP_PORT                            2
/*!
 * Number of trials for the join request.
 */
#define JOINREQ_NBTRIALS                            200
/*!
 * LoRaWAN default endNode class port
 */
#define LORAWAN_DEFAULT_CLASS                       CLASS_A
/*!
 * LoRaWAN default confirm state
 */
#define LORAWAN_DEFAULT_CONFIRM_MSG_STATE           LORAWAN_UNCONFIRMED_MSG
/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFF_SIZE                           64
/*!
 * User application data
 */
static uint8_t AppDataBuff[LORAWAN_APP_DATA_BUFF_SIZE];

uint32_t Alarm_LED = 0,a = 1;

int exti_flag=0,basic_flag=0;

int exti_de=0;

static uint32_t ServerSetTDC;

static uint32_t AlarmSetTDC;

uint8_t TDC_flag=0;

uint8_t flag_1=1 ,LP = 0;

extern uint8_t Alarm_times;

extern uint8_t Alarm_times1;

extern uint32_t start;

extern uint16_t AD_code3;

extern uint32_t Positioning_time;

uint32_t Start_times=0,End_times=0;

FP32 gps_latitude ,gps_longitude;

uint32_t longitude;

uint32_t latitude;

uint32_t SendData=0;

uint16_t batteryLevel_mV;

float Roll_basic=0,Pitch_basic=0,Yaw_basic=0;
float Roll_sum=0,Pitch_sum=0,Yaw_sum=0;
float Roll=0,Pitch=0,Yaw=0;
float Roll1=0,Pitch1=0,Yaw1=0;
float Roll_new=0,Pitch_new=0,Yaw_new=0;
float Roll_old=0,Pitch_old=0,Yaw_old=0;

void lora_send_fsm(void);
void send_data(void);
void send_exti(void);

/*!
 * User application data structure
 */
static lora_AppData_t AppData={ AppDataBuff,  0 ,0 };
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* call back when LoRa endNode has received a frame*/
static void LORA_RxData( lora_AppData_t *AppData);

/* call back when LoRa endNode has just joined*/
static void LORA_HasJoined( void );

/* call back when LoRa endNode has just switch the class*/
static void LORA_ConfirmClass ( DeviceClass_t Class );

/* LoRa endNode send request*/
static void Send( void );

static void lora_send(void);

void send_ALARM_data(void);

#if defined(LoRa_Sensor_Node)
/* start the tx process*/
static void LoraStartTx(TxEventType_t EventType);

static TimerEvent_t TxTimer;

/* tx timer callback function*/
static void OnTxTimerEvent( void );
#endif

/* Private variables ---------------------------------------------------------*/
/* load Main call backs structure*/
static LoRaMainCallback_t LoRaMainCallbacks ={ HW_GetBatteryLevel,
                                               HW_GetTemperatureLevel,
                                               HW_GetUniqueId,
                                               HW_GetRandomSeed,
                                               LORA_RxData,
                                               LORA_HasJoined,
                                               LORA_ConfirmClass};

/* !
 *Initialises the Lora Parameters
 */
static  LoRaParam_t LoRaParamInit= {LORAWAN_ADR_STATE,
                                    LORAWAN_DEFAULT_DATA_RATE,  
                                    LORAWAN_PUBLIC_NETWORK,
                                    JOINREQ_NBTRIALS};

#define Kp 40.0f                       // proportional gain governs rate of convergence toaccelerometer/magnetometer
	 //Kp比例增益 决定了加速度计和磁力计的收敛速度
#define Ki 0.02f          // integral gain governs rate of convergenceof gyroscope biases
		//Ki积分增益 决定了陀螺仪偏差的收敛速度
#define halfT 0.0048f      // half the sample period  
		//halfT采样周期的一半
#define dt 0.0096f		
/***************************************************/

static float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
static float exInt = 0, eyInt = 0, ezInt = 0; 
static short turns=0;
static float newdata=0.0f,olddata=0.0f;
static float pitchoffset,rolloffset,yawoffset;

static float k10=0.0f,k11=0.0f,k12=0.0f,k13=0.0f;
static float k20=0.0f,k21=0.0f,k22=0.0f,k23=0.0f;
static float k30=0.0f,k31=0.0f,k32=0.0f,k33=0.0f;
static float k40=0.0f,k41=0.0f,k42=0.0f,k43=0.0f;


float invSqrt(float number);
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float *roll,float *pitch,float *yaw);
void CountTurns(float *newdata,float *olddata,short *turns);
void CalYaw(float *yaw,short *turns);
void CalibrateToZero(void);
 
 
 	float pitch,roll,yaw;
	float pitch_sum,roll_sum,yaw_sum;
	short igx,igy,igz;
	short iax,iay,iaz;
	short imx,imy,imz;
	float gx,gy,gz;
	float ax,ay,az;
	float mx,my,mz;
	
	float gx_old,gy_old,gz_old;
	float ax_old,ay_old,az_old;
	float mx_old,my_old,mz_old;
	
	uint8_t flag_2=1;																		
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main( void )
{
	uint8_t t=0;
  /* STM32 HAL library initialization*/
  HAL_Init( );
  
  /* Configure the system clock*/
  SystemClock_Config( );
	
	EXTI4_15_IRQHandler_Config();
  
  /* Configure the debug mode*/
  DBG_Init( );
  
	usart1_Init();
		
	GPS_init();
		
	GPS_Run();
	
  /* Configure the hardware*/
  HW_Init( );
 
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
  CMD_Init();
	
	powerLED();
	
	IIC_GPIO_MODE_Config();
	HAL_Delay(10);
	t=MPU_Init();
	while (t)
	{
		PRINTF("MPU_Init error\n\r");
		HAL_Delay(200);
	}
	Restart = 0;		
  /*Disbale Stand-by mode*/
// if(lora_getState() != STATE_WAKE_JOIN)
//	{
  LPM_SetOffMode(LPM_APPLI_Id , LPM_Disable );
//  }
  
  /* Configure the Lora Stack*/
  LORA_Init( &LoRaMainCallbacks, &LoRaParamInit);
  
  while( 1 )
  {
		/* Handle UART commands */
    CMD_Process();
		
		if(s_gm == 1)
		{
//	   lora_send_fsm();
		 lora_send();
		 if(Restart == 1)
	   {
		  NVIC_SystemReset();
      Restart = 0;		 
	   }
		}

    DISABLE_IRQ( );
    /*
     * if an interrupt has occurred after DISABLE_IRQ, it is kept pending
     * and cortex will not enter low power anyway
     * don't go in low power mode if we just received a char
     */
#ifndef LOW_POWER_DISABLE
		MPU_Write_Byte(MPU9250_ADDR,0x6B,0X40);//MPU sleep
    LPM_EnterLowPower();
#endif
    ENABLE_IRQ();
    
    /* USER CODE BEGIN 2 */
    /* USER CODE END 2 */
  }
}

static void LORA_HasJoined( void )
{
  AT_PRINTF("JOINED\n\r");
	Read_Config();
  LORA_RequestClass( LORAWAN_DEFAULT_CLASS );
	
	#if defined(LoRa_Sensor_Node) /*LSN50 Preprocessor compile swicth:hw_conf.h*/
	LoraStartTx( TX_ON_TIMER);
  lora_state_GPS_Send();
	s_gm = 1;	
	start = 0;
	gps.flag = 1;
	#endif
	
	#if defined(AT_Data_Send)     /*LoRa ST Module*/
	AT_PRINTF("Please using AT+SEND or AT+SENDB to send you data!\n\r");
	#endif
}

static void Send( void )
{
  sensor_t sensor_data;
  
  if ( LORA_JoinStatus () != LORA_SET)
  {
    /*Not joined, try again later*/
    return;
  }

	BSP_sensor_Read( &sensor_data );
	
	uint32_t i = 0;
	
	if(basic_flag==1)
	{
		Roll_basic=0;
		Pitch_basic=0;
		basic_flag=0;
	}			
	else if(basic_flag==2)
	{
		Roll_basic=Roll;
		Pitch_basic=Pitch;
		basic_flag=0;
	}
 if(AD_code3 <= 2840)
	{

		LP = 1;
		PRINTF("\n\rAD_code3=%d  ", AD_code3);
	}
	else
	{
		LP = 0;
	}
	  MPU_Write_Byte(MPU9250_ADDR,0x6B,0X00);//唤醒
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//加速度与陀螺仪都工作

		for(int H=0; H<10; H++)
		{
//			MPU_Get_Gyro(&igx,&igy,&igz,&gx,&gy,&gz);
			MPU_Get_Accel(&iax,&iay,&iaz,&ax,&ay,&az);
//			MPU_Get_Mag(&imx,&imy,&imz,&mx,&my,&mz);
			AHRSupdate(0,0,0,ax,ay,az,0,0,0,&roll,&pitch,&yaw);
			olddata=newdata;
			newdata=yaw;
			CountTurns(&newdata,&olddata,&turns);
			CalYaw(&yaw,&turns);
			pitch+=pitchoffset;
			roll+=rolloffset;
			yaw+=yawoffset;
		}
			for(int H=0; H<30; H++)
		{
//			MPU_Get_Gyro(&igx,&igy,&igz,&gx,&gy,&gz);
			MPU_Get_Accel(&iax,&iay,&iaz,&ax,&ay,&az);
//			MPU_Get_Mag(&imx,&imy,&imz,&mx,&my,&mz);
			AHRSupdate(0,0,0,ax,ay,az,0,0,0,&roll,&pitch,&yaw);
			olddata=newdata;
			newdata=yaw;
			CountTurns(&newdata,&olddata,&turns);
			CalYaw(&yaw,&turns);
			pitch+=pitchoffset;
			roll+=rolloffset;
			yaw+=yawoffset;
		
			Pitch_sum+=pitch;
			Roll_sum+=roll;
		}

	Roll_new=Roll_sum/30.0;
	Pitch_new=Pitch_sum/30.0;
	
  if(flag_1==1)
  {
		flag_1=0;
		Roll_old=Roll_new;
		Pitch_old=Pitch_new;
	}

  if(-0.2<Roll_new-Roll_old&&Roll_new-Roll_old<0.2)
	{
		Roll1=(Roll_new+Roll_old)/2.0;
		Roll1=(Roll_old+Roll1)/2.0;
		Roll_old=Roll1;
	}	
	else 
	{Roll1=Roll_new;Roll_old=Roll_new;}
	
  if(-0.2<Pitch_new-Pitch_old&&Pitch_new-Pitch_old<0.2)
	{
		Pitch1=(Pitch_new+Pitch_old)/2.0;
		Pitch1=(Pitch_old+Pitch1)/2.0;
		Pitch_old=Pitch1;
	}		
	else {Pitch1=Pitch_new;Pitch_old=Pitch_new;}
	
	Roll=Roll1;
	Pitch=Pitch1;
	Roll1=Roll1-Roll_basic;
	Pitch1=Pitch1-Pitch_basic;

	Roll_sum=0;
	Pitch_sum=0;
	
	if(gps.latitude > 0 && gps.longitude > 0)		
 	{
   gps_latitude = gps.latitude;
	 gps_longitude = gps.longitude;
	 gps_state_on();
	 PRINTF("\n\rRoll=%d  ",(int)(Roll1*100));
	 PRINTF("Pitch=%d\n\r",(int)(Pitch1*100));
	 PRINTF("%s: %.4f\n\r",(gps.latNS == 'N')?"South":"North",gps_latitude);
	 PRINTF("%s: %.4f\n\r ",(gps.lgtEW == 'E')?"East":"West",gps_longitude);
   
	 if(gps.latNS != 'N')
	 {
	   latitude = gps_latitude*10000;
	   latitude = (~latitude)+1 ; 		 
	 }
	 else
	 {
		latitude = gps_latitude*10000;	 
	 }
	 if(gps.lgtEW != 'E')
	 {
	   longitude = gps_longitude*10000;	 
	   longitude = (~longitude)+1 ; 	 
	 }
	 else
	 {
		 longitude = gps_longitude*10000; 	 
	 }
   gps.latitude = 0;
   gps.longitude = 0;	
   start = 1;		
	}

	AppData.Port = LORAWAN_APP_PORT;
	if(lora_getGPSState() == STATE_GPS_OFF)
			{
				AppData.Buff[i++] = 0x00;
				AppData.Buff[i++] = 0x00;
				AppData.Buff[i++] = 0x00;
				AppData.Buff[i++] = 0x00;
				AppData.Buff[i++] = 0x00;	
				AppData.Buff[i++] = 0x00;
			}
		 else if(lora_getGPSState() == STATE_GPS_NO)
		 {
				AppData.Buff[i++] = 0x0F;
				AppData.Buff[i++] = 0xFF;
				AppData.Buff[i++] = 0xFF;
				AppData.Buff[i++] = 0x0F;
				AppData.Buff[i++] = 0xFF;	
				AppData.Buff[i++] = 0xFF;	 
		 }
		else
		{

		   AppData.Buff[i++] =(int)latitude>>16& 0xFF;
			 AppData.Buff[i++] =(int)latitude>>8& 0xFF;
			 AppData.Buff[i++] =(int)latitude& 0xFF;
			 AppData.Buff[i++] =(int)longitude>>16& 0xFF;
			 AppData.Buff[i++] =(int)longitude>>8& 0xFF;
			 AppData.Buff[i++] =(int)longitude& 0xFF;
		}
   if(set_sgm == 0)
		{

			if(ALARM == 1)
			 {
					AppData.Buff[i++] =(int)(sensor_data.oil)>>8 |0x40;      //oil float
					AppData.Buff[i++] =(int)sensor_data.oil;					
			 }
			else
			 {
					AppData.Buff[i++] =(int)(sensor_data.oil)>>8;       //oil float
					AppData.Buff[i++] =(int)sensor_data.oil;
			 }
					AppData.Buff[i++] =(int)(Roll1*100)>>8;       //Roll
					AppData.Buff[i++] =(int)(Roll1*100);
					AppData.Buff[i++] =(int)(Pitch1*100)>>8;       //Pitch
					AppData.Buff[i++] =(int)(Pitch1*100);
		}
	if(set_sgm == 1)
		{
		  if(ALARM == 1)
			 {
				 AppData.Buff[i++] =(int)(sensor_data.oil)>>8 |0x40;      //oil float
				 AppData.Buff[i++] =(int)sensor_data.oil;
			 }
			 else
			 {
				AppData.Buff[i++] =(int)(sensor_data.oil)>>8;       //oil float
				AppData.Buff[i++] =(int)sensor_data.oil;
			 }
		}			
	if(start == 1 )
	 {
	 gps.flag = 1;
	 AppData.BuffSize = i;
   LORA_send( &AppData, lora_config_reqack_get());
	}
	else
	{
		if(lora_getState() != STATE_LORA_ALARM)
		{
	    lora_state_GPS_Send();
		}
	}
}


static void LORA_RxData( lora_AppData_t *AppData )
{
  set_at_receive(AppData->Port, AppData->Buff, AppData->BuffSize);
	AT_PRINTF("Receive data\n\r");
	AT_PRINTF("%d:",AppData->Port);
	 for (int i = 0; i < AppData->BuffSize; i++)
  {
    AT_PRINTF("%02x", AppData->Buff[i]);
  }
	AT_PRINTF("\n\r");
	  switch(AppData->Buff[0] & 0xff)
      {		
				case 1:
				{
					if( AppData->BuffSize == 4 )
					{
					  ServerSetTDC=( AppData->Buff[1]<<16 | AppData->Buff[2]<<8 | AppData->Buff[3] );//S
					
						if(ServerSetTDC<5)
						{
							PRINTF("TDC setting must be more than 4S\n\r");
						}
						else
						{
					    TDC_flag=1;
							Server_TX_DUTYCYCLE=ServerSetTDC*1000;
							PRINTF("ServerSetTDC: %02x\n\r",ServerSetTDC);
							PRINTF("Server_TX_DUTYCYCLE: %02d\n\r",Server_TX_DUTYCYCLE);
						}
							BSP_sensor_Init();
							LED3_1;
		          HAL_Delay(1000);	
		          LED3_0;
					}
					break;
				}
				
				case 2:
				{
					if( AppData->BuffSize == 2 )
					{
						if(AppData->Buff[1]==0x01)
						{
							 Alarm_times = 60;
							 Alarm_times1 = 60;
               GPS_ALARM = 0;
							 ALARM = 0;
							 BSP_sensor_Init();
						   LED1_1;
						   HAL_Delay(1000);	
						   LED1_0;
							 PRINTF("Alarm_times\n\r");
						}
					}
					break;
				}
				
			case 3:
				{
					/*this port switches the class*/
					if( AppData->BuffSize == 1 )
					{
						switch (  AppData->Buff[0] )
						{
							case 0:
								{
								LORA_RequestClass(CLASS_A);
								PRINTF("CLASS_A\n\r");
								break;
								}
								case 1:
								{
									LORA_RequestClass(CLASS_B);
									PRINTF("CLASS_B\n\r");
									break;
								}
								case 2:
								{
									LORA_RequestClass(CLASS_C);
									PRINTF("CLASS_C\n\r");
									break;
								}
								default:
									break;
						 }
					 }
				  break;
				}

			case 4:
			{
				if( AppData->BuffSize == 2 )
					{
					  if(AppData->Buff[1]==0xFF)
					  {
					    NVIC_SystemReset();
					  }
				  }
					break;
			}
			
			case 5:
			{
				if( AppData->BuffSize == 2 )
				{
					if(AppData->Buff[1]==0x01)
					{
						basic_flag=1;//原始值
						PRINTF("basic_flag=1\n\r");
					}
					else if(AppData->Buff[1]==0x02)
					{
						basic_flag=2;//基准变化值
						PRINTF("basic_flag=2\n\r");
					}
				}
				break;
			}
				
			case 6:
			{
				if( AppData->BuffSize == 2 )
				{
					if(AppData->Buff[1]==0x01)
					{
//							 exti_de=0;GPIO_EXTI_IoInit();
						PRINTF("EXTI_IoInit()\n\r");
						
					}
					else if(AppData->Buff[1]==0x02)
					{
//								exti_de=1;GPIO_EXTI_IoDeInit();exti_flag=0;
						 PRINTF("GPIO_EXTI_IoDeInit()\n\r");
					}
				}
				break;
			}			
				default:
					break;
		}
		if(TDC_flag==1)
		{
			Store_Config();
			TimerInit( &TxTimer, OnTxTimerEvent );
			TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE); 
			TimerStart( &TxTimer);
			TDC_flag=0;
			PRINTF("Store_Config()\n\r");
		}
}

#if defined(LoRa_Sensor_Node)
static void OnTxTimerEvent( void )
{
	if(lora_getState() != STATE_GPS_SEND )
	 { 	
		Send( );			 
	 }
	TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
	
  /*Wait for next tx slot*/
  TimerStart( &TxTimer);
}

static void LoraStartTx(TxEventType_t EventType)
{
  if (EventType == TX_ON_TIMER)
  {
    /* send everytime timer elapses */
    TimerInit( &TxTimer, OnTxTimerEvent );
    TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE); 
		lora_state_GPS_Send();
		gps.flag = 1;
    gps.latitude = 0;
    gps.longitude = 0;			
    OnTxTimerEvent();
  }
}
#endif

static void LORA_ConfirmClass ( DeviceClass_t Class )
{
  PRINTF("switch to class %c done\n\r","ABC"[Class] );

  /*Optionnal*/
  /*informs the server that switch has occurred ASAP*/
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;
  
  LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
}

void lora_send(void)
{
  switch(lora_getState())
  {
		
		case STATE_LED:
		{
		  gps.flag = 1;
			start = 0;
			GPS_POWER_OFF();
			BSP_sensor_Init();
			LPM_SetOffMode(LPM_APPLI_Id ,LPM_Disable );
			if(a == 0)
			{
		   LED1_1;		
		   HAL_Delay(500);	
		   LED1_0;
			 HAL_Delay(500);	
			 start = 0;
			 a ++;
			}
			a ++;
			if( a == 10 )
			{
				a = 0;
				
			} 		
			break;
		}
		case  STATE_GPS_SEND:
		{
	
			if(gps.latitude > 0 && gps.longitude > 0)		
  		{
				SendData = 1;
			}	
      if(SendData == 1)
			{				
			 if(GPS_ALARM == 0)
				{
					send_data();
				  gps_state_on();
			    a = 1;
					GPS_ALARM = 0;
          SendData = 0;					
				}
				if(GPS_ALARM == 1)
				{
				 if(Alarm_LED == 0)
					{
						gps_latitude = gps.latitude;
						gps_longitude = gps.longitude;
						gps.flag = 1;
						gps.GSA_mode2 = 0;
						start = 1;	
						ALARM = 1;						
						Alarm_times1 = 0;
						Alarm_times = 60;
						Send( );
						GPS_POWER_OFF();
						BSP_sensor_Init();
					}
					if( Alarm_LED < 60)
					 {	 	 
						LED3_1; 
						HAL_Delay(500);
						LED3_0;
						HAL_Delay(500);
						Alarm_LED ++; 
						GPS_ALARM = 1;
						PRINTF("Alarm_LED:%d\n\r",Alarm_LED);	
					 }
					 if( Alarm_LED == 60)
					 {
						Alarm_times = 0; 
						GPS_ALARM = 1;
					  Alarm_LED ++;						 
					 }
				 if(Alarm_times < 60)
				 {
					 send_ALARM_data();	
					 gps_state_on();
           a = 100;		
           GPS_ALARM = 1;
           SendData = 0;						 
           PRINTF("seng data \n\r");					 
				 }
				 if(Alarm_times1 == 60)
				 {
					 lora_state_GPS_Send();
					 start = 0;	
					 ALARM = 0;
					 Alarm_LED = 0;
           GPS_ALARM = 0;						 
					 PRINTF("led\n\r");			 
					 APP_TX_DUTYCYCLE=Server_TX_DUTYCYCLE;
					 
					 TimerInit( &TxTimer, OnTxTimerEvent );
					 TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
			
					 /*Wait for next tx slot*/
					 TimerStart( &TxTimer);
					 LPM_SetOffMode(LPM_APPLI_Id ,LPM_Disable );
					 BSP_sensor_Init();
					 LED1_1;
					 HAL_Delay(1000);	
					 LED1_0;
					 DISABLE_IRQ( );
				/*
				 * if an interrupt has occurred after DISABLE_IRQ, it is kept pending
				 * and cortex will not enter low power anyway
				 * don't go in low power mode if we just received a char
				 */
#ifndef LOW_POWER_DISABLE
		MPU_Write_Byte(MPU9250_ADDR,0x6B,0X40);//MPU sleep
		LPM_EnterLowPower();
#endif
					 ENABLE_IRQ();			 
				 }	
				}
		  }
			if(LP == 0)
			{
				LPM_SetOffMode(LPM_APPLI_Id , LPM_Enable );
				BSP_sensor_Init();
			  POWER_ON();
			  GPS_INPUT();
				LP = 0;
				LED0_0;
			  Start_times ++;
			}
			if(LP == 1)
      {
				start = 1;
				gps_state_no();
			  APP_TX_DUTYCYCLE = Server_TX_DUTYCYCLE;
			  TimerInit( &TxTimer, OnTxTimerEvent );
	      TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
	      Send( );	
			  TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
        /*Wait for next tx slot*/
        TimerStart( &TxTimer);
			  LPM_SetOffMode(LPM_APPLI_Id ,LPM_Disable );
			  PRINTF("Server_TX_DUTYCYCLE11: %02d\n\r",APP_TX_DUTYCYCLE);		
        PRINTF("LP == 1\n\r");				
//				gps_state_no();
		    lora_state_Led();
				a = 1;

			}
				if(Start_times ==2500)
				{
					End_times ++;
					Start_times =0;
					LED0_1;
					HAL_Delay(100);
				}
        if(End_times == 30 || End_times == 60 || End_times ==90 || End_times ==120 )
				{
					PRINTF("End_times:%02d \n\r",End_times); 
					PRINTF("Positioning_time:%02d \n\r",Positioning_time);
					End_times ++;
				}
       			
			  if(End_times >=Positioning_time)
				{
					if(lora_getState() == STATE_GPS_SEND)
					{
						 gps_state_off();	
             a = 100;							
						 send_data();
						 PRINTF("GPS NO FIX\n\r");
						 a = 100;	
						 GPS_ALARM = 0;						
						 LED3_1; 
						 HAL_Delay(500);
						 LED3_0;
						 HAL_Delay(500);
					}
					if(GPS_ALARM == 1)
					{
					 if(Alarm_LED == 0)
						{
							gps_state_off();	
							gps_latitude = gps.latitude;
							gps_longitude = gps.longitude;
							gps.flag = 1;
							gps.GSA_mode2 = 0;
							start = 1;	
							ALARM = 1;	
							Alarm_times1 = 0;
							Alarm_times = 60;
							Send( );
							GPS_POWER_OFF();
							BSP_sensor_Init();
							LPM_SetOffMode(LPM_APPLI_Id ,LPM_Disable );		
						}
					if( Alarm_LED < 60)
					 {	 	 
						LED3_1; 
						HAL_Delay(500);
						LED3_0;
						HAL_Delay(500);
						Alarm_LED ++; 
						GPS_ALARM = 1;
//						Alarm_times = 0;
						PRINTF("Alarm_LED:%d\n\r",Alarm_LED);	
					 }
					 if( Alarm_LED == 60)
					 {
						Alarm_times = 0; 
						GPS_ALARM = 1;
					  Alarm_LED ++;						 
					 }
					 if(Alarm_times < 60)
					 {
						 gps_state_off();	
						 send_ALARM_data();
						 GPS_ALARM = 1;
						 a = 100;					
						 LED3_1; 
						 HAL_Delay(500);
						 LED3_0;
						 HAL_Delay(500);
					 }
					 if(Alarm_times1 == 60)
					 {
						 lora_state_GPS_Send();
						 start = 0;	
						 ALARM = 0;
						 Alarm_LED = 0;
						 GPS_ALARM = 0;					 
						 PRINTF("led\n\r");			 
						 APP_TX_DUTYCYCLE=Server_TX_DUTYCYCLE;
						 
						 TimerInit( &TxTimer, OnTxTimerEvent );
						 TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
				
						 /*Wait for next tx slot*/
						 TimerStart( &TxTimer);
						 LPM_SetOffMode(LPM_APPLI_Id ,LPM_Disable );
						 BSP_sensor_Init();
						 LED0_1;
						 HAL_Delay(1000);	
						 LED0_0;
						 DISABLE_IRQ( );
					/*
					 * if an interrupt has occurred after DISABLE_IRQ, it is kept pending
					 * and cortex will not enter low power anyway
					 * don't go in low power mode if we just received a char
					 */
#ifndef LOW_POWER_DISABLE
		MPU_Write_Byte(MPU9250_ADDR,0x6B,0X40);//MPU sleep
		LPM_EnterLowPower();
#endif
						 ENABLE_IRQ();			 
					 }	
					}				 
			 }
		 break;
		}
		default:
    {
			PRINTF("default\n\r");
			lora_state_Led();	
      start = 0;				
		  break;
    }	
	}
}

void send_data(void)
{
       start = 1;
			 APP_TX_DUTYCYCLE = Server_TX_DUTYCYCLE;
			 TimerInit( &TxTimer, OnTxTimerEvent );
	     TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
	     Send( );	
			 TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
       /*Wait for next tx slot*/
       TimerStart( &TxTimer);
			 LPM_SetOffMode(LPM_APPLI_Id ,LPM_Disable );
			 PRINTF("Server_TX_DUTYCYCLE: %02d\n\r",APP_TX_DUTYCYCLE);
		   lora_state_Led();
  		 gps.flag = 1;
			 BSP_sensor_Init();
       LED0_0;
			 End_times = 0 ;
			 gps.GSA_mode2 = 0;
			 gps_latitude = gps.latitude;
	     gps_longitude = gps.longitude;
       DISABLE_IRQ( );
    /*
     * if an interrupt has occurred after DISABLE_IRQ, it is kept pending
     * and cortex will not enter low power anyway
     * don't go in low power mode if we just received a char
     */
#ifndef LOW_POWER_DISABLE
		MPU_Write_Byte(MPU9250_ADDR,0x6B,0X40);//MPU sleep
    LPM_EnterLowPower();
#endif
    ENABLE_IRQ();	
}

void send_ALARM_data(void)
{
#if defined( REGION_EU868 )
       APP_TX_DUTYCYCLE=Alarm_TX_DUTYCYCLE;
#else
			 APP_TX_DUTYCYCLE=Alarm_TX_DUTYCYCLE;
#endif			
       TimerInit( &TxTimer, OnTxTimerEvent );			 
			 TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
	    
       Send( );	
			 TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
       /*Wait for next tx slot*/
       TimerStart( &TxTimer);		
			 PRINTF("Server_TX_DUTYCYCLE: %02d\n\r",APP_TX_DUTYCYCLE);
       lora_state_Led();
			 BSP_sensor_Init();
			 End_times = 0 ;
	     LED0_0;
	     a = 100;	
			 LED3_1; 
			 HAL_Delay(1000);
			 LED3_0; 
//			 gps_latitude = gps.latitude;
//	     gps_longitude = gps.longitude;	
       DISABLE_IRQ( );
    /*
     * if an interrupt has occurred after DISABLE_IRQ, it is kept pending
     * and cortex will not enter low power anyway
     * don't go in low power mode if we just received a char
     */
#ifndef LOW_POWER_DISABLE
		MPU_Write_Byte(MPU9250_ADDR,0x6B,0X40);//MPU sleep
    LPM_EnterLowPower();
#endif
    ENABLE_IRQ();	
}

/*
*@功能：快速获得开方的倒数
*
*
*/
float invSqrt(float number)
{
	long i;
	float x,y;
	const float f=1.5f;
	
	x=number*0.5f;
	y=number;
	i=*((long*)&y);
	i=0x5f375a86-(i>>1);
	y=*((float *)&i);
	y=y*(f-(x*y*y));
	return y;
}


/*
*@功能：融合加速度计和磁力计进行姿态调整
*
*
*/
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float *roll,float *pitch,float *yaw)
{
	if(flag_2==1)
  {
		flag_2=0;
		ax_old=ax;
		ay_old=ay;
		az_old=az;
		
		gx_old=gx;
		gy_old=gy;
		gz_old=gz;
		
		mx_old=mx;
		my_old=my;
		mz_old=mz;
	}

	ax=(ax+ax_old)/2.0;
	ay=(ay+ay_old)/2.0;
	az=(az+az_old)/2.0;
	
	gx=(gx+gx_old)/2.0;
	gx=(gx+gx_old)/2.0;
	gx=(gx+gx_old)/2.0;
	
	mx=(mx+mx_old)/2.0;
	mx=(mx+mx_old)/2.0;
	mx=(mx+mx_old)/2.0;
	
		ax_old=ax;
		ay_old=ay;
		az_old=az;
		
		gx_old=gx;
		gy_old=gy;
		gz_old=gz;
		
		mx_old=mx;
		my_old=my;
		mz_old=mz;
		
           float norm;									//用于单位化
           float hx, hy, hz, bx, bz;		//
           float vx, vy, vz, wx, wy, wz; 
           float ex, ey, ez;
//					 float tmp0,tmp1,tmp2,tmp3;
 
           // auxiliary variables to reduce number of repeated operations  辅助变量减少重复操作次数
           float q0q0 = q0*q0;
           float q0q1 = q0*q1;
           float q0q2 = q0*q2;
           float q0q3 = q0*q3;
           float q1q1 = q1*q1;
           float q1q2 = q1*q2;
           float q1q3 = q1*q3;
           float q2q2 = q2*q2;
           float q2q3 = q2*q3;
           float q3q3 = q3*q3;
          
           // normalise the measurements  对加速度计和磁力计数据进行规范化
           norm = invSqrt(ax*ax + ay*ay + az*az);
           ax = ax * norm;
           ay = ay * norm;
           az = az * norm;
           norm = invSqrt(mx*mx + my*my + mz*mz);
           mx = mx * norm;
           my = my * norm;
           mz = mz * norm;
          
           // compute reference direction of magnetic field  计算磁场的参考方向
					 //hx,hy,hz是mx,my,mz在参考坐标系的表示
           hx = 2*mx*(0.50 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
           hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.50 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
           hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.50 - q1q1 -q2q2);    
						//bx,by,bz是地球磁场在参考坐标系的表示
           bx = sqrt((hx*hx) + (hy*hy));
           bz = hz;
          
// estimated direction of gravity and magnetic field (v and w)  //估计重力和磁场的方向
//vx,vy,vz是重力加速度在物体坐标系的表示
           vx = 2*(q1q3 - q0q2);
           vy = 2*(q0q1 + q2q3);
           vz = q0q0 - q1q1 - q2q2 + q3q3;
					 //wx,wy,wz是地磁场在物体坐标系的表示
           wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
           wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
           wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2); 
          
// error is sum ofcross product between reference direction of fields and directionmeasured by sensors 
//ex,ey,ez是加速度计与磁力计测量出的方向与实际重力加速度与地磁场方向的误差，误差用叉积来表示，且加速度计与磁力计的权重是一样的
           ex = (ay*vz - az*vy) + (my*wz - mz*wy);
           ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
           ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

           // integral error scaled integral gain
					 //积分误差
           exInt = exInt + ex*Ki*dt;
           eyInt = eyInt + ey*Ki*dt;
           ezInt = ezInt + ez*Ki*dt;
					// printf("exInt=%0.1f eyInt=%0.1f ezInt=%0.1f ",exInt,eyInt,ezInt);
           // adjusted gyroscope measurements
					 //PI调节陀螺仪数据
           gx = gx + Kp*ex + exInt;
           gy = gy + Kp*ey + eyInt;
           gz = gz + Kp*ez + ezInt;
					 //printf("gx=%0.1f gy=%0.1f gz=%0.1f",gx,gy,gz);
          
           // integrate quaernion rate aafnd normalaizle
					 //欧拉法解微分方程
//           tmp0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
//           tmp1 = q1 + ( q0*gx + q2*gz - q3*gy)*halfT;
//           tmp2 = q2 + ( q0*gy - q1*gz + q3*gx)*halfT;
//           tmp3 = q3 + ( q0*gz + q1*gy - q2*gx)*halfT; 
//					 q0=tmp0;
//					 q1=tmp1;
//					 q2=tmp2;
//					 q3=tmp3;
					 //printf("q0=%0.1f q1=%0.1f q2=%0.1f q3=%0.1f",q0,q1,q2,q3);
////RUNGE_KUTTA 法解微分方程
					  k10=0.5 * (-gx*q1 - gy*q2 - gz*q3);
						k11=0.5 * ( gx*q0 + gz*q2 - gy*q3);
						k12=0.5 * ( gy*q0 - gz*q1 + gx*q3);
						k13=0.5 * ( gz*q0 + gy*q1 - gx*q2);
						
						k20=0.5 * (halfT*(q0+halfT*k10) + (halfT-gx)*(q1+halfT*k11) + (halfT-gy)*(q2+halfT*k12) + (halfT-gz)*(q3+halfT*k13));
						k21=0.5 * ((halfT+gx)*(q0+halfT*k10) + halfT*(q1+halfT*k11) + (halfT+gz)*(q2+halfT*k12) + (halfT-gy)*(q3+halfT*k13));
						k22=0.5 * ((halfT+gy)*(q0+halfT*k10) + (halfT-gz)*(q1+halfT*k11) + halfT*(q2+halfT*k12) + (halfT+gx)*(q3+halfT*k13));
						k23=0.5 * ((halfT+gz)*(q0+halfT*k10) + (halfT+gy)*(q1+halfT*k11) + (halfT-gx)*(q2+halfT*k12) + halfT*(q3+halfT*k13));
						
						k30=0.5 * (halfT*(q0+halfT*k20) + (halfT-gx)*(q1+halfT*k21) + (halfT-gy)*(q2+halfT*k22) + (halfT-gz)*(q3+halfT*k23));
						k31=0.5 * ((halfT+gx)*(q0+halfT*k20) + halfT*(q1+halfT*k21) + (halfT+gz)*(q2+halfT*k22) + (halfT-gy)*(q3+halfT*k23));
						k32=0.5 * ((halfT+gy)*(q0+halfT*k20) + (halfT-gz)*(q1+halfT*k21) + halfT*(q2+halfT*k22) + (halfT+gx)*(q3+halfT*k23));
						k33=0.5 * ((halfT+gz)*(q0+halfT*k20) + (halfT+gy)*(q1+halfT*k21) + (halfT-gx)*(q2+halfT*k22) + halfT*(q3+halfT*k23));
						
						k40=0.5 * (dt*(q0+dt*k30) + (dt-gx)*(q1+dt*k31) + (dt-gy)*(q2+dt*k32) + (dt-gz)*(q3+dt*k33));
						k41=0.5 * ((dt+gx)*(q0+dt*k30) + dt*(q1+dt*k31) + (dt+gz)*(q2+dt*k32) + (dt-gy)*(q3+dt*k33));
						k42=0.5 * ((dt+gy)*(q0+dt*k30) + (dt-gz)*(q1+dt*k31) + dt*(q2+dt*k32) + (dt+gx)*(q3+dt*k33));
						k43=0.5 * ((dt+gz)*(q0+dt*k30) + (dt+gy)*(q1+dt*k31) + (dt-gx)*(q2+dt*k32) + dt*(q3+dt*k33));	
						
						q0=q0 + dt/6.0 * (k10+2*k20+2*k30+k40);
						q1=q1 + dt/6.0 * (k11+2*k21+2*k31+k41);
						q2=q2 + dt/6.0 * (k12+2*k22+2*k32+k42);
						q3=q3 + dt/6.0 * (k13+2*k23+2*k33+k43);
						
           // normalise quaternion
           norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
           q0 = q0 * norm;
           q1 = q1 * norm;
           q2 = q2 * norm;
           q3 = q3 * norm;
					 
					 *pitch = -asin(-2 * q1 * q3 + 2 * q0 * q2)* 57.3;	// pitch
					 *roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1)* 57.3;	// roll
					 *yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
}

/*
*@功能：计算水平方向转的圈数
*
*
*/
void CountTurns(float *newdata,float *olddata,short *turns)
{
	if (*newdata<-170.0f && *olddata>170.0f)
		(*turns)++;
	if (*newdata>170.0f && *olddata<-170.0f)
		(*turns)--;

}

/*
*@功能：计算偏航角
*
*
*/
void CalYaw(float *yaw,short *turns)
{
	*yaw=360.0**turns+*yaw;
}

/*
*@功能：补偿欧拉角偏移，主要补偿yaw角
*
*
*/
void CalibrateToZero(void)
{
			uint8_t t=0;
			float sumpitch=0,sumroll=0,sumyaw=0;
			float pitch,roll,yaw;
			short igx,igy,igz;
			short iax,iay,iaz;
			short imx,imy,imz;
			float gx,gy,gz;
			float ax,ay,az;
			float mx,my,mz;
			for (t=0;t<150;t++)
			{
			MPU_Get_Gyro(&igx,&igy,&igz,&gx,&gy,&gz);
			MPU_Get_Accel(&iax,&iay,&iaz,&ax,&ay,&az);
			MPU_Get_Mag(&imx,&imy,&imz,&mx,&my,&mz);
			AHRSupdate(gx,gy,gz,ax,ay,az,mx,my,mz,&roll,&pitch,&yaw);				
//			delay_us(6430);
				HAL_Delay(7);
				if (t>=100)
				{
					sumpitch+=pitch;
					sumroll+=roll;
					sumyaw+=yaw;
				}
			}
			pitchoffset=-sumpitch/150.0f;
			rolloffset=-sumroll/150.0f;
			yawoffset=-sumyaw/150.0f;
			
//			PRINTF("offset %0.1f %0.1f\n\r",rolloffset,pitchoffset);
			pitchoffset=0;
			rolloffset=0;
			yawoffset=0;
}

void USART1_IRQHandler(void)
{
	usart1_IRQHandler(&uart1);
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
