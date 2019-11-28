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

uint32_t Keep_TX_DUTYCYCLE=21600000;

uint32_t GPS_ALARM=0;

uint32_t GS=0;

extern uint32_t set_sgm;

extern uint32_t LON ;
extern uint32_t MD ;
extern uint32_t MLON ;
extern uint32_t Threshold ;
extern uint32_t Freq ;
extern uint8_t mpuint_flags;

uint32_t CHE = 0;

int ALARM = 0;

uint32_t FLAG=0;

uint8_t send_fail=0;

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

uint32_t start_time=0;
	
uint32_t AlarmSetTDC;

uint8_t flag_1=1 ,LP = 0;

extern uint8_t Alarm_times;

extern uint8_t Alarm_times1;

extern uint32_t start;

extern uint16_t AD_code3;

extern uint32_t Positioning_time;

extern uint8_t md_flags;

uint32_t Start_times=0,End_times=0,gps_time = 0;;

FP32 gps_latitude ,gps_longitude;

int32_t longitude;

int32_t latitude;

uint32_t SendData=0;

uint16_t batteryLevel_mV;

uint16_t TIMES = 10000;

bool is_lora_joined=0;

bool motion_flags=0;

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
static TimerEvent_t TxTimer2;

/* tx timer callback function*/
static void OnTxTimerEvent( void );

static void time(TxEventType_t EventType);

static void timing( void );

extern void printf_joinmessage(void);
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

#define Kp 40.0f        // proportional gain governs rate of convergence toaccelerometer/magnetometer
																		
#define Ki 0.02f        // integral gain governs rate of convergenceof gyroscope biases
	
#define halfT 0.0048f   // half the sample period  
	
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
//	uint8_t t=0;
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

  LPM_SetOffMode(LPM_APPLI_Id , LPM_Disable );

  
  /* Configure the Lora Stack*/
  LORA_Init( &LoRaMainCallbacks, &LoRaParamInit);
  
  while( 1 )
  {
		/* Handle UART commands */
    CMD_Process();	

		if(md_flags==1)
		{
			MPU_INT_Init();		
      md_flags=0;			
		}
			
		lora_send();

		if((motion_flags==1)&&(mpuint_flags==1))
		{
			TimerInit( &TxTimer, OnTxTimerEvent );
			gps.latitude = 0;
			gps.longitude = 0;
			lora_state_GPS_Send();      			
		  lora_send();
			APP_TX_DUTYCYCLE=Server_TX_DUTYCYCLE;
			TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
      /*Wait for next tx slot*/
      TimerStart( &TxTimer);
			motion_flags=0;		
			PPRINTF("Exit static mode\r\n");
		}
		
    DISABLE_IRQ( );
    /*
     * if an interrupt has occurred after DISABLE_IRQ, it is kept pending
     * and cortex will not enter low power anyway
     * don't go in low power mode if we just received a char
     */
#ifndef LOW_POWER_DISABLE
//		MPU_Write_Byte(MPU9250_ADDR,0x6B,0X40);//MPU sleep
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

	BSP_sensor_Init();
	LED3_1;
	LED1_1;  	
	HAL_Delay(1000);
	LED3_0;
	LED1_0; 
	
	Read_Config();
	if((lora_config_otaa_get() == LORA_ENABLE ? 1 : 0))
	{
  printf_joinmessage();
	}	
	
	if(Positioning_time==0)
	{
		Positioning_time=150;
		LON =	1;
		MD=1;
		set_sgm=1;
	  Alarm_TX_DUTYCYCLE=60000;	
	  Keep_TX_DUTYCYCLE=3600000;					
	}

  LORA_RequestClass( LORAWAN_DEFAULT_CLASS );
	
	start_time=HW_RTC_GetTimerValue();	
	
	#if defined(LoRa_Sensor_Node) /*LSN50 Preprocessor compile swicth:hw_conf.h*/
	LoraStartTx( TX_ON_TIMER);
	time(TX_ON_TIMER);
  lora_state_GPS_Send();

	start = 0;
	gps.flag = 1;
	is_lora_joined = 1;
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
		PRINTF("\n\rBattery voltage too low\r\n");
	}
	else
	{
		LP = 0;
	}
	  MPU_Write_Byte(MPU9250_ADDR,0x6B,0X00);//唤醒
	  MPU_Init();
//    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//加速度与陀螺仪都工作

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
	 PRINTF("\n\rRoll=%0.2f  ",((int)(Roll1*100))/100.0);
	 PRINTF("Pitch=%0.2f\n\r",((int)(Pitch1*100))/100.0);
//	 PRINTF("%s: %.6f\n\r",(gps.latNS == 'N')?"South":"North",gps_latitude);
//	 PRINTF("%s: %.6f\n\r ",(gps.lgtEW == 'E')?"East":"West",gps_longitude);
   
	 if(gps.latNS != 'N')
	 {
	   latitude = gps_latitude*1000000;
	   latitude = (~latitude) ;
     gps_latitude = (float)(latitude)/1000000;	 
     PRINTF("%s: %.6f\n\r",(gps.latNS == 'N')?"South":"North",gps_latitude);		 
	 }
	 else
	 {
		latitude = gps_latitude*1000000;
    PRINTF("%s: %.6f\n\r",(gps.latNS == 'N')?"South":"North",gps_latitude);		 
	 }
	 if(gps.lgtEW != 'E')
	 {
	   longitude = gps_longitude*1000000;	 
	   longitude = (~longitude) ;
		 gps_longitude = (float)(longitude)/1000000;
     PRINTF("%s: %.6f\n\r",(gps.latNS == 'E')?"East":"West",gps_longitude);		 
	 }
	 else
	 {
		 longitude = gps_longitude*1000000; 	
     PRINTF("%s: %.6f\n\r ",(gps.lgtEW == 'E')?"East":"West",gps_longitude);		 
	 }
   gps.latitude = 0;
   gps.longitude = 0;	
   start = 1;		
	}	
  FLAG = (int)(MD<<6 | LON<<5)& 0xFF;
//	PRINTF("\n\rFLAG=%d  ",FLAG);
	AppData.Port = LORAWAN_APP_PORT;
	if(lora_getGPSState() == STATE_GPS_OFF)
			{
				AppData.Buff[i++] = 0x00;
				AppData.Buff[i++] = 0x00;
				AppData.Buff[i++] = 0x00;
				AppData.Buff[i++] = 0x00;
				AppData.Buff[i++] = 0x00;	
				AppData.Buff[i++] = 0x00;
				AppData.Buff[i++] = 0x00;	
				AppData.Buff[i++] = 0x00;				
			}
		 else if(lora_getGPSState() == STATE_GPS_NO)
		 {
				AppData.Buff[i++] = 0xFF;
				AppData.Buff[i++] = 0xFF;
				AppData.Buff[i++] = 0xFF;
				AppData.Buff[i++] = 0xFF;
				AppData.Buff[i++] = 0xFF;	
				AppData.Buff[i++] = 0xFF;	
				AppData.Buff[i++] = 0xFF;	
				AppData.Buff[i++] = 0xFF;				 
		 }
		else
		{


			  AppData.Buff[i++] =(int)latitude>>24& 0xFF;
			  AppData.Buff[i++] =(int)latitude>>16& 0xFF;
			  AppData.Buff[i++] =(int)latitude>>8& 0xFF;
			  AppData.Buff[i++] =(int)latitude& 0xFF;
				AppData.Buff[i++] =(int)longitude>>24& 0xFF;
			  AppData.Buff[i++] =(int)longitude>>16& 0xFF;
			  AppData.Buff[i++] =(int)longitude>>8& 0xFF;
			  AppData.Buff[i++] =(int)longitude& 0xFF;		 
		}
   if(set_sgm == 1)
		{

			if(ALARM == 1)
			 {
					AppData.Buff[i++] =(int)(sensor_data.bat_mv)>>8 |0x40;      //oil float
					AppData.Buff[i++] =(int)sensor_data.bat_mv;					
				 
			 }
			else
			 {
					AppData.Buff[i++] =(int)(sensor_data.bat_mv)>>8;       //oil float
					AppData.Buff[i++] =(int)sensor_data.bat_mv;
			 }
			 AppData.Buff[i++] =(int)FLAG;
		}
	else if(set_sgm == 0)
		{
		  if(ALARM == 1)
			 {
				 AppData.Buff[i++] =(int)(sensor_data.bat_mv)>>8 |0x40;      //oil float
				 AppData.Buff[i++] =(int)sensor_data.bat_mv;
			 }
			 else
			 {
				AppData.Buff[i++] =(int)(sensor_data.bat_mv)>>8;       //oil float
				AppData.Buff[i++] =(int)sensor_data.bat_mv;
			 }
			 AppData.Buff[i++] =(int)FLAG;
			 AppData.Buff[i++] =(int)(Roll1*100)>>8;       //Roll
			 AppData.Buff[i++] =(int)(Roll1*100);
			 AppData.Buff[i++] =(int)(Pitch1*100)>>8;       //Pitch
			 AppData.Buff[i++] =(int)(Pitch1*100);
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
					
						if(ServerSetTDC<6)
						{
							PRINTF("TDC setting must be more than 6S\n\r");
							Server_TX_DUTYCYCLE=6000;
						}
						else
						{
							Server_TX_DUTYCYCLE=ServerSetTDC*1000;
							PRINTF("Set TDC: %d ms\n\r",Server_TX_DUTYCYCLE);
						}
						Store_Config();							
						if(LON == 1)
						{
							BSP_sensor_Init();
							LED0_1;						
		          HAL_Delay(1000);	
		          LED0_0;
						}	
					}
					break;
				}
				
				case 2:
				{
					if( AppData->BuffSize == 2 )
					{
						if(AppData->Buff[1]==0x01)
						{
							 start_time=HW_RTC_GetTimerValue();		
							 Alarm_times = 60;
							 Alarm_times1 = 60;
               GPS_ALARM = 0;
							 ALARM = 0;									
							 if(LON == 1)
							 {	 
							  BSP_sensor_Init();									 
						    LED1_1;							 
						    HAL_Delay(1000);	
						    LED1_0;
							 }	
							 PPRINTF("Exit Alarm\r\n");
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
				if( AppData->BuffSize == 4 )
					{
					  if((AppData->Buff[1]==0x00)&&(AppData->Buff[2]==0x00)&&(AppData->Buff[3]==0x01))
					  {
							lora_config_reqack_set(LORAWAN_CONFIRMED_MSG);
							Store_Config();
					  }
						else if((AppData->Buff[1]==0x00)&&(AppData->Buff[2]==0x00)&&(AppData->Buff[3]==0x00))
						{
							lora_config_reqack_set(LORAWAN_UNCONFIRMED_MSG);
							Store_Config();
						}
				  }
					break;
			}
				
			case 6:
			{
				if(AppData->BuffSize == 2 )
				{
					CHE = AppData->Buff[1];
					customize_set8channel_set(CHE);
					PRINTF("Set CHE: %02x\n\r",CHE);
					Store_Config();
				}
				break;
			}	
			case 7:
			{
				if(AppData->BuffSize == 4 )
				{
					if(AppData->Buff[1]==0x03)
					{
					MD = AppData->Buff[1];
					Threshold = AppData->Buff[2]; 
					Freq = AppData->Buff[3]; 
					PRINTF("Set MD: %02x,%02x,%02x\n\r",MD,Threshold,Freq);
					}
					else
					{
					MD = AppData->Buff[1];	
					PRINTF("Set MD: %02x\n\r",MD);						
					}
					if(AppData->Buff[1]!=0x00)
					{
						start_time=HW_RTC_GetTimerValue();							
					}
					md_flags=1;
					Store_Config();
				}				
				break;
			}	
			case 0xa9:
				{
					if( AppData->BuffSize == 4 )
					{
					  ServerSetTDC=( AppData->Buff[1]<<16 | AppData->Buff[2]<<8 | AppData->Buff[3] );//S
					
						if(ServerSetTDC<360)
						{
							PRINTF("KAT setting must be more than 6m\n\r");
							Keep_TX_DUTYCYCLE=360000;
						}
						else
						{
							Keep_TX_DUTYCYCLE=ServerSetTDC*1000;
							PRINTF("Set KAT: %d ms\n\r",Keep_TX_DUTYCYCLE);
						}
						Store_Config();	
					}
					break;
				}			
				default:
					break;
		}
}

#if defined(LoRa_Sensor_Node)
static void OnTxTimerEvent( void )
{
	
	gps.flag = 1;
  gps.latitude = 0;
  gps.longitude = 0;	
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

static void timing(void)
{
	uint32_t temp_time=0;
	if(MD!=0)
	{
		if((motion_flags==0)&&(ALARM ==0))	
		{
			temp_time=HW_RTC_GetTimerValue();
//			PPRINTF("temp_time is %d\r\n",temp_time);
					if(temp_time<start_time)
					{
						start_time=0;
					}
	
					if(mpuint_flags==1)
					{
						start_time=temp_time;
						mpuint_flags=0;		
					}	
					else if(temp_time-start_time>=300000)
					{
						start_time=temp_time;
						PPRINTF("Enter static mode\r\n");
						TimerInit( &TxTimer, OnTxTimerEvent );
						APP_TX_DUTYCYCLE=Keep_TX_DUTYCYCLE;		
						TimerSetValue( &TxTimer, APP_TX_DUTYCYCLE);
						/*Wait for next tx slot*/
						TimerStart( &TxTimer);
						motion_flags=1;	
					}
		}
	}
	TimerSetValue( &TxTimer2,  10000);
	
  /*Wait for next tx slot*/
  TimerStart( &TxTimer2);	

}

static void time(TxEventType_t EventType)
{
  if (EventType == TX_ON_TIMER)
  {
    /* send everytime timer elapses */
    TimerInit( &TxTimer2, timing );
    TimerSetValue( &TxTimer2,  10000); 
    timing();
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
			LPM_SetOffMode(LPM_APPLI_Id ,LPM_Disable );		
			break;
		}
		case  STATE_GPS_SEND:
		{	

			if(gps.GSA_mode2 == 3)		
  		{
				if(gps.latitude > 0 && gps.longitude > 0)
				{
				  SendData = 1;
				}
			}	
							
      if(SendData == 1)
			{				
			 if(GPS_ALARM == 0)
				{
				  gps_state_on();
					gps_latitude = gps.latitude;
					gps_longitude = gps.longitude;							
					send_data();						
			    a = 1;
					GPS_ALARM = 0;
          SendData = 0;	
					if(LON == 1)
					{
					 LED1_1; 
					}							
					 HAL_Delay(200);
					 LED1_0;
				   HAL_Delay(200);				
				}
				if(GPS_ALARM == 1)
				{
				 if(Alarm_times < 60)
				 {			 
					 gps_state_on();
					 gps_latitude = gps.latitude;
					 gps_longitude = gps.longitude;						 
					 send_ALARM_data();							 
           a = 100;		
           GPS_ALARM = 1;
           SendData = 0;						 
           PRINTF("send data \n\r");					 
				 }
				 if(Alarm_times1 == 60)
				 {
					 start_time=HW_RTC_GetTimerValue();		
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
					 if(LON == 1)
						 {
					    BSP_sensor_Init();								 
					    LED1_1;
						 }							 
					    HAL_Delay(1000);	
					    LED1_0;
					 PPRINTF("Exit Alarm\r\n");
					 DISABLE_IRQ( );
				/*
				 * if an interrupt has occurred after DISABLE_IRQ, it is kept pending
				 * and cortex will not enter low power anyway
				 * don't go in low power mode if we just received a char
				 */
#ifndef LOW_POWER_DISABLE
//		MPU_Write_Byte(MPU9250_ADDR,0x6B,0X40);//MPU sleep
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
				if(GS == 1)
				{			
					ALARM = 1;	
					gps_state_off();				
					start = 1;
					gps.latitude = 0;
					gps.longitude = 0;						
			    APP_TX_DUTYCYCLE = Server_TX_DUTYCYCLE;
			    TimerInit( &TxTimer, OnTxTimerEvent );
	        TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
	        Send( );	
			    TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
          /*Wait for next tx slot*/
          TimerStart( &TxTimer);
					GS = 0;
				}
				
					if(GPS_ALARM == 1)
					{
						if(Alarm_LED == 0)
					{
						gps.flag = 1;
						gps.GSA_mode2 = 0;
						start = 1;	
						ALARM = 1;						
						Alarm_times1 = 0;
						Alarm_times = 60;
						GPS_POWER_OFF();
						BSP_sensor_Init();
						PRINTF("\n\r");	
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
					 }
				  }
					
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
			  PRINTF("Update Interval: %d ms\n\r",APP_TX_DUTYCYCLE);		
        PRINTF("LP == 1\n\r");				
		    lora_state_Led();
				a = 1;

			}	
			
				if(Start_times == TIMES)
				{
					End_times ++;
					Start_times =0;
					gps_time ++ ;
				  if(LON == 1)
				   {
					  LED0_1;
				   }
					TIMES = 10000;
				  if( MD == 0)
				  {
					  MPU_Write_Byte(MPU9250_ADDR,0x6B,0X40);//MPU sleep         
						APP_TX_DUTYCYCLE=Server_TX_DUTYCYCLE;
						TimerInit( &TxTimer, OnTxTimerEvent );
						TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
						/*Wait for next tx slot*/
						TimerStart( &TxTimer);
				  }
					HAL_Delay(200);
				}
				if(End_times < Positioning_time)
				{
					if(gps_time == 30 )
					{
						PRINTF("\r\n");
						PRINTF("Fix Time:%d s \n\r",End_times); 
						PRINTF("Fix Timeout (FTIME):%d s \n\r",Positioning_time);
						gps_time = 0;
					}
       	}	
				
			  if(End_times >=Positioning_time)
				{
				  send_fail=1;	
					if(GPS_ALARM == 0)
					{
						 LED3_0;
						 LED1_0;
						 LED0_0;
						 gps_state_off();	
             a = 100;						
						 send_data();
						 PRINTF("GPS NO FIX\n\r");
						 GPS_ALARM = 0;	
             if(LON == 1)
             {									 
						   LED3_1; 							 
						   HAL_Delay(200);
						   LED3_0;
						   HAL_Delay(200);
							 LED3_1; 							 
						   HAL_Delay(200);
						   LED3_0;
						   HAL_Delay(200);
						 }	 
					}
					if(GPS_ALARM == 1)
					{
					 if(Alarm_times < 60)
					 {
						 LED3_0;
						 LED1_0;
						 LED0_0;
						 gps_state_off();	
						 send_ALARM_data();
						 GPS_ALARM = 1;
						 a = 100;	
						 SendData = 0;
					   PRINTF("GPS NO FIX\n\r");							 
					 }
					 if(Alarm_times1 == 60)
					 {
						 start_time=HW_RTC_GetTimerValue();		
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
						 if(LON == 1)
						 {
							BSP_sensor_Init();								 
						  LED1_1;
						 }							 
						  HAL_Delay(1000);	
						  LED1_0;
					   PPRINTF("Exit Alarm\r\n");						 
						 DISABLE_IRQ( );
					/*
					 * if an interrupt has occurred after DISABLE_IRQ, it is kept pending
					 * and cortex will not enter low power anyway
					 * don't go in low power mode if we just received a char
					 */
#ifndef LOW_POWER_DISABLE
//		MPU_Write_Byte(MPU9250_ADDR,0x6B,0X40);//MPU sleep
		LPM_EnterLowPower();
#endif
						 ENABLE_IRQ();			 
					 }	
					}	
      	 send_fail=0;							
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
	     if(motion_flags==0)
			 {
			 APP_TX_DUTYCYCLE = Server_TX_DUTYCYCLE;
			 }
			 TimerInit( &TxTimer, OnTxTimerEvent );
	     TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
	     Send( );	
			 TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
       /*Wait for next tx slot*/
       TimerStart( &TxTimer);
			 LPM_SetOffMode(LPM_APPLI_Id ,LPM_Disable );
			 PRINTF("Update Interval: %d ms\n\r",APP_TX_DUTYCYCLE);
		   lora_state_Led();
  		 gps.flag = 1;
			 BSP_sensor_Init();
       LED0_0;
			 End_times = 0 ;
			 gps.GSA_mode2 = 0;
			if( MD == 0)
			 {
					MPU_Write_Byte(MPU9250_ADDR,0x6B,0X40);//MPU sleep
			 }
			else
			 {
					MPU_INT_Init();
			 }
       DISABLE_IRQ( );
    /*
     * if an interrupt has occurred after DISABLE_IRQ, it is kept pending
     * and cortex will not enter low power anyway
     * don't go in low power mode if we just received a char
     */
#ifndef LOW_POWER_DISABLE
//		MPU_Write_Byte(MPU9250_ADDR,0x6B,0X40);//MPU sleep
    LPM_EnterLowPower();
#endif
    ENABLE_IRQ();	
}

void send_ALARM_data(void)
{
	
		   start = 1;		
       APP_TX_DUTYCYCLE=Alarm_TX_DUTYCYCLE;	
       TimerInit( &TxTimer, OnTxTimerEvent );			 
			 TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);		    
       Send( );	
			 TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
       /*Wait for next tx slot*/
       TimerStart( &TxTimer);		
			 PRINTF("Update Interval: %d ms\n\r",APP_TX_DUTYCYCLE);
       lora_state_Led();
			 BSP_sensor_Init();
			 End_times = 0 ;
	     LED0_0;
	     a = 100;
       if(LON == 1)
			 {				 
		  	 LED3_1; 
			 }
			   HAL_Delay(1000);
			   LED3_0;

			if( MD == 0)
			 {
					MPU_Write_Byte(MPU9250_ADDR,0x6B,0X40);//MPU sleep
			 }
			else
			 {
					MPU_INT_Init();
			 }		 	
       DISABLE_IRQ( );
    /*
     * if an interrupt has occurred after DISABLE_IRQ, it is kept pending
     * and cortex will not enter low power anyway
     * don't go in low power mode if we just received a char
     */
#ifndef LOW_POWER_DISABLE
//		MPU_Write_Byte(MPU9250_ADDR,0x6B,0X40);//MPU sleep
    LPM_EnterLowPower();
#endif
    ENABLE_IRQ();	
}

/*
*@Features: Quickly get an open countdown
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
*@Features: Fusion accelerometer and magnetometer for attitude adjustment
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
		
           float norm;									//For unitization
           float hx, hy, hz, bx, bz;		//
           float vx, vy, vz, wx, wy, wz; 
           float ex, ey, ez;
//					 float tmp0,tmp1,tmp2,tmp3;
 
           // auxiliary variables to reduce number of repeated operations
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
          
           // normalise the measurements  
           norm = invSqrt(ax*ax + ay*ay + az*az);
           ax = ax * norm;
           ay = ay * norm;
           az = az * norm;
           norm = invSqrt(mx*mx + my*my + mz*mz);
           mx = mx * norm;
           my = my * norm;
           mz = mz * norm;
          
           // compute reference direction of magnetic field 
					 //hx,hy,hz is mx,my,mz Representation in the reference coordinate system
           hx = 2*mx*(0.50 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
           hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.50 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
           hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.50 - q1q1 -q2q2);    
						//bx,by,bz Is the representation of the Earth's magnetic field in the reference coordinate system
           bx = sqrt((hx*hx) + (hy*hy));
           bz = hz;
          
// estimated direction of gravity and magnetic field (v and w) 
//vx,vy,vz Is the representation of gravity acceleration in the object coordinate system
           vx = 2*(q1q3 - q0q2);
           vy = 2*(q0q1 + q2q3);
           vz = q0q0 - q1q1 - q2q2 + q3q3;
					 //wx,wy,wz Is the representation of the earth's magnetic field in the object coordinate system
           wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
           wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
           wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2); 
          
// error is sum ofcross product between reference direction of fields and directionmeasured by sensors 
//ex,ey,ezIt is the error between the acceleration and the measured direction of the magnetometer and the actual gravity acceleration and the geomagnetic direction.
// The error is expressed by the cross product, and the weight of the accelerometer and the magnetometer are the same.
           ex = (ay*vz - az*vy) + (my*wz - mz*wy);
           ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
           ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

           // integral error scaled integral gain
           exInt = exInt + ex*Ki*dt;
           eyInt = eyInt + ey*Ki*dt;
           ezInt = ezInt + ez*Ki*dt;
					// printf("exInt=%0.1f eyInt=%0.1f ezInt=%0.1f ",exInt,eyInt,ezInt);
           // adjusted gyroscope measurements
           gx = gx + Kp*ex + exInt;
           gy = gy + Kp*ey + eyInt;
           gz = gz + Kp*ez + ezInt;
					 //printf("gx=%0.1f gy=%0.1f gz=%0.1f",gx,gy,gz);
          
           // integrate quaernion rate aafnd normalaizle
			
//           tmp0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
//           tmp1 = q1 + ( q0*gx + q2*gz - q3*gy)*halfT;
//           tmp2 = q2 + ( q0*gy - q1*gz + q3*gx)*halfT;
//           tmp3 = q3 + ( q0*gz + q1*gy - q2*gx)*halfT; 
//					 q0=tmp0;
//					 q1=tmp1;
//					 q2=tmp2;
//					 q3=tmp3;
					 //printf("q0=%0.1f q1=%0.1f q2=%0.1f q3=%0.1f",q0,q1,q2,q3);
////RUNGE_KUTTA Solving differential equation
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
*@Features: Calculate the number of turns in the horizontal direction
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
*@Features: Calculate the yaw angle
*
*
*/
void CalYaw(float *yaw,short *turns)
{
	*yaw=360.0**turns+*yaw;
}

/*
*@Features: Compensate for Euler angle offset, mainly to compensate for yaw angle
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
