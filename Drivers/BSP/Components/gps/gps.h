#ifndef __GPS_H_
#define __GPS_H_

#include <string.h>
#include <stdlib.h> 
#include <math.h>  
#include <stdio.h>  
#include "stm32l072xx.h"
#include "stm32l0xx_hal.h"
#include "stm32l0xx_nucleo.h"

#include "bsp_usart2.h"


#ifndef NULL
#define NULL    ((void *)0)
#endif


#define GPS_STANDBY_H()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)		
#define GPS_STANDBY_L()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)

#define GPS_RESET_ON()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET)		
#define GPS_RESET_OFF()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET)

#define GPS_POWER_ON()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET)		
#define GPS_POWER_OFF()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET)

typedef unsigned char  INT8U; // �޷���8λ���ͱ��� // 
typedef signed char    INT8S; // �з���8λ���ͱ��� // 
typedef unsigned short INT16U; // �޷���16λ���ͱ��� // 
typedef signed short   INT16S; // �з���16λ���ͱ��� // 
typedef unsigned int   INT32U; // �޷���32λ���ͱ��� // 
typedef signed int     INT32S; // �з���32λ���ͱ��� // 
typedef float          FP32; // �����ȸ�����(32λ����) // 
typedef double         FP64; // ˫���ȸ�����(64λ����) //  
//#define   BOOL     bool
 
typedef	struct 
	{ 
		int satid;      //������� 
		int elevation;  //�������ǣ�00 - 90����
		int azimuth;    //���Ƿ�λ�ǣ�00 - 359����
		int snr;        //����ȣ�00��99��dbHz 
	} SatelliteInfo; 

typedef  struct{

    char isvalid;
    int hh,mm,ss,ms;

    int DD, MM, YY;
    FP32 latitude;
    uint8_t latNS;    
    FP32 longitude;
    uint8_t   lgtEW;
    FP32 speed;       //�����ٶȣ�GPS�����λ �ڣ�Knots �Ѿ�ת��λKM/H
    FP32 direction;   //��λ�ǣ��� �����汱Ϊ�ο�
    
	  FP32  flag;
	
    FP32  altitude;     //���θ߶�
    uint8_t altitudeunit;       //���ε�λ

    uint8_t  FixMode;     //GPS״̬��0=δ��λ��1=�ǲ�ֶ�λ��2=��ֶ�λ��3=��ЧPPS��6=���ڹ���
    uint8_t GSA_mode1;//��λģʽ��A=�Զ��ֶ�2D/3D��M=�ֶ�2D/3D 
    uint8_t GSA_mode2;//��λ���ͣ�1=δ��λ��2=2D��λ��3=3D��λ 

    FP32 PDOP;          //�ۺ�λ�þ�������
    FP32 HDOP;          //ˮƽ��������
    FP32 VDOP;          //��ֱ�������� 
    uint32_t  ageOfDiff;//���ʱ�䣨�����һ�ν��յ�����źſ�ʼ��������������ǲ�ֶ�λ��Ϊ�գ� 
    uint16_t  diffStationID;//���վID��0000 - 1023��ǰ��λ��������0��������ǲ�ֶ�λ��Ϊ�գ�
     
    uint8_t usedsat[12];//��������������������
    uint8_t  usedsatnum;  //����ʹ�õ�����������00 - 12��
    uint8_t allsatnum;  //��ǰ�ɼ�����������00 - 12��
   SatelliteInfo satinfo[38];
     }GPSINFO;




extern  GPSINFO  gps; 

  
extern void GPS_init(void);
extern  uint8_t   GPS_parse(char *buf); 
extern void  GPS_usart(uint8_t buffer);  
extern  uint8_t GPS_INFO_update(void);
extern _Bool GPS_Run(void);
extern void GPS_Stop(void); 
extern void GPS_FirmwareUpdate(void);
extern void GPS_DegreeToDMS(FP32 deg,int *d,int *m,FP32 *s)   ;
extern void GPS_INPUT(void);
extern void POWER_ON(void);
extern void POWER_OFF(void); 
extern void GPS_doinit(void);
//	BOOL OpenDevice(TCHAR *strPort,int nBaudRate); 
//	void CloseDevice(); 
extern  _Bool GPS_IsRunning(void); 
void send_setting(void);
void PMTK353(void);
void PMTK886(void);
void copytxdata(uint8_t data1[],char *data2);
/*	BOOL Run(); 
	void Stop(); 
	
 
	BOOL IsLocationValid(); 
	FP32 GetTimestamp(); 
	FP32 GetLongitude(); 
	FP32 GetLatitude(); 
	FP32 GetHeight(); 
	char GetHeightUnit(); 
	FP32 GetVelocity(); 
	FP32 GetDirection(); 
	int GetSatNum(); 
	FP32 GetError(); 
 
  */



#endif   //__GPS_H_
