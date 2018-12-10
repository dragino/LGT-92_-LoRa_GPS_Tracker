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

typedef unsigned char  INT8U; // 无符号8位整型变量 // 
typedef signed char    INT8S; // 有符号8位整型变量 // 
typedef unsigned short INT16U; // 无符号16位整型变量 // 
typedef signed short   INT16S; // 有符号16位整型变量 // 
typedef unsigned int   INT32U; // 无符号32位整型变量 // 
typedef signed int     INT32S; // 有符号32位整型变量 // 
typedef float          FP32; // 单精度浮点数(32位长度) // 
typedef double         FP64; // 双精度浮点数(64位长度) //  
//#define   BOOL     bool
 
typedef	struct 
	{ 
		int satid;      //卫星序号 
		int elevation;  //卫星仰角（00 - 90）度
		int azimuth;    //卫星方位角（00 - 359）度
		int snr;        //信噪比（00－99）dbHz 
	} SatelliteInfo; 

typedef  struct{

    char isvalid;
    int hh,mm,ss,ms;

    int DD, MM, YY;
    FP32 latitude;
    uint8_t latNS;    
    FP32 longitude;
    uint8_t   lgtEW;
    FP32 speed;       //地面速度，GPS输出单位 节，Knots 已经转化位KM/H
    FP32 direction;   //方位角，度 ，以真北为参考
    
	  FP32  flag;
	
    FP32  altitude;     //海拔高度
    uint8_t altitudeunit;       //海拔单位

    uint8_t  FixMode;     //GPS状态，0=未定位，1=非差分定位，2=差分定位，3=无效PPS，6=正在估算
    uint8_t GSA_mode1;//定位模式，A=自动手动2D/3D，M=手动2D/3D 
    uint8_t GSA_mode2;//定位类型，1=未定位，2=2D定位，3=3D定位 

    FP32 PDOP;          //综合位置精度因子
    FP32 HDOP;          //水平精度因子
    FP32 VDOP;          //垂直精度因子 
    uint32_t  ageOfDiff;//差分时间（从最近一次接收到差分信号开始的秒数，如果不是差分定位将为空） 
    uint16_t  diffStationID;//差分站ID号0000 - 1023（前导位数不足则补0，如果不是差分定位将为空）
     
    uint8_t usedsat[12];//正在用来解算的卫星序号
    uint8_t  usedsatnum;  //正在使用的卫星数量（00 - 12）
    uint8_t allsatnum;  //当前可见卫星总数（00 - 12）
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
