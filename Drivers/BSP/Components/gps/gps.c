#include "GPS.h"  
#include "at.h"  

#define SEMICOLON   ','    
#define ASTERISK    '*'    
  
//sscanf()  代码量1K)
//strncpy() 代码量300
// atof()   代码量4300 
// atoi()   代码量600  

  GPSINFO  gps;  
  char lasttime[20]; 
  _Bool isrunning; 
  uint32_t   isFirmwareUpdate = 0; 
	int count =0;


_Bool GPS_Run(void)   
{   
    if( isrunning)   
        return 0;
           
    isrunning  = 1;
    GPS_init();   
   GPS_STANDBY_L();
   // Delay_ms(100);
   GPS_RESET_OFF();
	 GPS_POWER_ON();

    return isrunning;   
}   
   
void GPS_Stop(void)   
{   

       // GPS_POWER_OFF();  //关闭GPS电源
       // GPS_BOOT_H();
        isrunning=0;   
     
}  


void GPS_FirmwareUpdate(void)   
{   
        isFirmwareUpdate = 1;
        //GPS_POWER_OFF();  //关闭GPS电源
        //Delay_ms(1000);
       // GPS_BOOT_H();
       // Delay_ms(1000);
        //GPS_POWER_ON();  //开启GPS电源
         
       // USART1_Configuration(38400);
	     // USART2_Configuration(38400);
     
}  
   
_Bool GPS_IsRunning(void)   
{   
    return isrunning;   
}   
 /*   
BOOL GPS_IsLocationValid()   
{   
    return (code != 0);   
}   
   
FP32 GPS_GetTimestamp()   
{   
    return 0;   
}   
   
FP32 GPS_GetLongitude()   
{   
    return longitude;   
}   
   
FP32 GPS_GetLatitude()   
{   
    return latitude;   
}   
   
FP32 GPS_GetHeight()   
{   
    return height;   
}   
   
char GPS_GetHeightUnit()   
{   
    return heightunit;   
}   
   
FP32 GPS_GetVelocity()   
{   
    return velocity;   
}   
   
FP32 GPS_GetDirection()   
{   
    return direction;   
}   
   
int GPS_GetSatNum()   
{   
    return satnum;   
}   
   
FP32 GPS_GetError()   
{   
    return dis;   
}   
   
 */  
void GPS_DegreeToDMS(FP32 deg,int *d,int *m,FP32 *s)   
{   
    
   *d = (int)deg;   
   *m = (int)((deg - *d ) * 60);   
   *s = (((deg-*d)*60)-*m)*60;   
   if(60.0-*s<0.01)   
   {   
       *s = 0.0;   
       *m ++;   
       if(*m == 60)   
       {   
           *d++;   
           *m = 0;   
       }   
   }   
   
}  
    
/*
FP32 CCrd::RadianToDMS(FP32 r)   
{   
   FP32 t = r*180/PI;   
   int d = (int)t;   
   int m = (int)((t - d ) * 60);   
   FP32 s = (((t-d)*60)-m)*60;   
   if(60.0-s<0.01)   
   {   
       s = 0.0;   
       m ++;   
       if(m = 60)   
       {   
           d++;   
           m = 0;   
       }   
   }   
   return ((FP32)d+(FP32)m*0.01+(FP32)s*0.0001);   
}   
   
FP32 CCrd::DMSToRadian(FP32 dms)   
{   
    int d = (int)dms;   
    int m = (int)((dms -d) * 100);   
    FP32 s = ((dms - d) * 100 - m) * 100;   
    return ((FP32)d+(FP32)m/60+s/3600)*PI/180;   
} 
*/


int my_atoi(const char *str)
{
    int result = 0;
    int signal = 1; /* 默认为正数 */
    if((*str>='0'&&*str<='9')||*str=='-'||*str=='+')
    {
        if(*str=='-'||*str=='+')
        {   if(*str=='-')
	        signal = -1; /* 输入负数 */ 
	        str++;
         }
    }
    else return 0; 
   
   /* 开始转换 */
   while(*str>='0'&&*str<='9')
        result = result*10+(*str++ -'0'); 

   return signal*result;
}

/*

   这个函数是把浮点数字符串转换为浮点数的函数。

   函数将会跳过字符串中的空格字符和不是'+'、'-'、'.'、

   数字的字符。如果字符串是空的或者都是由空格组成，将不会

   做任何转换，仅仅是把字符串的结束地址赋给endptr。如果字

   符串合法，将会进行转换，并把字符串最后的NULL的地址给

   endptr。如果你想使用endptr参数，那么赋一个NULL值就

   可以了。

*/

FP32 my_strtod(const char* s, char** endptr) 

{

   const char*  p     = s;

    FP32  value = 0.L;

    int                   sign  = 0;

    FP32           factor;

    unsigned int          expo;

   

    while ( isspace(*p) )//跳过前面的空格

      p++;

 

    if(*p == '-' || *p == '+')

      sign = *p++;//把符号赋给字符sign，指针后移。

   

   //处理数字字符

 

    while ( (unsigned int)(*p - '0') < 10u )//转换整数部分

      value = value*10 + (*p++ - '0');

   //如果是正常的表示方式（如：1234.5678）

   if ( *p == '.' ) 

   {

        factor = 1.;

        p++;

        while ( (unsigned int)(*p - '0') < 10u ) 

      {

         factor *= 0.1;

            value  += (*p++ - '0') * factor;

        }

    }

   //如果是IEEE754标准的格式（如：1.23456E+3）

    if ( (*p | 32) == 'e' ) 

   {

      expo   = 0;

        factor = 10.L;

        switch (*++p)

      { 

        case '-': 

           factor = 0.1;

        case '+': 

           p++;

           break;

        case '0': 

        case '1': 

        case '2':

        case '3':

        case '4': 

        case '5': 

        case '6': 

        case '7': 

        case '8': 

        case '9':

           break;

        default : 

           value = 0.L;

           p     = s;

           goto done;

        }

        while ( (unsigned int)(*p - '0') < 10u )

            expo = 10 * expo + (*p++ - '0');

        while ( 1 )

      {

        if ( expo & 1 )

           value *= factor;

            if ( (expo >>= 1) == 0 )

                break;

            factor *= factor;

        }

    }

done:

    if ( endptr != 0 )

        *endptr = (char*)p;

 

    return (sign == '-' ? -value : value);

}

FP32 my_atof(char *str)

{

   return my_strtod(str,0);

}



char *match(char *str,char *patten)   
{   
    while(*str != 0 && *patten != 0 && *str == *patten)   
    {   
        str++;   
        patten++;   
    }   
   
    if(*patten == 0)   
        return str+1;   
    else   
        return NULL;   
}   
   
char *next_fld(char *str)   
{   
    char *ts=str;   
   
    while(*ts != ',' && *ts != 0)   
        ts++;   
    *ts='\0';   
   
    return str;   
} 
char *split(char *buf,char s,char **left)   
{   

    char *p=buf,*ret=buf;
      
    if(buf == NULL || buf[0] == 0)   
    {   
        *left=NULL;   
        return NULL;   
    }   
   
 
    while(*p != 0 && *p != s && *p != '\r' && *p != '\n')   
    {   
        p++;   
    }   
   
    if(*p != 0)   
    {   
        *left=p+1;   
        *p=0;   
    }   
    else   
    {   
        *left=NULL;   
    }   
   
    return ret;   
}   
   
unsigned char digits[16]={ '0','1','2','3','4','5','6','7','8','9',   
                           'A','B','C','D','E','F' };   
int check(char *sentence,char *cksum)   
{   

    unsigned char *p=(unsigned char *)sentence,sum=0/*,ts*/; 
    if(sentence == NULL || cksum == NULL)   
        return 0;   
   
  
    for(; *p != 0; p++)   
    {   
        sum ^= *p;   
    }   
   
    if(digits[sum >> 4] == cksum[0] && digits[sum & 0x0f] == cksum[1])   
        return 1;   
/*  
    sscanf(cksum,"%x",&ts);  
    if(sum != ts)  
        return 0;  
*/   
    return 0;   
}   
   
uint8_t GPS_parse(char *buf)   
{   
    //printf("%s\n",buf);    
   int d,m,mm;
    uint8_t i;
   char *word,*left=buf+1;    
   static uint8_t msgcount=0,msgid=0,satcount=0;   //解析GSV用到的变量
           //各通道采用的卫星编号    
    uint8_t usedsatcount=0;

    if(buf[0] != '$')   
        return 0;   
   

   
    word=split(left,ASTERISK,&left);   
    if(check(word,left) != 1)   
        return 0;   
   
    left=word;   
   
    word=split(left,SEMICOLON,&left);   
    if(!strcmp(word,"GPRMC"))   
    {   
        //时间戳    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            if(strncmp(lasttime,word,20))   
            {   
                sscanf(word,"%2d%2d%2d.%4d",&gps.hh,&gps.mm,&gps.ss,&gps.ms);   
                strncpy(lasttime,word,20);   
            }   
        }   
   
        //定位状态    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            if(word[0] == 'A')   
                gps.isvalid=1;   
            else   
                gps.isvalid=0;   
        }   
   
        //纬度    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
             
            sscanf(word,"%2d%2d.%4d",&d,&m,&mm);   
            gps.latitude=(float)d+(float)m/60.0+(float)mm/600000.0;   
        }   
   
        //南北半球标志    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            if(word[0] == 'S')   
              { // gps.latitude=-gps.latitude; 
                 gps.latNS = 'S';
              }else{
                 gps.latNS = 'N';
              } 
         }       

   
        //经度    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            int d,m,mm;   
            sscanf(word,"%3d%2d.%4d",&d,&m,&mm);   
            gps.longitude = (float)d+(float)m/60.0+(float)mm/600000.0;   
        }   
   
        //东西半球标志    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            if(word[0] == 'W')
            {   
                gps.lgtEW = 'W';
               // gps.longitude=-gps.longitude;
            } else{
                gps.lgtEW = 'E';
            }  
        }   
   
        //对地运动速度    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            gps.speed=my_atof(word)*1.852;   
        }   
   
        //对地运动方向    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            gps.direction=my_atof(word);   
        } 
        
         //utc 日期    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        { 
          sscanf(word,"%2d%2d%2d",&gps.DD,&gps.MM,&gps.YY);    
        }  
    }   
    else if(!strcmp(word,"GPGGA"))   
    {   
        //时间戳    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            if(strncmp(lasttime,word,20))   
            {   
                sscanf(word,"%2d%2d%2d.%4d",&gps.hh,&gps.mm,&gps.ss,&gps.ms);   
                strncpy(lasttime,word,20);   
            }   
        }   
   
        //纬度    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            int d,m,mm;   
            sscanf(word,"%2d%2d.%4d",&d,&m,&mm);   
            gps.latitude=(float)d+(float)m/60.0+(float)mm/600000.0;   
        }   
   
        //南北半球标志    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            if(word[0] == 'S')   
              { // gps.latitude=-gps.latitude; 
                 gps.latNS = 'S';
              }else{
                 gps.latNS = 'N';
              } 
         }    
   
        //经度    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            int d,m,mm;   
            sscanf(word,"%3d%2d.%4d",&d,&m,&mm);   
            gps.longitude=(float)d+(float)m/60.0+(float)mm/600000.0;   
        }   
   
        //东西半球标志    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            if(word[0] == 'W')
            {   
                gps.lgtEW = 'W';
               // gps.longitude=-gps.longitude;
            } else{
                gps.lgtEW = 'E';
            }  
        }   
   
        //定位有效性及格式    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            gps.FixMode=my_atoi(word);   
        }   
   
        //捕捉卫星数量    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            gps.usedsatnum=my_atoi(word);   
        }   
   
        //估计误差    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            gps.HDOP=my_atof(word); 
            
            //sscanf(word,"%f",&gps.HDOP);  
        }   
   
        //海拔高度    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            gps.altitude=my_atof(word);   
        }   
   
        //高度单位    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            gps.altitudeunit=word[0];   
        }   
    }   
    else if(!strcmp(word,"GPGSV"))   
    {   
        
   
        //消息总数    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            msgcount=my_atoi(word);   
        }   
   
        //消息编号    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            msgid=my_atoi(word);   
        }   
   
        if(msgid == 1)   
        {   
            satcount=0;   
            //memset(satinfo,0,sizeof(SatelliteInfo)*38);   
        }   
   
        //卫星总数    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            gps.allsatnum=my_atoi(word);   
        }   
   
       // printf("%s\n",left);   
        for(i=0;i<4;i++)   
        {   
            //卫星编号    
            word=split(left,SEMICOLON,&left);   
            if(word != NULL)   
            {   
                gps.satinfo[satcount].satid=my_atoi(word);   
            }   
   
            //卫星仰角    
            word=split(left,SEMICOLON,&left);   
            if(word != NULL)   
            {   
                gps.satinfo[satcount].elevation=my_atoi(word);   
            }   
   
            //卫星方位角    
            word=split(left,SEMICOLON,&left);   
            if(word != NULL)   
            {   
                gps.satinfo[satcount].azimuth=my_atoi(word);   
            }   
   
            //卫星信号信噪比    
            word=split(left,SEMICOLON,&left);   
            if(word != NULL)   
            {   
                gps.satinfo[satcount].snr=my_atoi(word);   
            }   
   
            satcount++;   
   
            if(word == NULL)   
                break;   
        }   
   
        if(msgid == msgcount )   
          ;   
    }   
    else if(!strcmp(word,"GPGSA"))   
    {   
        //定位模式1    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            gps.GSA_mode1=my_atoi(word);   
        }   
   
        //定位模式2    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            gps.GSA_mode2=my_atoi(word);   
        }   
   

          
        for(i=0;i<12;i++)   
        {   
            word=split(left,SEMICOLON,&left);   
            if(word != NULL)   
            {   
                gps.usedsat[i]=my_atoi(word);   
                usedsatcount++;   
            }   
        }   
   
        //位置精度值    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            gps.PDOP=(float)my_atof(word);   
        }   
   
        //水平精度值    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            gps.HDOP=(float)my_atof(word);   
        }   
   
        //高度精度值    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            gps.VDOP=(float)my_atof(word);   
        }       
    }  
    
    if(gps.latitude > 90.0)
      gps.latitude = 0.0;

        if(gps.longitude > 180.0)
      gps.longitude = 0.0;
   
    return 1;   
}

#define  NEMA_NUM_MAX   6     //缓冲的NEMA语句数量
#define  NEMA_CHAR_MAX  255   //缓冲的NEMA语句字符数量 

struct {
       uint8_t   isupdated;  //NEMA 缓冲区更新标志
      char buffer[NEMA_CHAR_MAX]; //NEMA 缓冲区
      }GPS_NEMA[NEMA_NUM_MAX] ;    



 void  GPS_usart( uint8_t buffer)
  {
	
    static  uint8_t NEMA_count = 0;
    static  uint8_t char_count = 0;
		 uint8_t Empty = 0;
    if(isFirmwareUpdate == 1)     //更新固件
      return;
    if(buffer == '$')
      { 			
         GPS_NEMA[NEMA_count].isupdated = 1;  //将上面一条语句打上更新标志
         NEMA_count++;
         if(NEMA_count > (NEMA_NUM_MAX-1))
         {
            NEMA_count = 0;
         }
         GPS_NEMA[NEMA_count].isupdated = 0;  //将本条语句打上未更新标志
         GPS_NEMA[NEMA_count].buffer[0] = '$';
         char_count = 1;

       }
			else
				{
         if(char_count < NEMA_CHAR_MAX-1)
          GPS_NEMA[NEMA_count].buffer[char_count++] = buffer;
        }
			count ++;	
		if(count == 255)
		{
			GPS_NEMA[NEMA_count].isupdated = 0  ;
			GPS_NEMA[NEMA_count].buffer[char_count++] = 0  ;
			count = 0;
		}
  }
uint8_t GPS_INFO_update(void)
{ 
     uint8_t i;
     uint8_t temp = 0;
    for(i=0;i<NEMA_NUM_MAX;i++)
    {
      if(GPS_NEMA[i].isupdated == 1)
        {	
            temp = GPS_parse(GPS_NEMA[i].buffer); 
            GPS_NEMA[i].isupdated = 0  ;	
        }       
    }
    
  return temp;
}



//======================================================================
//函 数 名: GPS_init() 
//功    能: 配置LED管脚
//入口参数: 无
//出口参数: 无
//返 回 值: 无
//======================================================================
 void GPS_init(void)
{ 			
    GPIO_InitTypeDef GPIO_InitStructure;  
											   

	/* 配置 PC.0 GPS_BOOT -PC.1 GPS_EN 为输出模式*/
    
   __GPIOB_CLK_ENABLE();

    GPIO_InitStructure.Pin =   GPIO_PIN_3 ;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull  = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);     
	
	  GPIO_InitStructure.Pin =   GPIO_PIN_4 ;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull  = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);  
	  
    GPIO_InitStructure.Pin =   GPIO_PIN_5 ;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull  = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);  
    
    GPS_Stop();  
}

 void GPS_doinit(void)
{ 			
    GPIO_InitTypeDef GPIO_InitStructure;  
											   

	/* 配置  为输入模式*/
    
   __GPIOB_CLK_ENABLE();

    GPIO_InitStructure.Pin =   GPIO_PIN_3 ; 
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
//    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);     
	
	  GPIO_InitStructure.Pin =   GPIO_PIN_4 ;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
//    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);  
	  
    GPIO_InitStructure.Pin =   GPIO_PIN_5 ;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
//    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);  
 
}

void GPS_INPUT(void)
{
	  int dd,mm;
    FP32 ss;
	
	  GPS_INFO_update();
//    GPS_DegreeToDMS(gps.latitude, &dd, &mm,&ss);
	
//    AT_PRINTF("%s:%3d %2d'%5.2f ",(gps.latNS == 'N')?"北纬":"南纬",dd, mm, ss);
//    AT_PRINTF("%s: %.6f度\n\r",(gps.latNS == 'N')?"北纬":"南纬",gps.latitude);

//    GPS_DegreeToDMS(gps.longitude, &dd, &mm,&ss);
//    AT_PRINTF("%s:%3d %2d'%05.2f\n\r ",(gps.lgtEW == 'E')?"东经":"西经",dd, mm, ss);
//	  AT_PRINTF_F("%s: %.6f度\n\r ",(gps.lgtEW == 'E')?"东经":"西经",gps.longitude);
//    AT_PRINTF_F("海拔:%.1f%c    ",gps.altitude,gps.altitudeunit);
//    AT_PRINTF_F("速度:%.1f km/h    ",gps.speed);
//    AT_PRINTF_F("航向:%.1f度    ",gps.direction);
//    AT_PRINTF_F("时间:%2d:%02d:%02d ",(gps.hh<16)?gps.hh+8:gps.hh-16,gps.mm,gps.ss);   
//    AT_PRINTF_F("日期:20%02d-%d-%d  ",gps.YY,gps.MM,gps.DD); 
//    AT_PRINTF_F("卫星:%2d/%2d\n\r",gps.usedsatnum,gps.allsatnum);

    switch(gps.FixMode)
    {
        case 0:
					//AT_PRINTF_F("GPS状态:未定位   \n\r");
				break;
        case 1: 
//					AT_PRINTF_F("GPS状态:%dD SPS  \n\r ",gps.GSA_mode2);
				break;
        case 2:
//					AT_PRINTF_F("GPS状态:%dD DGPS  \n\r",gps.GSA_mode2);
				break;
        case 6:
//					AT_PRINTF_F("GPS状态:估算中    \n\r");
				break;
        default :break;                                                 
    }
    if(gps.GSA_mode2 == 2 || gps.GSA_mode2 == 3)	
		{
		  gps.flag = 0;
		} 
}
void POWER_ON()
{
   GPS_init();
	 GPS_POWER_ON();
}
void POWER_OFF()
{
	 GPS_POWER_OFF();
}
