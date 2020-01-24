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
#include "mpu9250.h"
#include "filter.h"
#include "lora.h"
#include "flash_eraseprogram.h"
#include "delay.h"

//static float gyro_offsetx=0.005199,gyro_offsety=0.035498,gyro_offsetz=0.005152;
float tmp1,tmp2,tmp3;
float magoffsetx=1.31454428611172,magoffsety=-1.21753632395713,magoffsetz=1.6567777185719;
float B[6]={0.980358187761106,-0.0105514731414606,0.00754899338354401,0.950648704823113,-0.0354995317649016,1.07449478456729};
//float accoffsetx=0.096212,accoffsety=-0.028414,accoffsetz=0.030577;

static float accoffsetx=-0.030587,accoffsety=-0.032310,accoffsetz=0.046341;

float accsensx=1.00851297697413,accsensy=0.991366909333871,accsensz=1.00019364448499;

#define filter_high 0.8
#define filter_low 	0.2
short accoldx,accoldy,accoldz;
short magoldx,magoldy,magoldz;
short gyrooldx,gyrooldy,gyrooldz;

extern uint32_t MD ;
extern uint32_t Threshold ;
extern uint32_t Freq ;
extern uint32_t Server_TX_DUTYCYCLE;

uint8_t TF[2]={0x9F,0x07};

uint8_t MPU_Init(void)
{
    uint8_t res=0;
   
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X80);//reset MPU9250
    DelayMs(100);  //��ʱ100ms
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X00);//wake MPU9250
    MPU_Set_Gyro_Fsr(3);					        	//Gyro sensor,��2000dps
	  MPU_Set_Accel_Fsr(2);					       	 	//Acceleration gyro sensor,��8g
    MPU_Set_Rate(200);						       	 	//Set sampling frequency 200Hz
    MPU_Write_Byte(MPU9250_ADDR,MPU_INT_EN_REG,0X00);   //Close all interrupts	
	
	 MPU_Write_Byte(MPU9250_ADDR,MPU_USER_CTRL_REG,0X00);//I2C Main mode off
	 MPU_Write_Byte(MPU9250_ADDR,MPU_FIFO_EN_REG,0X00);	// off FIFO
	 MPU_Write_Byte(MPU9250_ADDR,MPU_INTBP_CFG_REG,0X00);//The INT pin is active low,the bypass mode is enabled,and the magnetometer can be read directly.
   res=MPU_Read_Byte(MPU9250_ADDR,MPU_DEVICE_ID_REG);  //Read the ID of MPU9250
    if(res==MPU6500_ID1||res==MPU6500_ID2) //Correct ID
    {
        MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X01);  	//Set CLKSEL,PLL, X axis as reference
			DelayMs(50);
        MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//Acceleration and gyroscope work
		MPU_Set_Rate(200);						       	//Set sampling frequency 200Hz   
    }else return 1;
 
    res=MPU_Read_Byte(AK8963_ADDR,MAG_WIA);    			//Read the ID of AK8963  
    if(res==AK8963_ID)
    {
        MPU_Write_Byte(AK8963_ADDR,MAG_CNTL2,0X01);		//Reset AK8963
		DelayMs(50);
        MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11);		//Set AK8963 as a single measurement
			
    }else return 1;
		
    LPF2pSetCutoffFreq_1(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);		//30Hz
    LPF2pSetCutoffFreq_2(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);
    LPF2pSetCutoffFreq_3(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);
    LPF2pSetCutoffFreq_4(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);
    LPF2pSetCutoffFreq_5(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);
    LPF2pSetCutoffFreq_6(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);

//		calibrate();
    return 0;
}

void MPU_INT_Init(void)
{
  MPU_Write_Byte(MPU9250_ADDR,0x6B,0X00);//wake
//  MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//Acceleration and gyroscope work
	
	MPU_Write_Byte(MPU9250_ADDR,MPU_INTBP_CFG_REG,0X00);
	MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X00);
	MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X07);
	MPU_Write_Byte(MPU9250_ADDR,MPU_ACCEL_CFG2_REG,0X09);
	MPU_Write_Byte(MPU9250_ADDR,MPU_INT_EN_REG,0X40);
	MPU_Write_Byte(MPU9250_ADDR,MPU_MDETECT_CTRL_REG,0XC0);
//	MPU_Write_Byte(MPU9250_ADDR,MPU_MOTION_DET_REG,0X18);
//	MPU_Write_Byte(MPU9250_ADDR,MPU_LP_ACCEL_ODR_REG,0X05);
	 if(MD == 1)
	{
	  MPU_Write_Byte(MPU9250_ADDR,MPU_MOTION_DET_REG,0X0C);
	  MPU_Write_Byte(MPU9250_ADDR,MPU_LP_ACCEL_ODR_REG,0X02);	
	}
	else if(MD == 2)
	{
	  MPU_Write_Byte(MPU9250_ADDR,MPU_MOTION_DET_REG,0X9F);
	  MPU_Write_Byte(MPU9250_ADDR,MPU_LP_ACCEL_ODR_REG,0X07);		
	}		
	else if(MD == 3)
	{
		Read_Config();
		TF[0]=Threshold;
		TF[1]=Freq;
		MPU_Write_Byte(MPU9250_ADDR,MPU_MOTION_DET_REG,TF[0]);
	  MPU_Write_Byte(MPU9250_ADDR,MPU_LP_ACCEL_ODR_REG,TF[1]);			
	}		
	MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X20);
//	DelayMs(200);
}
//Set the MPU9250 gyro sensor full-scale range
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//Return value: 0,set successfully
//          Other,setup failed
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR,MPU_GYRO_CFG_REG,(fsr<<3)|3);//���������������̷�Χ  
}
//Set the MPU9250 accelerometer full-scale range
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//Return value: 0,set successfully
//          Other,setup failed
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ  
}

//Set the digital low-pass filter of the MPU9250
//lpf:digital low-pass filter frequency(Hz)
//Return value: 0,set successfully
//          Other,setup failed
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU9250_ADDR,MPU_CFG_REG,data);//Set the digital low-pass filter
}

//Set the sampling frequency of the MPU9250(assumed Fs=1KHz)
//rate:4~1000(Hz)
//Return value: 0,set successfully
//          Other,setup failed
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU9250_ADDR,MPU_SAMPLE_RATE_REG,data);	//Set the digital low-pass filter
 	return MPU_Set_LPF(rate/2);	//Automatically set LPF to half of the sampling frequency
}

/*
*@Function: compensate for gyroscope drift
*
*
*/
void calibrate1(void)
{
	uint16_t t;
	short igx1,igy1,igz1;
	short iax1,iay1,iaz1;
	float ax2,ay2,az2;
	float gx2,gy2,gz2,sumx=0,sumy=0,sumz=0,sumx1=0,sumy1=0,sumz1=0;
	
	MPU_Write_Byte(MPU9250_ADDR,0x6B,0X00);//wake
  MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X00);
	
	for (t=0;t<300;t++)
	{
		MPU_Get_Gyro(&igx1,&igy1,&igz1,&gx2,&gy2,&gz2);
		MPU_Get_Accel(&iax1,&iay1,&iaz1,&ax2,&ay2,&az2);
		
		sumx=sumx+gx2;
		sumy=sumy+gy2;
		sumz=sumz+gz2;
		
		sumx1=sumx1+ax2;
		sumy1=sumy1+ay2;
		sumz1=sumz1+az2;
	}
//	gyro_offsetx=sumx/300.0;
//	gyro_offsety=sumy/300.0;
//	gyro_offsetz=sumz/300.0;
	
	accoffsetx=sumx1/300.0;
	accoffsety=sumy1/300.0;
	accoffsetz=sumz1/300.0-1.0;
	
//	gyro_offsetx=0;
//	gyro_offsety=0;
//	gyro_offsetz=0;
	
	accoffsetx=0;
	accoffsety=0;
	accoffsetz=0;
}

void calibrate2(void)
{
	uint16_t t;
	short igx1,igy1,igz1;
	short iax1,iay1,iaz1;
	float ax2,ay2,az2;
	float gx2,gy2,gz2,sumx=0,sumy=0,sumz=0,sumx1=0,sumy1=0,sumz1=0;  	
	
	for (t=0;t<300;t++)
	{
		MPU_Get_Gyro(&igx1,&igy1,&igz1,&gx2,&gy2,&gz2);
		MPU_Get_Accel(&iax1,&iay1,&iaz1,&ax2,&ay2,&az2);
		
		sumx=sumx+gx2;
		sumy=sumy+gy2;
		sumz=sumz+gz2;
		
		sumx1=sumx1+ax2;
		sumy1=sumy1+ay2;
		sumz1=sumz1+az2;
	}
	MPU_Write_Byte(MPU9250_ADDR,0x6B,0X40);//mpu sleep
//	gyro_offsetx=sumx/300.0;
//	gyro_offsety=sumy/300.0;
//	gyro_offsetz=sumz/300.0;
	
	accoffsetx=sumx1/300.0;
	accoffsety=sumy1/300.0;
	accoffsetz=sumz1/300.0-1.0;
	
	long j1=accoffsetx*1000000/1;
	long j2=accoffsety*1000000/1;
	long j3=accoffsetz*1000000/1;
	
	FLASH_erase(0x801F480);//Page1001
	FLASH_program_on_addr(0x801F484,j1);
	FLASH_program_on_addr(0x801F488,j2);
	FLASH_program_on_addr(0x801F48C,j3);
	
	PRINTF("acc=%f ",accoffsetx);
	PRINTF("%f ",accoffsety);
	PRINTF("%f ", accoffsetz);
	
	PRINTF("\n\rcalibrating...\n\r");
}

void get_calibrated_data(void)
{
	long j1=FLASH_read(0x801F484);
	long j2=FLASH_read(0x801F488);
	long j3=FLASH_read(0x801F48C);
	
	accoffsetx=j1/1000000.0;
	accoffsety=j2/1000000.0;
	accoffsetz=j3/1000000.0;
	
	PRINTF("acc=%f ",accoffsetx);
	PRINTF("%f ",accoffsety);
	PRINTF("%f\n\r", accoffsetz);
}

//Get the temperature value
//Return value:temperature value (expanded by 100 times)
short MPU_Get_Temperature(void)
{
    uint8_t buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU9250_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((uint16_t)buf[0]<<8)|buf[1];  
    temp=21+((double)raw)/333.87;  
    return temp*100;;
}

/////////////////////////////////
//Get the gyro value(original value),filter the average of the source data,and adjust the gyroscope orientation
//gx,gy,gz:Raw data of the gyroscope X, Y, Z axis (with symbol)
//Return value: 0,set successfully
//          Other,error code
uint8_t MPU_Get_Gyroscope(short *gy,short *gx,short *gz)
{
    uint8_t buf[6],res; 
	res=MPU_Read_Len(MPU9250_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gy=(((uint16_t)buf[0]<<8)|buf[1]);  
		*gx=(((uint16_t)buf[2]<<8)|buf[3]);  
		*gz=(((uint16_t)buf[4]<<8)|buf[5]);
//		*gx=-*gx;
//		*gx=(short)(gyrooldx*0.5+*gx*0.5);
//		*gy=(short)(gyrooldy*0.5+*gy*0.5);
//		*gz=(short)(gyrooldz*0.5+*gz*0.5);
//		gyrooldx=*gx;
//		gyrooldy=*gy;
//		gyrooldz=*gz;
		
	} 	
    return res;
}
/*
*@Function: Get gyroscope data in radians per second
*
*
*/
uint8_t MPU_Get_Gyro(short *igx,short *igy,short *igz,float *gx,float *gy,float *gz)
{
	uint8_t res;
	res=MPU_Get_Gyroscope(igx,igy,igz);
	if (res==0)
	{
	*gx=LPF2pApply_4((float)(*igx)*gryo_scale);
	*gy=LPF2pApply_5((float)(*igy)*gryo_scale);
	*gz=LPF2pApply_6((float)(*igz)*gryo_scale);
	}
	return res;
}


//Get the acceleration value(original value),low pass filter and adjust acceleration orientation
//ax,ay,az:Raw data of the acceleration X, Y, Z axis (with symbol)
//Return value: 0,set successfully
//          Other,error code
uint8_t MPU_Get_Accelerometer(short *ay,short *ax,short *az)
{
    uint8_t buf[6],res;  
	res=MPU_Read_Len(MPU9250_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ay=(((uint16_t)buf[0]<<8)|buf[1]);  
		*ax=(((uint16_t)buf[2]<<8)|buf[3]);  
		*az=(((uint16_t)buf[4]<<8)|buf[5]);
		*ax=-*ax;
//		*ax=(short)(accoldx*filter_high+*ax*filter_low);
//		*ay=(short)(accoldy*filter_high+*ay*filter_low);
//		*az=(short)(accoldz*filter_high+*az*filter_low);
//		accoldx=*ax;
//		accoldy=*ay;
//		accoldz=*az;
		
	} 	
    return res;
}

/*
*@Function: Obtain accelerometer data in g and conpensate for accelerometer
*
*
*/
uint8_t MPU_Get_Accel(short *iax,short *iay,short *iaz,float *ax,float *ay,float *az)
{
	uint8_t res;
	res=MPU_Get_Accelerometer(iax,iay,iaz);
	if (res==0)
	{
//	tmp1=(float)(*iax)*accel_scale-accoffsetx;
//	tmp2=(float)(*iay)*accel_scale-accoffsety;
//	tmp3=(float)(*iaz)*accel_scale-accoffsetz;
	
	tmp1=LPF2pApply_1((float)(*iax)*accel_scale-accoffsetx);
  tmp2=LPF2pApply_2((float)(*iay)*accel_scale-accoffsety);
  tmp3=LPF2pApply_3((float)(*iaz)*accel_scale-accoffsetz);
		
//	*ax=tmp1*accsensx;
//	*ay=tmp2*accsensy;
//	*az=tmp3*accsensz;
		
	*ax=tmp1;
	*ay=tmp2;
	*az=tmp3;
	}
	return res;
}
//�õ�������ֵ(ԭʼֵ)��ƽ���˲���������λ
//mx,my,mz:������x,y,z���ԭʼ����(������)
//Get the acceleration value(original value),low pass filter and adjust acceleration orientation
//mx,my,mz:Raw data of the acceleration X, Y, Z axis (with symbol)
//Return value: 0,set successfully
//          Other,error code
uint8_t MPU_Get_Magnetometer(short *mx,short *my,short *mz)
{
    uint8_t buf[6],res;  
 	res=MPU_Read_Len(AK8963_ADDR,MAG_XOUT_L,6,buf);
	if(res==0)
	{
		*mx=((uint16_t)buf[1]<<8)|buf[0];  
		*my=((uint16_t)buf[3]<<8)|buf[2];  
		*mz=((uint16_t)buf[5]<<8)|buf[4];
		*my=-*my;
		*mz=-*mz;
		*mx=(short)(magoldx*0.5+*mx*0.5);
		*my=(short)(magoldy*0.5+*my*0.5);
		*mz=(short)(magoldz*0.5+*mz*0.5);
		magoldx=*mx;
		magoldy=*my;
		magoldz=*mz;
	} 	 
	MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11); //AK8963ÿ�ζ����Ժ���Ҫ��������Ϊ���β���ģʽ
    return res;
}

/*
*@Function:����ô��������ݣ���λ��˹�����Դ����ƽ��в���
*
*
*/
uint8_t MPU_Get_Mag(short *imx,short *imy,short *imz,float *mx,float *my,float *mz)
{
	uint8_t res;
	res=MPU_Get_Magnetometer(imx,imy,imz);
	if (res==0)
	{
	tmp1=(float)(*imx)*mag_scale-magoffsetx;
	tmp2=(float)(*imy)*mag_scale-magoffsety;
	tmp3=(float)(*imz)*mag_scale-magoffsetz;
	*mx=B[0]*tmp1+B[1]*tmp2+B[2]*tmp3;
	*my=B[1]*tmp1+B[3]*tmp2+B[4]*tmp3;
	*mz=B[2]*tmp1+B[4]*tmp2+B[5]*tmp3;
	}
	return res;
}

//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    uint8_t i;
    IIC_Start();
     IIC_SendByte((addr<<1)|0); //����������ַ+д����
    if(IIC_WaitAck())          //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }
     IIC_SendByte(reg);         //д�Ĵ�����ַ
    IIC_WaitAck();             //�ȴ�Ӧ��
    for(i=0;i<len;i++)
    {
         IIC_SendByte(buf[i]);  //��������
        if(IIC_WaitAck())      //�ȴ�ACK
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_Stop();
    return 0;
} 

//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
    IIC_Start();
     IIC_SendByte((addr<<1)|0); //����������ַ+д����
    if(IIC_WaitAck())          //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }
     IIC_SendByte(reg);         //д�Ĵ�����ַ
    IIC_WaitAck();             //�ȴ�Ӧ��
	  IIC_Start();                
     IIC_SendByte((addr<<1)|1); //����������ַ+������
    IIC_WaitAck();             //�ȴ�Ӧ��
    while(len)
    {
        if(len==1)*buf=IIC_ReadByte(0);//������,����nACK 
		else *buf=IIC_ReadByte(1);		//������,����ACK  
		len--;
		buf++;  
    }
    IIC_Stop();                 //����һ��ֹͣ����
    return 0;       
}

//IICдһ���ֽ� 
//devaddr:����IIC��ַ
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Write_Byte(uint8_t addr,uint8_t reg,uint8_t data)
{
    IIC_Start();
    IIC_SendByte((addr<<1)|0); //����������ַ+д����
    if(IIC_WaitAck())          //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }
    IIC_SendByte(reg);         //д�Ĵ�����ַ
    IIC_WaitAck();             //�ȴ�Ӧ��
    IIC_SendByte(data);        //��������
    if(IIC_WaitAck())          //�ȴ�ACK
    {
        IIC_Stop();
        return 1;
    }
    IIC_Stop();
    return 0;
}

//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������
uint8_t MPU_Read_Byte(uint8_t addr,uint8_t reg)
{
    uint8_t res;
    IIC_Start();
    IIC_SendByte((addr<<1)|0); //����������ַ+д����
    IIC_WaitAck();             //�ȴ�Ӧ��
    IIC_SendByte(reg);         //д�Ĵ�����ַ
    IIC_WaitAck();             //�ȴ�Ӧ��
	IIC_Start();                
    IIC_SendByte((addr<<1)|1); //����������ַ+������
    IIC_WaitAck();             //�ȴ�Ӧ��
    res=IIC_ReadByte(0);		//������,����nACK  
    IIC_Stop();                 //����һ��ֹͣ����
    return res;  
}
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
