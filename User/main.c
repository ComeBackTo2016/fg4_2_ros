/* 
����������
���ĵ���װ�ã��������PWM-T1C1 �ڲ�����T4C1 ʹ��PB1 ϸ��-1-2-3(PB0-PC5-PC4) ����-PA7 CLK_F-PA3 CLK_B-PB3 ������:T2
��������װ�ã�ֱ�����PWM-T8C1C2 CLK-IR-PB5 �������-ads1115ch4
��ݮ�ɴ���: UART1()DMA1CH4(),5() 
485���ڣ�UART4()DMA2CH3(),5()[ˮ�ʴ�������IMU]
�����˿ڣ�����:ADC4-PA4 ��ѹ:ADS1115-CH3 ©ˮ���:PC0123 ������⴫����:IRL-1115CH1 IRR-1115CH2
���Դ���: UART5 PD2RX PC12TX
*/

#include "stm32f10x_it.h"
#include "init.h"
#include "MS5837.h"
#include "can.h"
#include "imu_data_decode.h"
#include "string.h"
#include "conmunication.h"
#include "HwInit.h"
#include "StepMotor.h"
#include "DCMotor.h"
#include "WaterSensor.h"
#include "SerialDMA.h"
#include "ms5837.h"
#include "Control.h"
#include "ads1115.h"
#define CLI()		__set_PRIMASK(1)
#define SEI()		__set_PRIMASK(0)

float temper_ms,depth_ms,pressure_ms;
int check_angle=0;
short int IR_offst_L = 0, IR_offst_R = 0;

ADS1115Typedef ir;
float A1 = 0, A2 = 0, A3 = 0, A4 = 0, u = 1;
int8_t dataa[4] = {0};

extern float LeakageData[4];
extern float BatIV[2];
extern float IRData[3];
int16_t booox[20] = {0}, yuyu;
unsigned char i;
extern uint8_t RaspiSend[RASPIBUFFERLEN];
extern int16_t MS5837_STATE;
extern FrameXbox360MTData frame360mtrec;

int16_t dytm = 0, pwww = 0;
extern float DistanceIR;


int main(void)
{	
		uint16_t syscnt=0;
	
		CLI();	
		SysTick_Init();
		UART_INIT();
		PWM_INIT();				
			
		ENCODE_INIT();
		  
		IIC_Init();
		CAN_Config();
		TIMER_INIT();
		LeakageSensorInit();
		Common_GPIO_Config();
		NVIC_of_All();
		SEI();
		
		Delay_ms(1000);
		StepMotorInit();
		PythonDataStructInit();
		MS5837DeviceInit();
		GetPoseJY901Init(30.0f);
		SwitchOfWaterQualitySensor(SWITCH_ALL, SW_ON);
		frame360mtrec.motor_l = 1500;
		frame360mtrec.motor_r = 1500;
		FramePushDataInit();

    while (1)
    { 
		Raspi_Data_Process(); // Run time: 8us
		switch (syscnt)
		{
			case 0:
				/* ©ˮ���������ݼ�� */
				LeakageData[0] = GetLeakageSensorStaus(S1);
				LeakageData[1] = GetLeakageSensorStaus(S2);
				LeakageData[2] = GetLeakageSensorStaus(S3);
				LeakageData[3] = GetLeakageSensorStaus(S4);
				/* �������ݲ��� */
				BatIV[1] = GetLeakageSensorStaus(BatI);
				/* ���ĵ��ڣ�Ĭ��λ��0.5 */
				BarycenterControl();
				syscnt = 1;
				break;
			case 1:
				/* ˮ�ø�������,Ĭ��λ��500 */
				BuoyancyControl();
			
				/* IMU����������ȡ */
				GetPoseJY901(); // Run time: 389.6us  T:1/10
				syscnt = 2;
				break;
			case 2:
				ADS1115GetData();
				DistanceIR = SharpSensorLengthGet(IRData[2]);
				/* ���Ϻ��⴫����R */
				/* �絼�ʴ��������ݶ�ȡ(������1/3hz) */
				EC_TEData_Send(); // Run time: 3us
				syscnt = 3;
				break;
			case 3:
//					/* ��ȴ��������ݻ�ȡ */
				if(MS5837_STATE == 1)
				{
					MS5837_read(); // Run time: 600us
				}
				syscnt = 4;
				break;
			case 4:
				Boradcast_Glider_Parameter();
				syscnt = 0;
				break;
		}
		}
}


/*********************************************END OF FILE**********************/
