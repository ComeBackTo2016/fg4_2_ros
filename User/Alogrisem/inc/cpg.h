#ifndef _cpg_H__
#define _cpg_H__


#define PI 3.14159
#define PWM_Period_ms 0.1
#define PWM_Duty_Middle_us 50
#define PWM_Duty_LeftLimit_us 0
#define PWM_Duty_RightLimit_us 100
#define Trans_factor             3               //60000/20000
#define MiddlePosition_value     50            //PWM_Duty_Middle_us * Trans_factor;  //60000?PWM????,?200000us.
#define LeftLimit_value          0            //PWM_Duty_LeftLimit_us  * Trans_factor;
#define RightLimit_value         100            //PWM_Duty_RightLimit_us * Trans_factor;
#define min(x,y) ((x) < (y) ? x : y)
/* The Parameter will be update in IIC-interupt fun of Control.c */
//float Motor_Speed; 
//float Motor_Log_degree_12;
//float Motor_Log_degree_13;
//float Motor_Amplitude_degree[3];    
//float Motor_Dynamic_Offset_degree[3];

//float Pectoral_Volcety ;                 // 胸鳍速度
//float Pectoral_Phase_Dif;                // 胸鳍相角差
//float Pectoral_Amplitude[2];             // 幅值参数
//float Pectoral_Dynamic_Offset[2];        // 动角

void cpg_run();//尾鳍CPG算法跟新
void CPG_Configuration();
void CPG_caculate();
void Pectoral_CPG_Calculate();

#endif