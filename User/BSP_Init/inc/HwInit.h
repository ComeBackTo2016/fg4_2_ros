#ifndef __HWINIT_H
#define	__HWINIT_H

#include "stm32f10x.h"

/* ����˿ڶ��塪��������� */
#define LV_DIR_1 PAout(7)
#define LV_MS3_1 PCout(4)
#define LV_MS2_1 PCout(5)
#define LV_MS1_1 PBout(0)
#define LV_EN_1  PBout(1)
#define MO_EN_1  PBout(15)
/* ����˿ڶ��塪����������Դ��� */
#define SW_EC PBout(14)
#define SW_DO PBout(13)
#define SW_PH PBout(12)
/* ����˿ڶ��塪���������ʹ�ܶ� */
#define EN_Motor PBout(15)
/* ����˿ڶ��塪����ײ���� */
//#define CLK_IR		PBin(5)
//#define CLK_F		PAin(3)
//#define CLK_B		PBin(4) 
#define CLK_IR		PBin(5)
#define CLK_F		PBin(9)
#define CLK_B		PBin(8) 
//typedef enum {DISABLE = 0, ENABLE = !DISABLE} DirctState;

#endif 