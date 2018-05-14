#ifndef __LED_H
#define	__LED_H

#include "stm32f10x.h"

/** the macro definition to trigger the led on or off 
  * 1 - off
  *0 - on
  */
#define ON  0
#define OFF 1

/* ���κ꣬��������������һ��ʹ�� */
#define LED1(a)	if (a)	\
					GPIO_SetBits(GPIOE,GPIO_Pin_4);\
					else		\
					GPIO_ResetBits(GPIOE,GPIO_Pin_4)

#define LED2(a)	if (a)	\
					GPIO_SetBits(GPIOE,GPIO_Pin_5);\
					else		\
					GPIO_ResetBits(GPIOE,GPIO_Pin_5)

#define LED3(a)	if (a)	\
					GPIO_SetBits(GPIOE,GPIO_Pin_6);\
					else		\
					GPIO_ResetBits(GPIOE,GPIO_Pin_6)

#define LED4(a)	if (a)	\
					GPIO_SetBits(GPIOE,GPIO_Pin_7);\
					else		\
					GPIO_ResetBits(GPIOE,GPIO_Pin_7)

/* ֱ�Ӳ����Ĵ����ķ�������IO */
#define	digitalHi(p,i)			{p->BSRR=i;}			//����Ϊ�ߵ�ƽ		
#define digitalLo(p,i)			{p->BRR=i;}				//����͵�ƽ
#define digitalToggle(p,i)		{p->ODR ^=i;}			//�����ת״̬


/* �������IO�ĺ� */
#define LED1_TOGGLE		digitalToggle(GPIOE,GPIO_Pin_4)
#define LED1_OFF		digitalHi(GPIOE,GPIO_Pin_4)
#define LED1_ON			digitalLo(GPIOE,GPIO_Pin_4)

#define LED2_TOGGLE		digitalToggle(GPIOE,GPIO_Pin_5)
#define LED2_OFF		digitalHi(GPIOE,GPIO_Pin_5)
#define LED2_ON			digitalLo(GPIOE,GPIO_Pin_5)

#define LED3_TOGGLE		digitalToggle(GPIOE,GPIO_Pin_6)
#define LED3_OFF		digitalHi(GPIOE,GPIO_Pin_6)
#define LED3_ON			digitalLo(GPIOE,GPIO_Pin_6)

#define LED4_TOGGLE		digitalToggle(GPIOE,GPIO_Pin_7)
#define LED4_OFF		digitalHi(GPIOE,GPIO_Pin_7)
#define LED4_ON			digitalLo(GPIOE,GPIO_Pin_7)
void LED_GPIO_Config(void);

#endif /* __LED_H */
