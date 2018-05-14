#include "CRC.h"

////////////////////////////////////////////////////////////////////////////////
// CRC校验，在上一轮校验的基础上继续作8位CRC校验
// 
// 输入参数：
//   chCurrByte 低8位数据有效，记录了上一次CRC校验的余数
//   chNextByte 低8位数据有效，记录了本次要继续校验的一个字节 
//
// 传出参数：
//   chCurrByte 低8位数据有效，记录了本次CRC校验的余数
////////////////////////////////////////////////////////////////////////////////

int checkCRC(int chCurrByte, int chNextByte)
{
	int nMask;
// CRC循环：每次调用进行8次循环，处理一个字节的数据。
	for(nMask = 0x80; nMask > 0; nMask = nMask>>1)
	{
   		if ((chCurrByte & 0x80) != 0)   // 首位为1：移位，并进行异或运算  
  		{ 
    		chCurrByte <<= 1;     // 移一位
    		if ( (chNextByte & nMask) != 0) // 补一位
    		{
     			chCurrByte |= 1;
    		}	
    		chCurrByte ^= 7;     // 首位已经移出，仅对低8位进行异或运算，7的二进制为0000,0111
   		}
   		else         // 首位为0，只移位，不进行异或运算
   		{  
    		chCurrByte <<= 1;     // 移一位
    		if ( (chNextByte & nMask) != 0) // 补一位
    		{
     			chCurrByte |= 1;
    		}
   		}
	}
	return chCurrByte;
}
