#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H

#include "stm32f10x.h"

#define SPI_FLASH_SPI                           SPI1
#define SPI_FLASH_SPI_CLK                       RCC_APB2Periph_SPI1
#define SPI_FLASH_SPI_SCK_PIN                   GPIO_Pin_5                  /* PA.05 */
#define SPI_FLASH_SPI_SCK_GPIO_PORT             GPIOA                       /* GPIOA */
#define SPI_FLASH_SPI_SCK_GPIO_CLK              RCC_APB2Periph_GPIOA
#define SPI_FLASH_SPI_MISO_PIN                  GPIO_Pin_6                  /* PA.06 */
#define SPI_FLASH_SPI_MISO_GPIO_PORT            GPIOA                       /* GPIOA */
#define SPI_FLASH_SPI_MISO_GPIO_CLK             RCC_APB2Periph_GPIOA
#define SPI_FLASH_SPI_MOSI_PIN                  GPIO_Pin_7                  /* PA.07 */
#define SPI_FLASH_SPI_MOSI_GPIO_PORT            GPIOA                       /* GPIOA */
#define SPI_FLASH_SPI_MOSI_GPIO_CLK             RCC_APB2Periph_GPIOA
#define SPI_FLASH_CS_PIN                        GPIO_Pin_4                  /* PA.04 */
#define SPI_FLASH_CS_GPIO_PORT                  GPIOA                       /* GPIOA */
#define SPI_FLASH_CS_GPIO_CLK                   RCC_APB2Periph_GPIOA


//#define SPI_FLASH_CS_LOW()       GPIOA->BRR = GPIO_Pin_2		// GPIO_ResetBits(GPIOA, GPIO_Pin_2) 
//#define SPI_FLASH_CS_HIGH()      GPIOA->BSRR = GPIO_Pin_2		// GPIO_SetBits(GPIOA, GPIO_Pin_2)

#define SPI_FLASH_CS_LOW()       GPIOB->BRR = GPIO_Pin_5		// GPIO_ResetBits(GPIOA, GPIO_Pin_2) 
#define SPI_FLASH_CS_HIGH()      GPIOB->BSRR = GPIO_Pin_5		// GPIO_SetBits(GPIOA, GPIO_Pin_2)

#define SPI_DEPTH_CS_LOW()       GPIOB->BRR = GPIO_Pin_4		// GPIO_ResetBits(GPIOA, GPIO_Pin_2) 
#define SPI_DEPTH_CS_HIGH()      GPIOB->BSRR = GPIO_Pin_4		// GPIO_SetBits(GPIOA, GPIO_Pin_2)

void SPI_FLASH_Init(void);
void SPI_FLASH_SectorErase(u32 SectorAddr);
void SPI_FLASH_BulkErase(void);
void SPI_FLASH_PageWrite(u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite);
void SPI_FLASH_BufferWrite(u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite);
void SPI_FLASH_BufferRead(u8* pBuffer, u32 ReadAddr, u16 NumByteToRead);
u32 SPI_FLASH_ReadID(void);
u32 SPI_FLASH_ReadDeviceID(void);
void SPI_FLASH_StartReadSequence(u32 ReadAddr);
void SPI_Flash_PowerDown(void);
void SPI_Flash_WAKEUP(void);


u8 SPI_FLASH_ReadByte(void);
u8 SPI_FLASH_SendByte(u8 byte);
u16 SPI_FLASH_SendHalfWord(u16 HalfWord);
void SPI_FLASH_WriteEnable(void);
void SPI_FLASH_WaitForWriteEnd(void);

#endif /* __SPI_FLASH_H */

