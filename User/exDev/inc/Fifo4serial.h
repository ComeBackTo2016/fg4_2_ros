#ifndef _FIFO4SERIAL_H_
#define _FIFO4SERIAL_H_

#include "stm32f10x.h"

#define QUEUE_EMPTY 0
#define QUEUE_FULL 1
#define QUEUE_OK 2
#define QUEUE_BUFFER 512

typedef struct {
	int8_t base[QUEUE_BUFFER] ;
	uint16_t bufferCount ;
	uint16_t front ;
	uint16_t rear ;
} Fifo4Serial ;
extern void QueueInit(Fifo4Serial *Q) ;
extern char QueueIn(Fifo4Serial *Q, int8_t dat) ;
extern char QueueOut(Fifo4Serial *Q, int8_t *dat) ;

#endif
