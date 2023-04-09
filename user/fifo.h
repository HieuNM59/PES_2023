/*
 * fifo.h
 *
 *  Created on: Dec 9, 2022
 *      Author: HieuNM
 */

#ifndef USER_UTILITIES_FIFO_H_
#define USER_UTILITIES_FIFO_H_

#include <stdint.h>

typedef struct _Fifo_{
    uint16_t wHeadIndex;
    uint16_t wTailIndex;
    uint16_t wCountElement;
    uint16_t wSize;
    uint8_t* pData;
    uint8_t byItemSize;   //byte
}fifo_t, *fifo_p;

void FifoInit(fifo_p pQueue, void *pBuffer, uint8_t sizeOfElements, uint16_t numOfElements);
uint8_t FifoIsFull(fifo_p pQueue);
uint8_t FifoIsEmpty(fifo_p pQueue);
uint8_t FifoPushData(fifo_p pQueue, uint8_t *pData);
uint8_t FifoPopData(fifo_p pQueue, uint8_t *pData);
void FifoFlush(fifo_p pQueue);

#endif /* USER_UTILITIES_FIFO_H_ */
