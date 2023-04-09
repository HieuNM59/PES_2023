/*
 * fifo.c
 *
 *  Created on: Dec 9, 2022
 *      Author: HieuNM
 */
#include "stddef.h"
#include "fifo.h"

#define FALSE 	0
#define TRUE 	1

void FifoInit(fifo_p pQueue, void *pBuffer, uint8_t sizeOfElements, uint16_t numOfElements){
    pQueue->wHeadIndex = 0;
    pQueue->wTailIndex = 0;
    pQueue->byItemSize = sizeOfElements;
    pQueue->wSize = numOfElements;
    pQueue->pData = pBuffer;
}

uint8_t FifoIsFull(fifo_p pQueue){
    if(pQueue != NULL){
        return (pQueue->wCountElement >= pQueue->wSize);
    }
    return FALSE;
}

uint8_t FifoIsEmpty(fifo_p pQueue){
    return (pQueue->wHeadIndex == pQueue->wTailIndex);
}

uint8_t FifoPushData(fifo_p pQueue, uint8_t* pData){
    for(uint8_t i_push = 0; i_push < pQueue->byItemSize; i_push++){
        pQueue->pData[pQueue->wHeadIndex] = pData[i_push];
        pQueue->wHeadIndex = (uint16_t)(pQueue->wHeadIndex + 1);
        pQueue->wCountElement ++;
    }

    if(FifoIsFull(pQueue)){
        pQueue->wTailIndex = (uint16_t)(pQueue->wTailIndex + pQueue->byItemSize);
    }
    return TRUE;
}

uint8_t FifoPopData(fifo_p pQueue, uint8_t* pData){
    if(FifoIsEmpty(pQueue)){
        pQueue->wCountElement = 0;
        return FALSE;
    }
    for(uint8_t i_pop = 0; i_pop < pQueue->byItemSize; i_pop++){
        pData[i_pop] = pQueue->pData[pQueue->wTailIndex];
        pQueue->wTailIndex = (uint16_t)(pQueue->wTailIndex + 1);
        pQueue->wCountElement --;
    }
    return TRUE;
}

void FifoFlush(fifo_p pQueue){
    pQueue->wHeadIndex = 0;
    pQueue->wTailIndex = 0;
    pQueue->wCountElement = 0;

    memset(pQueue->pData, 0, pQueue->wSize);
}
