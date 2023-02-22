/*
 * ps2.h
 *
 *  Created on: Feb 18, 2023
 *      Author:
 */

#ifndef USER_PS2_H_
#define USER_PS2_H_

#include "stdint.h"
#include "stm32f1xx_hal.h"

typedef struct _PES_BUTTON_
{
  uint16_t Select:1;
  uint16_t L:1;
  uint16_t R:1;
  uint16_t Start:1;
  uint16_t Up:1;
  uint16_t Down:1;
  uint16_t Right:1;
  uint16_t Left:1;
  uint16_t Tamgiac:1;
  uint16_t Tron:1;
  uint16_t Vuong:1;
  uint16_t X:1;
  uint16_t R1:1;
  uint16_t L1:1;
  uint16_t R2:1;
  uint16_t L2:1;
}pes_button_t;

extern pes_button_t pesButton;
void pes_receive_init(UART_HandleTypeDef* pUart);
void getPesRawData(uint8_t *pData);
void decodePES(void);
void decodePES_1(uint8_t *PES);
void getPesRawData(uint8_t *pData);
uint8_t getPesAnalog(uint8_t *pesRawData);
void pes_uart_event_handle(UART_HandleTypeDef *huart);

#endif /* USER_PS2_H_ */
