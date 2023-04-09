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

typedef struct _PES_BUTTON_ // Thay đổi thứ tự sẽ gây sai nút
{
	uint16_t L2:1;
	uint16_t R2:1;
	uint16_t L1:1;
	uint16_t R1:1;

	uint16_t Tamgiac:1;
	uint16_t Tron:1;
	uint16_t X:1;
	uint16_t Vuong:1;

	uint16_t Select:1;
	uint16_t L:1;
	uint16_t R:1;
	uint16_t Start:1;

	uint16_t Up:1;
	uint16_t Right:1;
	uint16_t Down:1;
	uint16_t Left:1;
}pes_button_t, *pes_button_p;

typedef struct _PES_JOY_STICk_	// Thay đổi thứ tự sẽ gây sai nút
{
	uint8_t left_Down:1;
	uint8_t left_Up:1;
	uint8_t left_Right:1;
	uint8_t left_Left:1;

	uint8_t right_Down:1;
	uint8_t right_Up:1;
	uint8_t right_Right:1;
	uint8_t right_Left:1;
}pes_joyStick_t, *pes_joyStick_p;

extern pes_button_p pesButton;
extern pes_joyStick_p pesJoyStick;

extern pes_button_p debug_pesbutton;
extern pes_joyStick_p debug_pesJoyStick;

void pes_receive_init(UART_HandleTypeDef* pUart);
void getPesRawData(uint8_t *pData);
void decodePES(void);
void decodePES_1(uint8_t *PES);
void getPesRawData(uint8_t *pData);
uint8_t getPesAnalog(uint8_t *pesRawData);
void pes_uart_event_handle(UART_HandleTypeDef *huart);
void ps2_EnableAnalogMode(void);

#endif /* USER_PS2_H_ */
