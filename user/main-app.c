/*
 * main-app.c
 *
 *  Created on: Feb 18, 2023
 *      Author:
 */
#include "stm32f1xx_hal.h"
#include "ps2.h"
#include "dwt_stm32_delay.h"
#include "main-app.h"

#define SIZE_OF_PAYLOAD 			4
#define TIME_POLL_PES				20	//ms

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

static void Debug_SendStr(char *pString);
static uint32_t getTimeElapse(uint32_t timeStart);

static uint32_t getTimeElapse(uint32_t timeStart){
	uint32_t curTime = HAL_GetTick();
	if(curTime > timeStart){
		return (HAL_GetTick() - timeStart);
	}
	return ((0xFFFFFF00 - timeStart) + curTime);

}

static void Debug_SendStr(char *pString) {
    while (*pString != NULL) {
    	HAL_UART_Transmit(&huart2,(uint8_t*) pString, 1, 100);
        pString++;
    }
}
void main_Init(void){
	DWT_Delay_Init();
	pes_receive_init(&huart3);
	Debug_SendStr("Main Init! \n");
}

void Debug_ButtonPrintfState(void){
	if(!pesButton.Up){
		Debug_SendStr("Button Up is press!\n");
	}
	else if(!pesButton.Down){
		Debug_SendStr("Button Down is press!\n");
	}
	else if(!pesButton.Right){
		Debug_SendStr("Button Right is press!\n");
	}
	else if(!pesButton.Left){
		Debug_SendStr("Button Left is press!\n");
	}
	else if(!pesButton.L1){
		Debug_SendStr("Button L1 is press!\n");
	}
	else if(!pesButton.L2){
		Debug_SendStr("Button L2 is press!\n");
	}
	else if(!pesButton.R1){
		Debug_SendStr("Button R1 is press!\n");
	}
	else if(!pesButton.R2){
		Debug_SendStr("Button R2 is press!\n");
	}
	else if(!pesButton.Select){
		Debug_SendStr("Button Select is press!\n");
	}
	else if(!pesButton.Start){
		Debug_SendStr("Button Start is press!\n");
	}
	else if(!pesButton.Tron){
		Debug_SendStr("Button Tron is press!\n");
	}
	else if(!pesButton.Vuong){
		Debug_SendStr("Button Vuong is press!\n");
	}
	else if(!pesButton.Tamgiac){
		Debug_SendStr("Button Tamgiac is press!\n");
	}
	else if(!pesButton.X){
		Debug_SendStr("Button X is press!\n");
	}
}

void main_process(void){
	static uint32_t ticks = 0;
	uint8_t PES[6] = {0, };
	uint8_t analogValue = 0xFF;
	uint8_t payload[SIZE_OF_PAYLOAD] = {0, };

	if(getTimeElapse(ticks) >= TIME_POLL_PES){
		getPesRawData(PES);

		analogValue = getPesAnalog(PES);

		payload[0] = 'F';			/* Header bytee */
		payload[1] = PES[0];		/* Low byte digital data */
		payload[2] = PES[1];		/* High byte digital data */
		payload[3] = analogValue;	/* Analog data */

		HAL_UART_Transmit(&huart3, payload, SIZE_OF_PAYLOAD, 200);
		ticks = HAL_GetTick();
	}

	decodePES();
	Debug_ButtonPrintfState();
}
