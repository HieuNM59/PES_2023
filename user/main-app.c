/*
 * main-app.c
 *
 *  Created on: Feb 18, 2023
 *      Author:
 */
#include "stm32f1xx_hal.h"
#include "ps2.h"
#include "dwt_stm32_delay.h"
#include "ssd1306.h"
#include "fonts.h"
#include "main-app.h"

#define LED_PC13_Pin GPIO_PIN_13
#define LED_PC13_GPIO_Port GPIOC

#define SIZE_OF_PAYLOAD 			4
#define TIME_POLL_PES				20	//ms

#define USE_DEBUG_IN_ONE_BOARD

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

static void Debug_SendStr(char *pString);

static uint32_t getTimeElapse(uint32_t timeStart);

void Debug_ButtonPrintfState(void);

static uint32_t getTimeElapse(uint32_t timeStart){
	uint32_t curTime = HAL_GetTick();
	if(curTime > timeStart){
		return (HAL_GetTick() - timeStart);
	}
	return ((0xFFFFFF00 - timeStart) + curTime);

}

/*
 * use for debug in uart2( tx is PA2)
 */
static void Debug_SendStr(char *pString) {
    while (*pString != NULL) {
    	HAL_UART_Transmit(&huart2,(uint8_t*) pString, 1, 100);
        pString++;
    }
}
/*
 * Main Init
 */
void main_Init(void){
	DWT_Delay_Init();
	ssd1306_Init();
	ps2_EnableAnalogMode();

//	pes_receive_init(&huart3);
	pesDebugInit();
	Debug_SendStr("Main Init! \n");
}

/*
 * Tần số SysCoreClock nên để 8Mhz như đã cấu hình, Cấu hình cao dễ gây nhiễu.
 * Đã Cấu hình I watchdog (500ms) để tránh việc treo MCU -> Không nên bỏ.
 */
void main_process(void){
	static uint32_t ticks = 0;
	uint8_t PES[6] = {0, };
	uint8_t analogValue = 0xFF;
	uint8_t payload[SIZE_OF_PAYLOAD] = {0, };

	/*
	 * Do tốc độ truyền của RF(HC-05) là có giới hạn băng thông và tốc độ,
	 * Nên cần thay đổi thời gian TIME_POLL_PES(Nên để 20 -> 50ms),
	 * Tránh việc bắn Uart liên tục gây quá tải HC-05 -> HC-05 bị reboot -> mất kết nối PS2
	 */
	if(getTimeElapse(ticks) >= TIME_POLL_PES){
		getPesRawData(PES);

		analogValue = getPesAnalog(PES);

		payload[0] = 'F';			/* Header bytee */
		payload[1] = PES[0];		/* Low byte digital data */
		payload[2] = PES[1];		/* High byte digital data */
		payload[3] = analogValue;	/* Analog data */

		HAL_UART_Transmit(&huart3, payload, SIZE_OF_PAYLOAD, 200);

		/* Use to debug in only one board.
		 * connect Uart 3 TX -> Uart3 RX (PB10 -> PB11)
		 * Debug printing in Uart 2 TX (PA2)
		 */
	#ifdef USE_DEBUG_IN_ONE_BOARD
		Debug_ButtonPrintfState();
	#endif

		ticks = HAL_GetTick();
	}
}

/*
 * used for debug button in uart2
 */
void Debug_ButtonPrintfState(void){
	if(!debug_pesbutton->Up){
		Debug_SendStr("Up is press!\n");
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
	}
	else if(!debug_pesbutton->Down){
		Debug_SendStr("Down is press!\n");
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
	}
	else if(!debug_pesbutton->Right){
		Debug_SendStr("Right is press!\n");
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
	}
	else if(!debug_pesbutton->Left){
		Debug_SendStr("Left is press!\n");
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
	}
	else if(!debug_pesbutton->L1){
		Debug_SendStr("L1 is press!\n");
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
	}
	else if(!debug_pesbutton->L2){
		Debug_SendStr("L2 is press!\n");
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
	}
	else if(!debug_pesbutton->R1){
		Debug_SendStr("R1 is press!\n");
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
	}
	else if(!debug_pesbutton->R2){
		Debug_SendStr("R2 is press!\n");
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
	}
	else if(!debug_pesbutton->Select){
		Debug_SendStr("Select is press!\n");
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
	}
	else if(!debug_pesbutton->Start){
		Debug_SendStr("Start is press!\n");
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
	}
	else if(!debug_pesbutton->Tron){
		Debug_SendStr("Tron is press!\n");
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
	}
	else if(!debug_pesbutton->Vuong){
		Debug_SendStr("Vuong is press!\n");
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
	}
	else if(!debug_pesbutton->Tamgiac){
		Debug_SendStr("Tamgiac is press!\n");
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
	}
	else if(!debug_pesbutton->X){
		Debug_SendStr("X is press!\n");
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
	}
	else{
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 0);
	}
}
