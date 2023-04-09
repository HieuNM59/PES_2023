/*
 * main-app.c
 *
 *  Created on: Feb 18, 2023
 *      Author:
 */
#include "stm32f1xx_hal.h"
#include "string.h"
#include "ps2.h"
#include "dwt_stm32_delay.h"
#include "ssd1306.h"
#include "fonts.h"
#include "fifo.h"
#include "main-app.h"

#define LED_PC13_Pin GPIO_PIN_13
#define LED_PC13_GPIO_Port GPIOC

#define MAX_BUFF					0xFF
#define SIZE_OF_PAYLOAD 			4
#define TIME_POLL_PES				60	//ms
#define TIME_POLL_MONITOR			100	//ms


#define USE_DEBUG_IN_ONE_BOARD

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;


static fifo_t pesFifo;

uint8_t pPesBuff[MAX_BUFF];

uint8_t u8_Recv;

static uint16_t digitalDebug = 0xFFFFu;
static uint8_t analogDebug = 0xFFu;

uint8_t g_debugFlag = 0;
pes_button_p debug_pesbutton = (pes_button_p)&digitalDebug;
pes_joyStick_p debug_pesJoyStick = (pes_joyStick_p)&analogDebug;

static void Debug_SendStr(char *pString);

static uint32_t getTimeElapse(uint32_t timeStart);

static void Debug_ButtonPrintfState(void);

static void pollFifo(void);

static void monitorInit();

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

static void monitorInit(){
	FifoInit(&pesFifo, pPesBuff, 1, MAX_BUFF);
	HAL_UART_Receive_IT(&huart3, &u8_Recv, 1);
}


// Frame = FEE+2col_2Line_1size_2total_string;
/*
 *
 *
 */
static void pollFifo(void){
	uint8_t size, colum, line, byTotal;
	static uint8_t data[30] = {0,};

	for(uint8_t i = 0; i < 3; i++){
		if(FifoPopData(&pesFifo, &data[i])){
		}
		else{
			return;
		}
	}
	if(data[0] == 'F' && data[1] == 'E' && data[2] == 'E')
	{
		for(uint8_t i = 0; i < 8; i++){
			if(FifoPopData(&pesFifo, &data[i])){
			}
			else{
				return;
			}
		}
		if(data[0] == '+'){
			colum = (data[1]-0x30)*10 + (data[2]-0x30);
			line = (data[3]-0x30)*10 + (data[4]-0x30);
			size = data[5] - 0x30;
			byTotal = (data[6]-0x30)*10 + (data[7]-0x30);

			memset(data, 0, sizeof(data));
			for(uint8_t i = 0; i < byTotal; i++){
				if(FifoPopData(&pesFifo, &data[i])){
				}
				else{
					return;
				}
			}
	        ssd1306_SetCursor(line, colum);
	        switch(size){
	        case 0:
	        	ssd1306_WriteString(&data[0], Font_7x10, White);
	        	break;
	        case 1:
	        	ssd1306_WriteString(&data[0], Font_11x18, White);
	        	break;
	        case 2:
	        	ssd1306_WriteString(&data[0], Font_16x26, White);
	        	break;
	        default:
	        	ssd1306_WriteString(&data[0], Font_7x10, White);
	        	break;
	        }
	        ssd1306_UpdateScreen();
		}

	}
	else if(data[0] == 'D' && data[1] == 'E' && data[2] == 'L'){
		ssd1306_Fill(Black);
		ssd1306_UpdateScreen();
	}
}
/*
 * Main Init
 */
void main_Init(void){
	DWT_Delay_Init();

	ssd1306_Init();

	monitorInit();
	/* Enable mode analog*/
	ps2_EnableAnalogMode();

	Debug_SendStr("Main Init! \n");
}

/*
 * Tần số SysCoreClock nên để 8Mhz như đã cấu hình, Cấu hình cao dễ gây nhiễu.
 * Đã Cấu hình I watchdog (500ms) để tránh việc treo MCU -> Không nên bỏ.
 */
void main_process(void){
	static uint32_t ticksPollPes = 0;
	static uint32_t ticksPollMonitor = 0;

	uint8_t PES[6] = {0, };
	uint8_t analogValue = 0xFF;
	uint8_t payload[SIZE_OF_PAYLOAD] = {0, };

	/*
	 * Do tốc độ truyền của RF(HC-05) là có giới hạn băng thông và tốc độ,
	 * Nên cần thay đổi thời gian TIME_POLL_PES(Nên để 20 -> 50ms),
	 * Tránh việc bắn Uart liên tục gây quá tải HC-05 -> HC-05 bị reboot -> mất kết nối PS2
	 */
	if(getTimeElapse(ticksPollPes) >= TIME_POLL_PES){
		getPesRawData(PES);

		analogValue = getPesAnalog(PES);

		payload[0] = 'F';			/* Header bytee */
		payload[1] = PES[0];		/* Low byte digital data */
		payload[2] = PES[1];		/* High byte digital data */
		payload[3] = analogValue;	/* Analog data */

		HAL_UART_Transmit(&huart3, payload, SIZE_OF_PAYLOAD, 200);

	#ifdef USE_DEBUG_IN_ONE_BOARD
		digitalDebug = (uint16_t)(payload[1] << 8 | payload[2]);
		analogDebug = analogValue;
		g_debugFlag = 1;
	#endif

		ticksPollPes = HAL_GetTick();
	}

	if(getTimeElapse(ticksPollMonitor) >= TIME_POLL_MONITOR){
		pollFifo();
		ticksPollMonitor = HAL_GetTick();
	}
	if(g_debugFlag){
		Debug_ButtonPrintfState();
		g_debugFlag = 0;
	}


}

/*
 * used for debug button in uart2
 */
static void Debug_ButtonPrintfState(void){

	static uint8_t lastState = 0;
	if(!debug_pesbutton->Up){
//		Debug_SendStr("Up is press!\n");
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
		if( lastState!=1 ){
			ssd1306_SetCursor(75, 53);
			ssd1306_WriteString("     UP", Font_7x10, White);
			ssd1306_UpdateScreen();
		}
		lastState = 1;
	}
	else if(!debug_pesbutton->Down){
//		Debug_SendStr("Down is press!\n");
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
		if( lastState != 2){
			ssd1306_SetCursor(75, 53);
			ssd1306_WriteString("   DOWN", Font_7x10, White);
			ssd1306_UpdateScreen();
		}

		lastState = 2;
	}
	else if(!debug_pesbutton->Right){
//		Debug_SendStr("Right is press!\n");
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
		if( lastState != 3){
			ssd1306_SetCursor(75, 53);
			ssd1306_WriteString("  RIGHT", Font_7x10, White);
			ssd1306_UpdateScreen();
		}

		lastState = 3;
	}
	else if(!debug_pesbutton->Left){
//		Debug_SendStr("Left is press!\n");
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
		if( lastState != 4){
			ssd1306_SetCursor(75, 53);
			ssd1306_WriteString("   LEFT", Font_7x10, White);
			ssd1306_UpdateScreen();
		}
		lastState = 4;
	}
	else if(!debug_pesbutton->L1){
//		Debug_SendStr("L1 is press!\n");
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
		if( lastState != 5){
			ssd1306_SetCursor(75, 53);
			ssd1306_WriteString("     L1", Font_7x10, White);
			ssd1306_UpdateScreen();
		}
		lastState = 5;
	}
	else if(!debug_pesbutton->L2){
//		Debug_SendStr("L2 is press!\n");
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
		if( lastState != 6){
			ssd1306_SetCursor(75, 53);
			ssd1306_WriteString("     L2", Font_7x10, White);
			ssd1306_UpdateScreen();
		}
		lastState = 6;
	}
	else if(!debug_pesbutton->R1){
//		Debug_SendStr("R1 is press!\n");
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
		if( lastState != 7){
			ssd1306_SetCursor(75, 53);
			ssd1306_WriteString("     R1", Font_7x10, White);
			ssd1306_UpdateScreen();
		}
		lastState = 7;
	}
	else if(!debug_pesbutton->R2){
//		Debug_SendStr("R2 is press!\n");
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
		if( lastState != 8){
			ssd1306_SetCursor(75, 53);
			ssd1306_WriteString("     R2", Font_7x10, White);
			ssd1306_UpdateScreen();
		}

		lastState = 8;
	}
	else if(!debug_pesbutton->Select){
//		Debug_SendStr("Select is press!\n");
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
		if( lastState != 9){
			ssd1306_SetCursor(75, 53);
			ssd1306_WriteString(" SELECT", Font_7x10, White);
			ssd1306_UpdateScreen();
		}

		lastState = 9;
	}
	else if(!debug_pesbutton->Start){
//		Debug_SendStr("Start is press!\n");
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
		if( lastState != 10){
			ssd1306_SetCursor(75, 53);
			ssd1306_WriteString("  START", Font_7x10, White);
			ssd1306_UpdateScreen();
		}
		lastState = 10;
	}
	else if(!debug_pesbutton->Tron){
//		Debug_SendStr("Tron is press!\n");
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
		if( lastState != 11){
			ssd1306_SetCursor(75, 53);
			ssd1306_WriteString("   TRON", Font_7x10, White);
			ssd1306_UpdateScreen();
		}

		lastState = 11;
	}
	else if(!debug_pesbutton->Vuong){
//		Debug_SendStr("Vuong is press!\n");
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
		if( lastState != 12){
			ssd1306_SetCursor(75, 53);
			ssd1306_WriteString("  VUONG", Font_7x10, White);
			ssd1306_UpdateScreen();
		}

		lastState = 12;
	}
	else if(!debug_pesbutton->Tamgiac){
//		Debug_SendStr("Tamgiac is press!\n");
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
		if( lastState != 13){
			ssd1306_SetCursor(75, 53);
			ssd1306_WriteString("TAMGIAC", Font_7x10, White);
			ssd1306_UpdateScreen();
		}
		lastState = 13;
	}
	else if(!debug_pesbutton->X){
//		Debug_SendStr("X is press!\n");
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
		if( lastState != 14){
			ssd1306_SetCursor(75, 53);
			ssd1306_WriteString("      X", Font_7x10, White);
			ssd1306_UpdateScreen();
		}
		lastState = 14;
	}
	else{
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 0);
	}
}


/*
 * Callback trên khối truyền
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART3)
	{
		FifoPushData(&pesFifo, &u8_Recv);
		HAL_UART_Receive_IT(&huart3, &u8_Recv, 1);
	}
}
