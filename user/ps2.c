/*
 * ps2.c
 *
 *  Created on: Feb 18, 2023
 *      Author:
 */

#include "dwt_stm32_delay.h"
#include "ps2.h"

#define PES_CLK_Pin GPIO_PIN_12
#define PES_CLK_GPIO_Port GPIOB
#define PES_ATT_Pin GPIO_PIN_13
#define PES_ATT_GPIO_Port GPIOB
#define PES_CMD_Pin GPIO_PIN_14
#define PES_CMD_GPIO_Port GPIOB
#define PES_DATA_Pin GPIO_PIN_15
#define PES_DATA_GPIO_Port GPIOB

/* Structure ---------------------------------------------------------*/

extern UART_HandleTypeDef huart2;
/* Public variables -------------------------------------------------*/
unsigned char PS2_CONFIGMODE[5] = {0x01, 0x43, 0x00, 0x01, 0x00};
unsigned char PS2_ANALOGMODE[9] = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
pes_button_t pesButton;


unsigned char access(unsigned int tbyte){
	unsigned char rbyte = 0, tempp = 0;
	unsigned int j;
	HAL_GPIO_WritePin(PES_CLK_GPIO_Port, PES_CLK_Pin, GPIO_PIN_SET);
	for(j=0; j<8; j++)
	{
		HAL_GPIO_WritePin(PES_CMD_GPIO_Port, PES_CMD_Pin, tbyte&0x01);
		DWT_Delay_us(10);
		HAL_GPIO_WritePin(PES_CLK_GPIO_Port, PES_CLK_Pin, GPIO_PIN_RESET);
		DWT_Delay_us(10);
		tempp=HAL_GPIO_ReadPin(PES_DATA_GPIO_Port, PES_DATA_Pin);
		rbyte = (rbyte >> 1) | (tempp << 7);
		HAL_GPIO_WritePin(PES_CLK_GPIO_Port, PES_CLK_Pin, GPIO_PIN_SET);
		tbyte = tbyte >> 1;
	}
	DWT_Delay_us(20);
	return rbyte;
}

void getPesRawData(uint8_t *pData){
	uint8_t analog = 128;
	HAL_GPIO_WritePin(PES_ATT_GPIO_Port,PES_ATT_Pin,GPIO_PIN_RESET);
	access(0x01);
	analog = access(0x42);
	access(0);
	pData[0] = access(0);
	pData[1] = access(0);
	if(analog == 0x73 || analog == 0x23 || analog == 0x53)
	{
		pData[2] = access(0);
		pData[3] = access(0);
		pData[4] = access(0);
		pData[5] = access(0);
	}
	if(analog == 0x12)
	{
		pData[2] = access(0);
		pData[3] = access(0);
	}
	HAL_GPIO_WritePin(PES_ATT_GPIO_Port,PES_ATT_Pin,GPIO_PIN_SET);
}

void send_ps2 (unsigned char *cmd, unsigned char length){
	int i;
	HAL_GPIO_WritePin(PES_ATT_GPIO_Port,PES_ATT_Pin,GPIO_PIN_RESET);
	DWT_Delay_us(10);
	for(i=0;i<length;i++)
	access(cmd[i]);
	DWT_Delay_us(10);
	HAL_GPIO_WritePin(PES_ATT_GPIO_Port,PES_ATT_Pin,GPIO_PIN_SET);
}

void ps_2_int(void){
	send_ps2(PS2_CONFIGMODE, 5);
	DWT_Delay_us(10);
	send_ps2(PS2_ANALOGMODE, 9);
	DWT_Delay_us(10);
}

uint8_t getPesAnalog(uint8_t *pesRawData){
	uint8_t analog;

    analog = 0xff;
    if(pesRawData[5] > 230)
		analog = analog & 0xfe;
	else if(pesRawData[5] < 20)
		analog = analog & 0xfd;
	else
		analog = analog | 0x03;

	if(pesRawData[4] > 230)
		analog = analog & 0xfb;
	else if(pesRawData[4] < 20)
		analog = analog & 0xf7;
	else
        analog = analog | 0x0c;

	if(pesRawData[3] > 230)
		analog = analog & 0xef;
	else if(pesRawData[3] < 20)
		analog = analog & 0xdf;
	else
		analog = analog | 0x30;

	if(pesRawData[2] > 230)
		analog = analog & 0xbf;
	else if(pesRawData[2] < 20)
		analog=analog & 0x7f;
	else
		analog=analog | 0xc0;
	return analog;
}


void decodePES_1(uint8_t *PES){
	pesButton.Select 	= (PES[0] & 0x01);
	pesButton.L 		= (PES[0] & 0x02) >> 1;
	pesButton.R 		= (PES[0] & 0x04) >> 2;
	pesButton.Start 	= (PES[0] & 0x08) >> 3;

	pesButton.Up 		= (PES[0] & 0x10) >> 4;
	pesButton.Right 	= (PES[0] & 0x20) >> 5;
	pesButton.Down 		= (PES[0] & 0x40) >> 6;
	pesButton.Left 		= (PES[0] & 0x80) >> 7;

	pesButton.L2 		= (PES[1] & 0x01);
	pesButton.R2 		= (PES[1] & 0x02) >> 1;
	pesButton.L1 		= (PES[1] & 0x04) >> 2;
	pesButton.R1 		= (PES[1] & 0x08) >> 3;

	pesButton.Tamgiac 	= (PES[1] & 0x10) >> 4;
	pesButton.Tron 		= (PES[1] & 0x20) >> 5;
	pesButton.X 		= (PES[1] & 0x40) >> 6;
	pesButton.Vuong 	= (PES[1] & 0x80) >> 7;
}
/* Receive Pes Data --------------------------------------------------*/


UART_HandleTypeDef* UartReceive;
uint8_t u8_pesData;
uint16_t pesDigitalRawData = 0xFFFF;

void pes_receive_init(UART_HandleTypeDef* pUart){
	UartReceive = pUart;
	HAL_UART_Receive_IT(UartReceive, &u8_pesData, 1);
}

void decodePES(void){
	uint8_t pesData[2];

	pesData[0] = pesDigitalRawData & 0xFF;
	pesData[1] = (pesDigitalRawData >> 8) & 0xFF;

	pesButton.Select 	= (pesData[0] & 0x01);
	pesButton.L 		= (pesData[0] & 0x02) >> 1;
	pesButton.R 		= (pesData[0] & 0x04) >> 2;
	pesButton.Start 	= (pesData[0] & 0x08) >> 3;

	pesButton.Up 		= (pesData[0] & 0x10) >> 4;
	pesButton.Right 	= (pesData[0] & 0x20) >> 5;
	pesButton.Down 		= (pesData[0] & 0x40) >> 6;
	pesButton.Left 		= (pesData[0] & 0x80) >> 7;

	pesButton.L2 		= (pesData[1] & 0x01);
	pesButton.R2 		= (pesData[1] & 0x02) >> 1;
	pesButton.L1 		= (pesData[1] & 0x04) >> 2;
	pesButton.R1 		= (pesData[1] & 0x08) >> 3;

	pesButton.Tamgiac 	= (pesData[1] & 0x10) >> 4;
	pesButton.Tron 		= (pesData[1] & 0x20) >> 5;
	pesButton.X 		= (pesData[1] & 0x40) >> 6;
	pesButton.Vuong 	= (pesData[1] & 0x80) >> 7;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	static uint8_t byHeadIsTrue = 0;
	static uint8_t i_pes = 0;
	static uint8_t pesBuff[3];

	if(huart->Instance == UartReceive->Instance){
		if(u8_pesData == 'F' ){
			byHeadIsTrue = 1;
			i_pes = 0;
		}
		else{
			if(byHeadIsTrue){
				pesBuff[i_pes++] = u8_pesData;
				if(i_pes > 2){
					pesDigitalRawData = pesBuff[1] << 8 | pesBuff[0];
					i_pes = 0;
					byHeadIsTrue = 0;
				}
			}
			else{
				pesDigitalRawData = 0xFFFF;
			}
		}
		HAL_UART_Receive_IT(UartReceive, &u8_pesData, 1);
	}
}
