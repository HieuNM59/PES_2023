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

/* Public variables -------------------------------------------------*/

uint8_t PS2_POLL_ARR[]	= {0x1, 0x42, 0x00, 0x00, 0x0};
uint8_t PS2_CONFIG_MODE[5] = {0x01, 0x43, 0x00, 0x01, 0x00};
uint8_t PS2_ANALOG_MODE[9] = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
uint8_t PS2_EXIT_CONFIG[9] = {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};


pes_button_t pesButton;
pes_joyStick_t pesJoyStick;

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

void ps2_SendConfig(uint8_t* cmd, uint8_t length)
{
	int i;
	HAL_GPIO_WritePin(PES_ATT_GPIO_Port,PES_ATT_Pin,GPIO_PIN_RESET);
	DWT_Delay_us(16);
	for(i = 0 ; i < length ; i++){
		access(cmd[i]);
	}
	DWT_Delay_us(16);
	HAL_GPIO_WritePin(PES_ATT_GPIO_Port,PES_ATT_Pin,GPIO_PIN_SET);
}

void ps2_EnableAnalogMode(void){
	ps2_SendConfig(PS2_POLL_ARR, sizeof(PS2_POLL_ARR));
	ps2_SendConfig(PS2_POLL_ARR, sizeof(PS2_POLL_ARR));
	ps2_SendConfig(PS2_POLL_ARR, sizeof(PS2_POLL_ARR));

	ps2_SendConfig(PS2_CONFIG_MODE, sizeof(PS2_CONFIG_MODE));
	ps2_SendConfig(PS2_ANALOG_MODE, sizeof(PS2_ANALOG_MODE));
	ps2_SendConfig(PS2_EXIT_CONFIG, sizeof(PS2_EXIT_CONFIG));
}
/**
 * Get digital datain transmit module
 */
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

/*
 *  Get analog in Transmit module
 */
uint8_t getPesAnalog(uint8_t *pesRawData){
	uint8_t analog;

	/* Analog Left Y Axis */
    analog = 0xff;
    if(pesRawData[5] > 230)
		analog = analog & 0xfe;
	else if(pesRawData[5] < 20)
		analog = analog & 0xfd;
	else
		analog = analog | 0x03;

	/* Analog Left X Axis */
	if(pesRawData[4] > 230)
		analog = analog & 0xfb;
	else if(pesRawData[4] < 20)
		analog = analog & 0xf7;
	else
        analog = analog | 0x0c;

	/* Analog Right Y Axis */
	if(pesRawData[3] > 230)
		analog = analog & 0xef;
	else if(pesRawData[3] < 20)
		analog = analog & 0xdf;
	else
		analog = analog | 0x30;

	/* Analog Right X Axis */
	if(pesRawData[2] > 230)
		analog = analog & 0xbf;
	else if(pesRawData[2] < 20)
		analog=analog & 0x7f;
	else
		analog=analog | 0xc0;
	return analog;
}

/**
 *  use for Debug transmit module
 */
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

/*This code used for receive Pes Data  --------------------------------------------------*/

/* Coppy and paste to Public variable ------------------------------------------------*/
UART_HandleTypeDef* UartReceive;
uint8_t u8_pesData;
uint16_t pesDigitalRawData = 0xFFFF;
uint8_t pesAnalogRawData = 0xFF;

/* Coppy and paste to main init ------------------------------------------------*/

void pes_receive_init(UART_HandleTypeDef* pUart){
	UartReceive = pUart;
	HAL_UART_Receive_IT(UartReceive, &u8_pesData, 1);
}

/* Coppy and paste to main loop ------------------------------------------------*/

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

	pesJoyStick.right_b.Up = (pesAnalogRawData & 0x80) >> 7;
	pesJoyStick.right_b.Down = (pesAnalogRawData & 0x40) >> 6;
	pesJoyStick.right_b.Left = (pesAnalogRawData & 0x20) >> 5;
	pesJoyStick.right_b.Right = (pesAnalogRawData & 0x10) >> 4;

	pesJoyStick.left_b.Up = (pesAnalogRawData & 0x08) >> 3;
	pesJoyStick.left_b.Down = (pesAnalogRawData & 0x04) >> 2;
	pesJoyStick.left_b.Left = (pesAnalogRawData & 0x02) >> 1;
	pesJoyStick.left_b.Right = (pesAnalogRawData & 0x01) ;

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	pes_uart_event_handle(huart);
}

/* Coppy and paste to HAL_UART_RxCpltCallback ------------------------------------------------*/

void pes_uart_event_handle(UART_HandleTypeDef *huart){
	static uint8_t byHeadIsTrue = 0;
	static uint8_t i_pes = 0;
	static uint8_t pesBuff[4];

	if(huart->Instance == UartReceive->Instance){
		if(u8_pesData == 'F' ){
			byHeadIsTrue = 1;
			i_pes = 0;
		}
		else{
			if(byHeadIsTrue){
				pesBuff[i_pes++] = u8_pesData;
				if(i_pes > 3){
					pesDigitalRawData = pesBuff[1] << 8 | pesBuff[0];
					pesAnalogRawData = pesBuff[2];
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
