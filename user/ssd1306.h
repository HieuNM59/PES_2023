
#include "stm32f1xx_hal.h"
#include "fonts.h"

#ifndef ssd1306
#define ssd1306

#define SSD1306_I2C_PORT		hi2c1
// I2C address 
#define SSD1306_I2C_ADDR        0x78

#define SSD1306_WIDTH           128

#define SSD1306_HEIGHT          64
#define ABS(x)   ((x) > 0 ? (x) : -(x))

typedef enum {
	Black = 0x00,
	White = 0x01,
} SSD1306_COLOR;

typedef struct {
	uint16_t CurrentX;
	uint16_t CurrentY;
	uint8_t Inverted;
	uint8_t Initialized;
} SSD1306_t;


extern I2C_HandleTypeDef SSD1306_I2C_PORT;

uint8_t ssd1306_Init(void);
void ssd1306_Fill(SSD1306_COLOR color);
void ssd1306_UpdateScreen(void);
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);
char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color);
char ssd1306_WriteString(char* str, FontDef Font, SSD1306_COLOR color);
void ssd1306_SetCursor(uint8_t x, uint8_t y);
void ssd1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, SSD1306_COLOR c);
void ssd1306_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_COLOR c);
void ssd1306_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_COLOR c);
void ssd1306_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1306_COLOR color);
void ssd1306_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1306_COLOR color);
void ssd1306_DrawCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR c);
void ssd1306_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR c);

void ssd1306_ON(void);
void ssd1306_OFF(void);

#endif
