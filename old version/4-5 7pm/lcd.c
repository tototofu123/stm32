#include "lcd.h"
#include "ascii.h"
#include <math.h>

void LCD_REG_Config(void);
void LCD_FillColor(uint32_t ulAmout_Point, uint16_t usColor);
uint16_t LCD_Read_PixelData(void);

void Delay(__IO uint32_t nCount) { for (; nCount != 0; nCount--); }

void LCD_INIT(void)
{
	LCD_BackLed_Control(ENABLE);
	LCD_Rst();
	LCD_REG_Config();
	LCD_Clear(0, 0, 240, 320, BACKGROUND);
}

void LCD_Rst(void)
{
	HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_RESET);
	Delay(0xAFFf << 2);
	HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_SET);
	Delay(0xAFFf << 2);
}

void LCD_BackLed_Control(FunctionalState enumState)
{
	if (enumState)
		HAL_GPIO_WritePin(LCD_BK_PORT, LCD_BK_PIN, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LCD_BK_PORT, LCD_BK_PIN, GPIO_PIN_SET);
}

void LCD_Write_Cmd(uint16_t usCmd)
{
	*(__IO uint16_t *)(FSMC_Addr_LCD_CMD) = usCmd;
}

void LCD_Write_Data(uint16_t usData)
{
	*(__IO uint16_t *)(FSMC_Addr_LCD_DATA) = usData;
}

uint16_t LCD_Read_Data(void)
{
	return (*(__IO uint16_t *)(FSMC_Addr_LCD_DATA));
}

void LCD_REG_Config(void)
{
	DEBUG_DELAY();
	LCD_Write_Cmd(0xCF); LCD_Write_Data(0x00); LCD_Write_Data(0x81); LCD_Write_Data(0x30);
	DEBUG_DELAY();
	LCD_Write_Cmd(0xED); LCD_Write_Data(0x64); LCD_Write_Data(0x03); LCD_Write_Data(0x12); LCD_Write_Data(0x81);
	DEBUG_DELAY();
	LCD_Write_Cmd(0xE8); LCD_Write_Data(0x85); LCD_Write_Data(0x10); LCD_Write_Data(0x78);
	DEBUG_DELAY();
	LCD_Write_Cmd(0xCB); LCD_Write_Data(0x39); LCD_Write_Data(0x2C); LCD_Write_Data(0x00); LCD_Write_Data(0x34); LCD_Write_Data(0x02);
	DEBUG_DELAY();
	LCD_Write_Cmd(0xF7); LCD_Write_Data(0x20);
	DEBUG_DELAY();
	LCD_Write_Cmd(0xEA); LCD_Write_Data(0x00); LCD_Write_Data(0x00);
	DEBUG_DELAY();
	LCD_Write_Cmd(0xB1); LCD_Write_Data(0x00); LCD_Write_Data(0x1B);
	DEBUG_DELAY();
	LCD_Write_Cmd(0xB6); LCD_Write_Data(0x0A); LCD_Write_Data(0xA2);
	DEBUG_DELAY();
	LCD_Write_Cmd(0xC0); LCD_Write_Data(0x35);
	DEBUG_DELAY();
	LCD_Write_Cmd(0xC1); LCD_Write_Data(0x11);
	LCD_Write_Cmd(0xC5); LCD_Write_Data(0x45); LCD_Write_Data(0x45);
	LCD_Write_Cmd(0xC7); LCD_Write_Data(0xA2);
	LCD_Write_Cmd(0xF2); LCD_Write_Data(0x00);
	LCD_Write_Cmd(0x26); LCD_Write_Data(0x01);
	DEBUG_DELAY();
	LCD_Write_Cmd(0xE0); LCD_Write_Data(0x0F); LCD_Write_Data(0x26); LCD_Write_Data(0x24); LCD_Write_Data(0x0B); LCD_Write_Data(0x0E); LCD_Write_Data(0x09); LCD_Write_Data(0x54); LCD_Write_Data(0xA8); LCD_Write_Data(0x46); LCD_Write_Data(0x0C); LCD_Write_Data(0x17); LCD_Write_Data(0x09); LCD_Write_Data(0x0F); LCD_Write_Data(0x07); LCD_Write_Data(0x00);
	LCD_Write_Cmd(0XE1); LCD_Write_Data(0x00); LCD_Write_Data(0x19); LCD_Write_Data(0x1B); LCD_Write_Data(0x04); LCD_Write_Data(0x10); LCD_Write_Data(0x07); LCD_Write_Data(0x2A); LCD_Write_Data(0x47); LCD_Write_Data(0x39); LCD_Write_Data(0x03); LCD_Write_Data(0x06); LCD_Write_Data(0x06); LCD_Write_Data(0x30); LCD_Write_Data(0x38); LCD_Write_Data(0x0F);
	DEBUG_DELAY();
	LCD_Write_Cmd(0x36); LCD_Write_Data(0xC8);
	DEBUG_DELAY();
	LCD_Write_Cmd(CMD_Set_COLUMN); LCD_Write_Data(0x00); LCD_Write_Data(0x00); LCD_Write_Data(0x00); LCD_Write_Data(0xEF);
	DEBUG_DELAY();
	LCD_Write_Cmd(CMD_Set_PAGE); LCD_Write_Data(0x00); LCD_Write_Data(0x00); LCD_Write_Data(0x01); LCD_Write_Data(0x3F);
	DEBUG_DELAY();
	LCD_Write_Cmd(0x3a); LCD_Write_Data(0x55);
	LCD_Write_Cmd(0x11);
	Delay(0xAFFf << 2);
	DEBUG_DELAY();
	LCD_Write_Cmd(0x29);
}

void LCD_OpenWindow(uint16_t usCOLUMN, uint16_t usPAGE, uint16_t usWidth, uint16_t usHeight)
{
	LCD_Write_Cmd(CMD_Set_COLUMN);
	LCD_Write_Data(usCOLUMN >> 8); LCD_Write_Data(usCOLUMN & 0xff);
	LCD_Write_Data((usCOLUMN + usWidth - 1) >> 8); LCD_Write_Data((usCOLUMN + usWidth - 1) & 0xff);
	LCD_Write_Cmd(CMD_Set_PAGE);
	LCD_Write_Data(usPAGE >> 8); LCD_Write_Data(usPAGE & 0xff);
	LCD_Write_Data((usPAGE + usHeight - 1) >> 8); LCD_Write_Data((usPAGE + usHeight - 1) & 0xff);
}

void LCD_FillColor(uint32_t usPoint, uint16_t usColor)
{
	uint32_t i = 0;
	LCD_Write_Cmd(CMD_SetPixel);
	for (i = 0; i < usPoint; i++) LCD_Write_Data(usColor);
}

void LCD_Clear(uint16_t usCOLUMN, uint16_t usPAGE, uint16_t usWidth, uint16_t usHeight, uint16_t usColor)
{
	LCD_OpenWindow(usCOLUMN, usPAGE, usWidth, usHeight);
	LCD_FillColor(usWidth * usHeight, usColor);
}

uint16_t LCD_Read_PixelData(void)
{
	uint16_t usR = 0, usG = 0, usB = 0;
	LCD_Write_Cmd(0x2E);
	usR = LCD_Read_Data(); usR = LCD_Read_Data(); usB = LCD_Read_Data(); usG = LCD_Read_Data();
	return (((usR >> 11) << 11) | ((usG >> 10) << 5) | (usB >> 11));
}

uint16_t LCD_GetPointPixel(uint16_t usCOLUMN, uint16_t usPAGE)
{
	LCD_OpenWindow(usCOLUMN, usPAGE, 1, 1);
	return LCD_Read_PixelData();
}

void LCD_DrawLine(uint16_t usC1, uint16_t usP1, uint16_t usC2, uint16_t usP2, uint16_t usColor)
{
	uint16_t us;
	uint16_t usC_Current, usP_Current;
	int32_t lError_C = 0, lError_P = 0, lDelta_C, lDelta_P, lDistance;
	int32_t lIncrease_C, lIncrease_P;
	lDelta_C = (int32_t)usC2 - (int32_t)usC1; lDelta_P = (int32_t)usP2 - (int32_t)usP1;
	usC_Current = usC1; usP_Current = usP1;
	if (lDelta_C > 0) lIncrease_C = 1; else if (lDelta_C == 0) lIncrease_C = 0; else { lIncrease_C = -1; lDelta_C = -lDelta_C; }
	if (lDelta_P > 0) lIncrease_P = 1; else if (lDelta_P == 0) lIncrease_P = 0; else { lIncrease_P = -1; lDelta_P = -lDelta_P; }
	if (lDelta_C > lDelta_P) lDistance = lDelta_C; else lDistance = lDelta_P;
	for (us = 0; us <= lDistance + 1; us++) {
		LCD_DrawDot(usC_Current, usP_Current, usColor);
		lError_C += lDelta_C; lError_P += lDelta_P;
		if (lError_C > lDistance) { lError_C -= lDistance; usC_Current += (uint16_t)lIncrease_C; }
		if (lError_P > lDistance) { lError_P -= lDistance; usP_Current += (uint16_t)lIncrease_P; }
	}
}

void LCD_DrawRectangle(uint16_t usC, uint16_t usP, uint16_t usWidth, uint16_t usHeight, uint16_t usColor)
{
    LCD_DrawLine(usC, usP, usC + usWidth, usP, usColor);
    LCD_DrawLine(usC, usP + usHeight, usC + usWidth, usP + usHeight, usColor);
    LCD_DrawLine(usC, usP, usC, usP + usHeight, usColor);
    LCD_DrawLine(usC + usWidth, usP, usC + usWidth, usP + usHeight, usColor);
}

uint16_t Current_TextColor = BLUE;
uint16_t Current_TextBackColor = WHITE;

void LCD_SetColors(uint16_t text, uint16_t back) { Current_TextColor = text; Current_TextBackColor = back; }

void LCD_DrawChar(uint16_t usC, uint16_t usP, const char cChar)
{
	uint8_t ucTemp, ucRelativePositon, ucPage, ucColumn;
	ucRelativePositon = cChar - ' ';
	LCD_OpenWindow(usC, usP, WIDTH_EN_CHAR, HEIGHT_EN_CHAR);
	LCD_Write_Cmd(CMD_SetPixel);
	for (ucPage = 0; ucPage < HEIGHT_EN_CHAR; ucPage++) {
		ucTemp = ucAscii_1608[ucRelativePositon][ucPage];
		for (ucColumn = 0; ucColumn < WIDTH_EN_CHAR; ucColumn++) {
			if (ucTemp & 0x01) LCD_Write_Data(Current_TextColor); else LCD_Write_Data(Current_TextBackColor);
			ucTemp >>= 1;
		}
	}
}

void LCD_DrawString(uint16_t usC, uint16_t usP, const char *pStr)
{
	while (*pStr != '\0') {
		if ((usC - LCD_DispWindow_Start_COLUMN + WIDTH_EN_CHAR) > LCD_DispWindow_COLUMN) { usC = LCD_DispWindow_Start_COLUMN; usP += HEIGHT_EN_CHAR; }
		if ((usP - LCD_DispWindow_Start_PAGE + HEIGHT_EN_CHAR) > LCD_DispWindow_PAGE) { usC = LCD_DispWindow_Start_COLUMN; usP = LCD_DispWindow_Start_PAGE; }
		LCD_DrawChar(usC, usP, *pStr);
		pStr++; usC += WIDTH_EN_CHAR;
	}
}

void LCD_DrawDot(uint16_t usCOLUMN, uint16_t usPAGE, uint16_t usColor)
{
	LCD_OpenWindow(usCOLUMN, usPAGE, 1, 1);
	LCD_FillColor(1, usColor);
}

void LCD_DrawEllipse(uint16_t usC, uint16_t usP, uint16_t SR, uint16_t LR, uint16_t usColor)
{
	float angle; uint16_t x, y;
	for (angle = 0; angle < 6.2832f; angle += 0.001f) { x = (uint16_t)(usC + LR * cos(angle)); y = (uint16_t)(usP + SR * sin(angle)); LCD_DrawDot(x, y, usColor); }
}

void LCD_DrawChinese(uint16_t usC, uint16_t usP, const uint8_t *bitmap)
{
    uint16_t row, col, byte_idx, bit;
    for (row = 0; row < 24; row++) {
        for (col = 0; col < 24; col++) {
            byte_idx = row * 3 + col / 8; bit = col % 8;
            if (bitmap[byte_idx] & (0x80 >> bit)) LCD_DrawDot(usC + col, usP + row, BLACK);
        }
    }
}
