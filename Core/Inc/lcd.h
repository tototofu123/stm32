/*
 * =============================================================================
 * LCD.H - LOW-LEVEL TFT LCD DISPLAY DRIVER API
 * =============================================================================
 *
 * This header declares the LCD hardware configuration, color constants, drawing
 * primitive functions, and text rendering for the 320x240 TFT panel.
 *
 * Responsibility (lcd.c implementation):
 * - Initialize LCD panel and FSMC external memory interface.
 * - Control LCD power (backlight, reset).
 * - Implement low-level primitives: clear region, draw line, draw rectangle.
 * - Implement pixel-level and text rendering using the ASCII font bitmap.
 * - Manage text color (foreground and background).
 * - Provide window-setting for efficient region updates.
 * - Support ellipse drawing for visual effects.
 *
 * Hardware configuration:
 * - FSMC Bank 1 (external memory interface) address: 0x60000000 (command/data).
 * - Reset pin: GPIO PE1.
 * - Backlight control: GPIO PD12.
 * - Display resolution: 320x240 pixels (16-bit RGB565).
 * - Character size: 8x16 pixels per ASCII glyph.
 *
 * Global variables exported:
 * - Current_TextColor: Active text foreground color.
 * - Current_TextBackColor: Active text background color.
 *
 * Data flow:
 * - ui.c and other modules call high-level drawing functions (LCD_DrawString, etc.).
 * - These functions use lower-level primitives (LCD_Clear, LCD_DrawLine, etc.).
 * - All coordinate systems: column (x, 0-239), page/row (y, 0-319).
 * - All colors are 16-bit RGB565 format (5-bit R, 6-bit G, 5-bit B).
 *
 * No C++ classes; pure C driver interface.
 * =============================================================================
 */

#ifndef __LCD_H
#define	__LCD_H

#include "stm32f1xx_hal.h"

/*
 * =============================================================================
 * FSMC HARDWARE MEMORY MAPPING
 * =============================================================================
 * External memory interface addresses for LCD command and data writes.
 */
/* LCD command register address (FSMC Bank 1, NS signal selects command mode) */
#define      FSMC_Addr_LCD_CMD         ( ( uint32_t ) 0x60000000 )	    
/* LCD data register address (FSMC Bank 1, address lines select data mode) */
#define      FSMC_Addr_LCD_DATA        ( ( uint32_t ) 0x60020000 )          

/* FSMC Bank selection for NOR/SRAM interface */
#define      FSMC_Bank1_NORSRAMx       FSMC_Bank1_NORSRAM1

/*
 * =============================================================================
 * LCD HARDWARE CONTROL PIN CONFIGURATION
 * =============================================================================
 */
/* LCD Reset pin (active-low): controls panel power-on sequence */
#define      LCD_RST_APBxClock_FUN     RCC_APB2PeriphClockCmd
#define      LCD_RST_CLK               RCC_APB2Periph_GPIOE   
#define      LCD_RST_PORT              GPIOE
#define      LCD_RST_PIN               GPIO_PIN_1

/* Backlight control pin: enables/disables LED backlight */
#define      LCD_BK_APBxClock_FUN      RCC_APB2PeriphClockCmd
#define      LCD_BK_CLK                RCC_APB2Periph_GPIOD    
#define      LCD_BK_PORT               GPIOD
#define      LCD_BK_PIN                GPIO_PIN_12

/* Debug timing (no-op in release builds) */
#define      DEBUG_DELAY()

/*
 * =============================================================================
 * LCD DISPLAY RESOLUTION CONSTANTS
 * =============================================================================
 */
/* Maximum display dimensions (panel size) */
#define      LCD_Default_Max_COLUMN	240     /* Display width in pixels */
#define      LCD_Default_Max_PAGE	320     /* Display height in pixels */

/* Viewport start position (typically full display) */
#define      LCD_DispWindow_Start_COLUMN	0     /* Viewport left edge */
#define      LCD_DispWindow_Start_PAGE		0     /* Viewport top edge */

/* Viewport dimensions (typically full display) */
#define      LCD_DispWindow_COLUMN	240     /* Viewport width */
#define      LCD_DispWindow_PAGE	320     /* Viewport height */

/*
 * =============================================================================
 * FONT AND TEXT CONSTANTS
 * =============================================================================
 */
/* ASCII character dimensions (8x16 monospace font from bitmap) */
#define      WIDTH_EN_CHAR		8	      /* Character width in pixels */
#define      HEIGHT_EN_CHAR		16		    /* Character height in pixels */

/* Chinese character lookup (stub; GBK code support not fully implemented) */
#define      GetGBKCode( ucBuffer, usChar )  	 

/*
 * =============================================================================
 * COLOR PALETTE (16-BIT RGB565 FORMAT)
 * =============================================================================
 * Each color is defined as 5-bit R, 6-bit G, 5-bit B packed into uint16_t.
 */
#define      WHITE		           0xFFFF	   /* R=31, G=63, B=31 (all on) */
#define      BLACK                         0x0000	 /* R=0,  G=0,  B=0 (all off) */
#define      GREY                          0xF7DE	 /* Light gray */
#define      BLUE                          0x001F	 /* R=0,  G=0,  B=31 (pure blue) */
#define      GREEN                         0x07E0	 /* R=0,  G=63, B=0 (pure green) */
#define      RED                           0xF800	 /* R=31, G=0,  B=0 (pure red) */
#define      MAGENTA                       0xF81F	 /* R=31, G=0,  B=31 (red+blue) */
#define      CYAN                          0x7FFF	 /* R=0,  G=63, B=31 (green+blue) */
#define      YELLOW                        0xFFE0	 /* R=31, G=63, B=0 (red+green) */
#define      BACKGROUND		           WHITE   /* Default background color */  


/*
 * =============================================================================
 * LCD CONTROLLER COMMAND CODES
 * =============================================================================
 * Commands sent to the ILI9325 (or compatible) LCD controller.
 */
#define      CMD_Set_COLUMN		   0x2A	     /* Set column address range */
#define      CMD_Set_PAGE		   0x2B	     /* Set row/page address range */
#define      CMD_SetPixel		   0x2C	     /* Write pixel data to frame buffer */     

/*
 * =============================================================================
 * GLOBAL TEXT RENDERING STATE
 * =============================================================================
 */
/* Current text drawing colors (used by LCD_DrawChar and LCD_DrawString) */
extern uint16_t Current_TextColor;      /* Foreground text color (RGB565) */
extern uint16_t Current_TextBackColor;  /* Background text color (RGB565) */

/*
 * =============================================================================
 * LCD HARDWARE INTERFACE FUNCTIONS
 * =============================================================================
 */
/*
 * LCD_INIT:
 * Initializes LCD panel: enables power, resets controller, loads register config.
 * Should be called once during system startup before any drawing operations.
 */
void            LCD_INIT		( void );

/*
 * LCD_Rst:
 * Toggles the LCD reset pin to hard-reset the panel controller.
 * Part of initialization sequence.
 */
void            LCD_Rst			( void );

/*
 * LCD_BackLed_Control:
 * Enables or disables the LCD backlight LED.
 * Input: enumState (ENABLE=backlight on, DISABLE=backlight off).
 */
void            LCD_BackLed_Control	( FunctionalState enumState );

/*
 * LCD_Write_Cmd:
 * Sends a command byte to the LCD controller via FSMC.
 * Input: usCmd (controller command code).
 */
void            LCD_Write_Cmd		( uint16_t usCmd );

/*
 * LCD_Write_Data:
 * Sends data to the LCD controller via FSMC (pixel color or register value).
 * Input: usData (16-bit color or data value).
 */
void            LCD_Write_Data		( uint16_t usData );

/*
 * LCD_Read_Data:
 * Reads data from the LCD controller (for diagnostics or pixel read-back).
 * Output: 16-bit color/data value read from panel.
 */
uint16_t        LCD_Read_Data		( void );

/*
 * =============================================================================
 * LCD DRAWING PRIMITIVES
 * =============================================================================
 */
/*
 * LCD_FillColor:
 * Fills a region with a solid color. Used by LCD_Clear and region redraws.
 * Input: usPoint (number of pixels to fill), usColor (RGB565 color).
 */
void		LCD_FillColor		( uint32_t usPoint, uint16_t usColor );

/*
 * LCD_OpenWindow:
 * Sets the active drawing window/region on the panel.
 * Subsequent pixel writes stay within this region.
 * Input: usC (column/x), usP (page/y), usWidth, usHeight.
 */
void            LCD_OpenWindow		( uint16_t usC, uint16_t usP, uint16_t usWidth, uint16_t usHeight );

/*
 * LCD_Clear:
 * Clears (fills with a color) a rectangular region of the display.
 * Input: usC (x), usP (y), usWidth, usHeight, usColor (fill color).
 */
void            LCD_Clear		( uint16_t usC, uint16_t usP, uint16_t usWidth, uint16_t usHeight, uint16_t usColor );

/*
 * LCD_GetPointPixel:
 * Reads the color value of a single pixel at the given coordinate.
 * Input: usC (x), usP (y).
 * Output: 16-bit RGB565 color.
 */
uint16_t        LCD_GetPointPixel	( uint16_t usC , uint16_t usP );

/*
 * LCD_DrawLine:
 * Draws a line between two points using Bresenham algorithm.
 * Input: usC1,usP1 (start x,y), usC2,usP2 (end x,y), usColor.
 */
void            LCD_DrawLine		( uint16_t usC1, uint16_t usP1, uint16_t usC2, uint16_t usP2, uint16_t usColor );

/*
 * LCD_DrawRectangle:
 * Draws a rectangle border (not filled) using four lines.
 * Input: usC (x), usP (y), usWidth, usHeight, usColor.
 */
void            LCD_DrawRectangle	( uint16_t usC, uint16_t usP, uint16_t usWidth, uint16_t usHeight, uint16_t usColor );

/*
 * LCD_DrawDot:
 * Draws a single pixel at the given coordinate.
 * Input: usC (x), usP (y), usColor.
 */
void            LCD_DrawDot		( uint16_t usC, uint16_t usP, uint16_t usColor );

/*
 * LCD_DrawEllipse:
 * Draws an ellipse (oval) shape at the given center.
 * Input: usC (center x), usP (center y), SR (short radius), LR (long radius), usColor.
 */
void 		LCD_DrawEllipse		( uint16_t usC, uint16_t usP, uint16_t SR, uint16_t LR, uint16_t usColor);

/*
 * =============================================================================
 * TEXT RENDERING FUNCTIONS
 * =============================================================================
 */
/*
 * LCD_SetColors:
 * Sets the foreground and background colors for subsequent text drawing.
 * Input: text (foreground RGB565), back (background RGB565).
 */
void            LCD_SetColors		( uint16_t text, uint16_t back );

/*
 * LCD_DrawChar:
 * Draws a single ASCII character at the given position using the bitmap font.
 * Input: usC (x), usP (y), cChar (ASCII character code).
 */
void            LCD_DrawChar		( uint16_t usC, uint16_t usP, const char cChar);

/*
 * LCD_DrawString:
 * Draws a null-terminated ASCII string starting at the given position.
 * Input: usC (x), usP (y), pStr (pointer to string).
 */
void            LCD_DrawString		( uint16_t usC, uint16_t usP, const char * pStr);

/*
 * LCD_DrawChinese:
 * Draws a Chinese character from a bitmap (stub implementation).
 * Input: usC (x), usP (y), bitmap (pointer to character bitmap data).
 * Note: Currently not fully implemented.
 */
void LCD_DrawChinese(uint16_t usC, uint16_t usP, const uint8_t *bitmap);

#endif  /* __LCD_H */ 
