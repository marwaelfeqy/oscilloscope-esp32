#include <Arduino.h>

#ifndef _TFT_H_
#define _TFT_H_


#define ARDUINO_UNO       0
#define ARDUINO_DUE       1
#define ESP32             2
#define DEFINED           1 
#define NOT_DEFINED       0 
/*
 * Select BOARD to one of this Options:
 * ARDUINO_UNO
 * ARDUINO_DUE
 * ESP32
 */
#define BOARD             ESP32

#if(BOARD == ARDUINO_DUE)
/**
 * if and only if the Selected Board was ARDUINO_DUE
 * you have to select the SHIELD to one of this Options:
 * DEFINED
 * NOT_DEFINED
 */
#define SHIELD    DEFINED
#endif
      
  #define setWriteDirInline() { DDRD |=  B11111100; DDRB |=  B00000011; }
  #define write8inline(d) {                          \
    PORTD = (PORTD & B00000011) | ((d) & B11111100); \
    PORTB = (PORTB & B11111100) | ((d) & B00000011); \
    WR_STROBE; }

 
    
/* Common Macros Begin */
#define SET_BIT(VAR,BIT)                      VAR |=  (1 << (BIT) )
#define CLR_BIT(VAR,BIT)                      VAR &= ~(1 << (BIT) )
#define TOG_BIT(VAR,BIT)                      VAR ^=  (1 << (BIT) ) 
#define GET_BIT(VAR,BIT)                      ((VAR >> BIT) & 1 )
/* Common Macros End  */


/* Macros Begin */

/* Arduino UNO  */
#if(BOARD == ARDUINO_UNO)
#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
#define LCD_RESET A4    // optional 
 #define RD_ACTIVE  *rdPort &=  rdPinUnset
 #define RD_IDLE    *rdPort |=  rdPinSet
 #define WR_ACTIVE  *wrPort &=  wrPinUnset
 #define WR_IDLE    *wrPort |=  wrPinSet
 #define CD_COMMAND *cdPort &=  cdPinUnset
 #define CD_DATA    *cdPort |=  cdPinSet
 #define CS_ACTIVE  *csPort &=  csPinUnset
 #define CS_IDLE    *csPort |=  csPinSet
 
  /* Arduino DUE */
#elif(BOARD == ARDUINO_DUE)
#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
#define LCD_RESET A4    // optional
#define RD_PORT    PIOA
#define WR_PORT    PIOA
#define CD_PORT    PIOA
#define CS_PORT    PIOA
#define RD_MASK 0x00010000             //PA16 
#define WR_MASK 0x01000000             //PA24
#define CD_MASK 0x00800000             //PA23
#define CS_MASK 0x00400000             //PA22
#define RD_ACTIVE  RD_PORT->PIO_CODR |= RD_MASK
#define RD_IDLE    RD_PORT->PIO_SODR |= RD_MASK
#define WR_ACTIVE  WR_PORT->PIO_CODR |= WR_MASK
#define WR_IDLE    WR_PORT->PIO_SODR |= WR_MASK
#define CD_COMMAND CD_PORT->PIO_CODR |= CD_MASK
#define CD_DATA    CD_PORT->PIO_SODR |= CD_MASK
#define CS_ACTIVE  CS_PORT->PIO_CODR |= CS_MASK
#define CS_IDLE    CS_PORT->PIO_SODR |= CS_MASK

/* ESP-32 */
#elif(BOARD == ESP32)
#define LCD_CS 1
#define LCD_CD 2
#define LCD_WR 3
#define LCD_RD 4
#define LCD_RESET 5     // optional
#define RD_MASK 0x10             //PA1 
#define WR_MASK 0x08             //PA2
#define CD_MASK 0x04             //PA3
#define CS_MASK 0x02             //PA4
#define RD_ACTIVE  GPIO.out_w1tc |= RD_MASK
#define RD_IDLE    GPIO.out_w1ts |= RD_MASK
#define WR_ACTIVE  GPIO.out_w1tc |= WR_MASK
#define WR_IDLE    GPIO.out_w1ts |= WR_MASK
#define CD_COMMAND GPIO.out_w1tc |= CD_MASK
#define CD_DATA    GPIO.out_w1ts |= CD_MASK
#define CS_ACTIVE  GPIO.out_w1tc |= CS_MASK
#define CS_IDLE    GPIO.out_w1ts |= CS_MASK    
//#define RD_ACTIVE  GPIO_HAL_GET_HW(0)->out_w1tc |= RD_MASK
//#define RD_IDLE    GPIO_HAL_GET_HW(0)->out_w1ts |= RD_MASK
//#define WR_ACTIVE  GPIO_HAL_GET_HW(0)->out_w1tc |= WR_MASK
//#define WR_IDLE    GPIO_HAL_GET_HW(0)->out_w1ts |= WR_MASK
//#define CD_COMMAND GPIO_HAL_GET_HW(0)->out_w1tc |= CD_MASK
//#define CD_DATA    GPIO_HAL_GET_HW(0)->out_w1ts |= CD_MASK
//#define CS_ACTIVE  GPIO_HAL_GET_HW(0)->out_w1tc |= CS_MASK
//#define CS_IDLE    GPIO_HAL_GET_HW(0)->out_w1ts |= CS_MASK

#endif





#define WR_STROBE { WR_ACTIVE; WR_IDLE; }
/* Macros End  */


/* TFT Specs Begin  */
#define TFTWIDTH   240
#define TFTHEIGHT  320
/* TFT Specs End  */

/* TFT Routines Begin  */
#define ILI9341_SOFTRESET          0x01
#define ILI9341_SLEEPIN            0x10
#define ILI9341_SLEEPOUT           0x11
#define ILI9341_NORMALDISP         0x13
#define ILI9341_INVERTOFF          0x20
#define ILI9341_INVERTON           0x21
#define ILI9341_GAMMASET           0x26
#define ILI9341_DISPLAYOFF         0x28
#define ILI9341_DISPLAYON          0x29
#define ILI9341_COLADDRSET         0x2A
#define ILI9341_PAGEADDRSET        0x2B
#define ILI9341_MEMORYWRITE        0x2C
#define ILI9341_PIXELFORMAT        0x3A
#define ILI9341_FRAMECONTROL       0xB1
#define ILI9341_DISPLAYFUNC        0xB6
#define ILI9341_ENTRYMODE          0xB7
#define ILI9341_POWERCONTROL1      0xC0
#define ILI9341_POWERCONTROL2      0xC1
#define ILI9341_VCOMCONTROL1       0xC5
#define ILI9341_VCOMCONTROL2       0xC7
#define ILI9341_MEMCONTROL         0x36
#define ILI9341_MADCTL             0x36

#define ILI9341_MADCTL_MY  0x80
#define ILI9341_MADCTL_MX  0x40
#define ILI9341_MADCTL_MV  0x20
#define ILI9341_MADCTL_ML  0x10
#define ILI9341_MADCTL_RGB 0x00
#define ILI9341_MADCTL_BGR 0x08
#define ILI9341_MADCTL_MH  0x04
/* TFT Routines End */

void write8(uint8_t value);
void writeRegister8(uint8_t a, uint8_t d);
void writeRegister16(uint16_t a, uint16_t d);
void writeRegister32(uint8_t r, uint32_t d);

void tft_lcd( uint8_t cs, uint8_t cd, uint8_t wr, uint8_t rd, uint8_t reset) ;

void tft_init(void);
void tft_reset(void);
void setAddrWindow(int x1, int y1, int x2, int y2);
void tft_begin(void);
void flood(uint16_t color, uint32_t len);
void tft_FillScreen(uint16_t color);
void tft_drawPixel(int16_t x, int16_t y, uint16_t color);
void tft_drawRGBBitmap(int16_t x, int16_t y,uint16_t* bitmap, int16_t w, int16_t h) ;
void tft_DrawLine( uint16_t x1, uint16_t y1,
											uint16_t x2, uint16_t y2,uint16_t color);
void tft_FastHLine(int16_t x, int16_t y, int16_t length,uint16_t color);
void tft_FastVLine( int16_t x, int16_t y, int16_t length,uint16_t color);
void tft_DrawCircle(uint16_t x0, uint16_t y0, int r, uint16_t color);
void tft_FillCircle(int16_t x0, int16_t y0, int16_t r,uint16_t color);

void tft_DrawCircleHelper( int16_t x0, int16_t y0,
               int16_t r, uint8_t cornername, uint16_t color);
void tft_FillCircleHelper(int16_t x0, int16_t y0, int16_t r,
    uint8_t cornername, int16_t delta, uint16_t color);

void tft_DrawRect( uint16_t x1, uint16_t y1,uint16_t x2, uint16_t y2,uint16_t color);
void tft_FillRect(int16_t x, int16_t y, int16_t w, int16_t h,uint16_t color);
void tft_DrawRoundRect(int16_t x, int16_t y, int16_t w,
  int16_t h, int16_t r, uint16_t color);
void tft_FillRoundRect(int16_t x, int16_t y, int16_t w,
				 int16_t h, int16_t r, uint16_t color);
void tft_DrawTriangle(int16_t x0, int16_t y0,
				int16_t x1, int16_t y1,
				int16_t x2, int16_t y2, uint16_t color);
void tft_FillTriangle ( int16_t x0, int16_t y0,
				  int16_t x1, int16_t y1,
				  int16_t x2, int16_t y2, uint16_t color);

void tft_oscBackground( uint16_t color);
void tft_oscRefill( uint16_t color);

#endif