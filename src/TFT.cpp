#include <Arduino.h>

#include "tft.h"


uint16_t TFTWIDTH =  240;
uint16_t TFTHEIGHT = 320;

 volatile uint8_t *csPort    , *cdPort    , *wrPort    , *rdPort;
    uint8_t           csPinSet  ,  cdPinSet  ,  wrPinSet  ,  rdPinSet  ,
            csPinUnset,  cdPinUnset,  wrPinUnset,  rdPinUnset,
            _reset;
void write8(uint8_t value)
{
  #if(BOARD == ARDUINO_UNO)
    write8inline(value);
  #elif(BOARD == ARDUINO_DUE) 
    #if(SHIELD == DEFINED)
        /* Note they're not in order  */
        digitalWrite(2 , GET_BIT(value,2) );
        digitalWrite(3 , GET_BIT(value,3) );
        digitalWrite(4 , GET_BIT(value,4) );
        digitalWrite(5 , GET_BIT(value,5) );
        digitalWrite(6 , GET_BIT(value,6) );
        digitalWrite(7 , GET_BIT(value,7) );
        digitalWrite(8 , GET_BIT(value,0) );
        digitalWrite(9 , GET_BIT(value,1) );
    #elif(SHIELD == NOT_DEFINED)
      // PORT Manipulation for Arduuno DUE
        PIO_Set(PIOC, (((value) & 0xFF)<<1)); 
        PIO_Clear(PIOC, (((~value) & 0xFF)<<1));
    #endif
    WR_ACTIVE; WR_IDLE; 
  #elif(BOARD == ESP32)
  //PORT Manipulation for ESP32
  GPIO.out_w1ts = (value<<12);
  GPIO.out_w1tc = ((~value)<<12);
  WR_ACTIVE; WR_IDLE; 
  #endif
   
  
}

void writeRegister8(uint8_t a, uint8_t d)
{
  CD_COMMAND; 
  write8(a); 
  CD_DATA; 
  write8(d); 
}
void writeRegister16(uint16_t a, uint16_t d)
{
  uint8_t hi, lo; 
  hi = (a) >> 8; 
  lo = (a); 
  CD_COMMAND; 
  write8(hi); 
  write8(lo); 
  hi = (d) >> 8;
  lo = (d); 
  CD_DATA   ; 
  write8(hi); 
  write8(lo); 
}

void writeRegister32(uint8_t r, uint32_t d) {
  CS_ACTIVE;
  CD_COMMAND;
  write8(r);
  CD_DATA;
  delayMicroseconds(10);
  write8(d >> 24);
  delayMicroseconds(10);
  write8(d >> 16);
  delayMicroseconds(10);
  write8(d >> 8);
  delayMicroseconds(10);
  write8(d);
  CS_IDLE;

}

// Initialization common to both shield & breakout configs
void tft_init(void) {

//#ifdef USE_ADAFRUIT_SHIELD_PINOUT
  CS_IDLE; // Set all control bits to idle state
  WR_IDLE;
  RD_IDLE;
  CD_DATA;
  digitalWrite(5, HIGH); // Reset line
  pinMode(LCD_CS, OUTPUT);   // Enable outputs
  pinMode(LCD_CD, OUTPUT);
  pinMode(LCD_WR, OUTPUT);
  pinMode(LCD_RD, OUTPUT);
  pinMode(LCD_RESET, OUTPUT);
//#endif

#if(BOARD == ARDUINO_UNO)
  //  setWriteDir(); // Set up LCD data port(s) for WRITE operations
  setWriteDirInline() 
#elif(BOARD == ARDUINO_DUE)
  #if(SHIELD == DEFINED)
// for DUE Slow Response
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);

  #elif(SHIELD == NOT_DEFINED)
  pinMode(33,OUTPUT);
  pinMode(34,OUTPUT);
  pinMode(35,OUTPUT);
  pinMode(36,OUTPUT);
  pinMode(37,OUTPUT);
  pinMode(38,OUTPUT);
  pinMode(39,OUTPUT);
  pinMode(40,OUTPUT);
  #endif

//For ESP32
#elif(BOARD == ESP32)
  pinMode(12,OUTPUT);
  pinMode(13,OUTPUT);
  pinMode(14,OUTPUT);
  pinMode(15,OUTPUT);
  pinMode(16,OUTPUT);
  pinMode(17,OUTPUT);
  pinMode(18,OUTPUT);
  pinMode(19,OUTPUT);   
#endif
//    gpio_reset_pin(12);
//    gpio_reset_pin(13);
//    gpio_reset_pin(14);
//    gpio_reset_pin(15);
//    gpio_reset_pin(16);
//    gpio_reset_pin(17);
//    gpio_reset_pin(18);
//    gpio_reset_pin(19); 
//    gpio_set_direction(12, GPIO_MODE_OUTPUT);    
//    gpio_set_direction(13, GPIO_MODE_OUTPUT);
//    gpio_set_direction(14, GPIO_MODE_OUTPUT);   
//    gpio_set_direction(15, GPIO_MODE_OUTPUT);   
//    gpio_set_direction(16, GPIO_MODE_OUTPUT);   
//    gpio_set_direction(17, GPIO_MODE_OUTPUT);   
//    gpio_set_direction(18, GPIO_MODE_OUTPUT);   
//    gpio_set_direction(19, GPIO_MODE_OUTPUT);   

 // rotation  = 0;
 // cursor_y  = cursor_x = 0;
 // textsize  = 1;
 // textcolor = 0xFFFF;
//  _width    = TFTWIDTH;
//  _height   = TFTHEIGHT;
}
void tft_lcd( uint8_t cs, uint8_t cd, uint8_t wr, uint8_t rd, uint8_t reset) 
{
  #if(BOARD == ARDUINO_UNO)
    _reset     = reset;
    csPort     = portOutputRegister(digitalPinToPort(cs));
    cdPort     = portOutputRegister(digitalPinToPort(cd));
    wrPort     = portOutputRegister(digitalPinToPort(wr));
    rdPort     = portOutputRegister(digitalPinToPort(rd));
    csPinSet   = digitalPinToBitMask(cs);
    cdPinSet   = digitalPinToBitMask(cd);
    wrPinSet   = digitalPinToBitMask(wr);
    rdPinSet   = digitalPinToBitMask(rd);
    csPinUnset = ~csPinSet;
    cdPinUnset = ~cdPinSet;
    wrPinUnset = ~wrPinSet;
    rdPinUnset = ~rdPinSet;
    *csPort   |=  csPinSet; // Set all control bits to HIGH (idle)
    *cdPort   |=  cdPinSet; // Signals are ACTIVE LOW
    *wrPort   |=  wrPinSet;
    *rdPort   |=  rdPinSet;
#elif(BOARD == ARDUINO_DUE)
    pinMode(cs, OUTPUT);    // Enable outputs
    pinMode(cd, OUTPUT);
    pinMode(wr, OUTPUT);
    pinMode(rd, OUTPUT);

#elif(BOARD == ESP32)
    pinMode(cs, OUTPUT);    // Enable outputs
    pinMode(cd, OUTPUT);
    pinMode(wr, OUTPUT);
    pinMode(rd, OUTPUT);
#endif
//    pinMode(cs, OUTPUT);    // Enable outputs
//    pinMode(cd, OUTPUT);
//    pinMode(wr, OUTPUT);
//    pinMode(rd, OUTPUT);
    if(reset)
    {
      digitalWrite(reset, HIGH);
      pinMode(reset, OUTPUT);
    }

  tft_init();
}

void tft_reset(void)
{
  CS_IDLE;
  WR_IDLE;
  RD_IDLE;
//#ifdef USE_ADAFRUIT_SHIELD_PINOUT
  digitalWrite(5, LOW);
  delay(2);
  digitalWrite(5, HIGH);
  // Data transfer sync
  CS_ACTIVE;
  CD_COMMAND;
  write8(0x00);
  for(uint8_t i=0; i<3; i++)  // Three extra 0x00s
  {
    WR_ACTIVE;
    WR_IDLE;
  }
  CS_IDLE;
}

void setAddrWindow(int x1, int y1, int x2, int y2) {
   uint32_t t;
   CS_ACTIVE;
    t = x1;
    t <<= 16;
    t |= x2;
    writeRegister32(ILI9341_COLADDRSET, t);  // HX8357D uses same registers!
    t = y1;
    t <<= 16;
    t |= y2;
    writeRegister32(ILI9341_PAGEADDRSET, t); // HX8357D uses same registers!
  CS_IDLE;
}

void tft_begin(void)
{
    uint8_t i = 0;
  tft_reset();
  delay(200);
  
  /********************/
    uint16_t a, d;
  ///////////////////////////////////////  driver = ID_9341;
  CS_ACTIVE;
//  digitalWrite(LCD_CS , LOW);
  
    writeRegister8(ILI9341_SOFTRESET, 0);
    delay(50);
    writeRegister8(ILI9341_DISPLAYOFF, 0);

    writeRegister8(ILI9341_POWERCONTROL1, 0x23);
    writeRegister8(ILI9341_POWERCONTROL2, 0x10);
    writeRegister16(ILI9341_VCOMCONTROL1, 0x2B2B);
    writeRegister8(ILI9341_VCOMCONTROL2, 0xC0);
    writeRegister8(ILI9341_MEMCONTROL, ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR);
    writeRegister8(ILI9341_PIXELFORMAT, 0x55);
    writeRegister16(ILI9341_FRAMECONTROL, 0x001B);
    
    writeRegister8(ILI9341_ENTRYMODE, 0x07);
    /* writeRegister32(ILI9341_DISPLAYFUNC, 0x0A822700);*/

    writeRegister8(ILI9341_SLEEPOUT, 0);
    delay(150);
    writeRegister8(ILI9341_DISPLAYON, 0);
    delay(500);
    setAddrWindow(0, 0, TFTWIDTH-1, TFTHEIGHT-1);
    return;

  /********************/

}

// Fast block fill operation for fillScreen, fillRect, H/V line, etc.
// Requires setAddrWindow() has previously been called to set the fill
// bounds.  'len' is inclusive, MUST be >= 1.
void flood(uint16_t color, uint32_t len) {
  uint16_t blocks;
  uint8_t  i, hi = color >> 8,
              lo = color;

  CS_ACTIVE;
  CD_COMMAND;
  write8(0x2C);
  // Write first pixel normally, decrement counter by 1
  CD_DATA;
  write8(hi);
  write8(lo);
  len--;

  blocks = (uint16_t)(len / 64); // 64 pixels/block
  if(hi == lo) {
    // High and low bytes are identical.  Leave prior data
    // on the port(s) and just toggle the write strobe.
    while(blocks--) {
      i = 16; // 64 pixels/block / 4 pixels/pass
      do {
        WR_STROBE; WR_STROBE; WR_STROBE; WR_STROBE; // 2 bytes/pixel
        WR_STROBE; WR_STROBE; WR_STROBE; WR_STROBE; // x 4 pixels
      } while(--i);
    }
    // Fill any remaining pixels (1 to 64)
    for(i = (uint8_t)len & 63; i--; ) {
      WR_STROBE;
      WR_STROBE;
    }
  } else {
    while(blocks--) {
      i = 16; // 64 pixels/block / 4 pixels/pass
      do {
        write8(hi); write8(lo); write8(hi); write8(lo);
        write8(hi); write8(lo); write8(hi); write8(lo);
      } while(--i);
    }
    for(i = (uint8_t)len & 63; i--; ) {
      write8(hi);
      write8(lo);
    }
  }
  CS_IDLE;
}

void tft_FillScreen(uint16_t color)
{
 // setAddrWindow(0, 0, _width - 1, _height - 1);
   setAddrWindow(0, 0, TFTWIDTH - 1, TFTHEIGHT - 1);
  flood(color, (long)TFTWIDTH * (long)TFTHEIGHT);
  
}

void tft_drawPixel(int16_t x, int16_t y, uint16_t color) 
{

  // Clip
  if((x < 0) || (y < 0) || (x >= TFTWIDTH) || (y >= TFTHEIGHT)) return;

  CS_ACTIVE;
    setAddrWindow(x, y, TFTWIDTH-1, TFTHEIGHT-1);
    CS_ACTIVE;
    CD_COMMAND; 
    write8(0x2C);
    CD_DATA; 
    write8(color >> 8); write8(color);
  
  CS_IDLE;
}


void tft_drawRGBBitmap(int16_t x, int16_t y,uint16_t* bitmap, int16_t w, int16_t h)
{
  for (int16_t j = 0; j < h; j++, y++) {
    for (int16_t i = 0; i < w; i++) {
      tft_drawPixel(x + i, y, bitmap[j * w + i]);
    }
  }

}

void tft_DrawLine( uint16_t x1, uint16_t y1,uint16_t x2, uint16_t y2,uint16_t color)
{
  int steep = abs(y2-y1)>abs(x2-x1);
  uint16_t temp=0;
  if(steep)
  {
    temp=x1;
    x1=y1;
    y1=temp;
    temp=x2;
    x2=y2;
    y2=temp;
  }
  if(x1>x2)
  {
    temp=x1;
    x1=x2;
    x2=temp;
    temp=y1;
    y1=y2;
    y2=temp;
  }
  int dx,dy;
  dx=x2-x1;
  dy=abs(y2-y1);
  int err=dx/2;
  int ystep;
  if(y1<y2) ystep=1;
  else ystep=-1;
  for(;x1<=x2;x1++)
  {
    if(steep) tft_drawPixel(y1,x1,color);
    else tft_drawPixel(x1,y1,color);
    err-=dy;
    if(err<0)
    {
      y1 += ystep;
      err=dx;
    }
  }
}

void tft_FastVLine(int16_t x, int16_t y, int16_t length,uint16_t color)
{
  int16_t y2;

  // Initial off-screen clipping
  if((length <= 0      ) ||
     (x      <  0      ) || ( x                  >= TFTWIDTH) ||
     (y      >= TFTHEIGHT) || ((y2 = (y+length-1)) <  0     )) return;
  if(y < 0) {         // Clip top
    length += y;
    y       = 0;
  }
  if(y2 >= TFTHEIGHT) { // Clip bottom
    y2      = TFTHEIGHT - 1;
    length  = y2 - y + 1;
  }

  setAddrWindow(x, y, x, y2);
  flood(color, length);
  setAddrWindow(0, 0, TFTWIDTH - 1, TFTHEIGHT - 1);
  
}


void tft_FastHLine(int16_t x, int16_t y, int16_t length,
  uint16_t color)
{
  int16_t x2;

  // Initial off-screen clipping
  if((length <= 0     ) ||
     (y      <  0     ) || ( y                  >= TFTHEIGHT ) ||
     (x      >= TFTWIDTH) || ((x2 = (x+length-1)) <  0      )) return;

  if(x < 0) {        // Clip left
    length += x;
    x       = 0;
  }
  if(x2 >= TFTWIDTH) { // Clip right
    x2      = TFTWIDTH - 1;
    length  = x2 - x + 1;
  }

  setAddrWindow(x, y, x2, y);
  flood(color, length);
  setAddrWindow(0, 0, TFTWIDTH - 1, TFTHEIGHT - 1);
  
}

void tft_DrawCircle(uint16_t x0, uint16_t y0, int r, uint16_t color)
{
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  tft_drawPixel(x0  , y0+r, color);
  tft_drawPixel(x0  , y0-r, color);
  tft_drawPixel(x0+r, y0  , color);
  tft_drawPixel(x0-r, y0  , color);

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
  
    tft_drawPixel(x0 + x, y0 + y, color);
    tft_drawPixel(x0 - x, y0 + y, color);
    tft_drawPixel(x0 + x, y0 - y, color);
    tft_drawPixel(x0 - x, y0 - y, color);
    tft_drawPixel(x0 + y, y0 + x, color);
    tft_drawPixel(x0 - y, y0 + x, color);
    tft_drawPixel(x0 + y, y0 - x, color);
    tft_drawPixel(x0 - y, y0 - x, color);
  }
}

void tft_DrawCircleHelper( int16_t x0, int16_t y0,
               int16_t r, uint8_t cornername, uint16_t color) {
  int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;
  int16_t y     = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;
    if (cornername & 0x4) {
      tft_drawPixel(x0 + x, y0 + y, color);
      tft_drawPixel(x0 + y, y0 + x, color);
    } 
    if (cornername & 0x2) {
      tft_drawPixel(x0 + x, y0 - y, color);
      tft_drawPixel(x0 + y, y0 - x, color);
    }
    if (cornername & 0x8) {
      tft_drawPixel(x0 - y, y0 + x, color);
      tft_drawPixel(x0 - x, y0 + y, color);
    }
    if (cornername & 0x1) {
      tft_drawPixel(x0 - y, y0 - x, color);
      tft_drawPixel(x0 - x, y0 - y, color);
    }
  }
}

void tft_FillCircleHelper(int16_t x0, int16_t y0, int16_t r,
    uint8_t cornername, int16_t delta, uint16_t color) {

  int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;
  int16_t y     = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;

    if (cornername & 0x1) {
      tft_FastVLine(x0+x, y0-y, (2*y+1+delta), color);
      tft_FastVLine(x0+y, y0-x, 2*x+1+delta, color);
    }
    if (cornername & 0x2) {
      tft_FastVLine(x0-x, y0-y, 2*y+1+delta, color);
      tft_FastVLine(x0-y, y0-x, 2*x+1+delta, color);
    }
  }
}

void tft_FillCircle(int16_t x0, int16_t y0, int16_t r,
			      uint16_t color) {
  tft_FastVLine(x0, y0-r, 2*r+1, color);
  tft_FillCircleHelper(x0, y0, r, 3,0, color);
}
void tft_DrawRect( uint16_t x1, uint16_t y1,
											uint16_t x2, uint16_t y2,uint16_t color)
{
	tft_FastHLine(x1,y1,x2,color);
	tft_FastVLine(x2,y1,y2,color);
	tft_FastVLine(x1,y1,y2,color);
	tft_FastHLine(x1,y2,x2,color);
}
void tft_FillRect(int16_t x, int16_t y, int16_t w, int16_t h,
			    uint16_t color) {
  // Update in subclasses if desired!
  for (int16_t i=x; i<x+w; i++) {
    tft_FastVLine(i, y, h, color);
  }
}

void tft_DrawRoundRect(int16_t x, int16_t y, int16_t w,
  int16_t h, int16_t r, uint16_t color) {
  // smarter version
  tft_FastHLine(x+r  , y    , w-2*r, color); // Top
  tft_FastHLine(x+r  , y+h-1, w-2*r, color); // Bottom
  tft_FastVLine(x    , y+r  , h-2*r, color); // Left
  tft_FastVLine(x+w-1, y+r  , h-2*r, color); // Right
  // draw four corners
  tft_DrawCircleHelper(x+r    , y+r    , r, 1, color);
  tft_DrawCircleHelper(x+w-r-1, y+r    , r, 2, color);
  tft_DrawCircleHelper(x+w-r-1, y+h-r-1, r, 4, color);
  tft_DrawCircleHelper(x+r    , y+h-r-1, r, 8, color);
}
void tft_FillRoundRect(int16_t x, int16_t y, int16_t w,
				 int16_t h, int16_t r, uint16_t color) {
  // smarter version
  tft_FillRect(x+r, y, w-2*r, h, color);

  // draw four corners
  tft_FillCircleHelper(x+w-r-1, y+r, r, 1, h-2*r-1, color);
  tft_FillCircleHelper(x+r    , y+r, r, 2, h-2*r-1, color);
}

void tft_DrawTriangle(int16_t x0, int16_t y0,

				int16_t x1, int16_t y1,
				int16_t x2, int16_t y2, uint16_t color) {
  tft_DrawLine(x0, y0, x1, y1, color);
  tft_DrawLine(x1, y1, x2, y2, color);
  tft_DrawLine(x2, y2, x0, y0, color);
}
void tft_FillTriangle ( int16_t x0, int16_t y0,
				  int16_t x1, int16_t y1,
				  int16_t x2, int16_t y2, uint16_t color) {

  int16_t a, b, y,z, last;

  // Sort coordinates by Y order (y2 >= y1 >= y0)
  if (y0 > y1) 
    {
     z=y0;
     y0=y1;
     y1=z;
     z=x0;
     x0=x1;
     x1=z;
   }
  
  if (y1 > y2) {
     z=y2;
     y2=y1;
     y1=z;
     z=x2;
     x2=x1;
     x1=z;
  }
  if (y0 > y1) {
    z=y0;
     y0=y1;
     y1=z;
     z=x0;
     x0=x1;
     x1=z;
 }

  if(y0 == y2) { // Handle awkward all-on-same-line case as its own thing
    a = b = x0;
    if(x1 < a)      a = x1;
    else if(x1 > b) b = x1;
    if(x2 < a)      a = x2;
    else if(x2 > b) b = x2;
    tft_FastHLine(a, y0, b-a+1, color);
    return;
  }

  int16_t
    dx01 = x1 - x0,
    dy01 = y1 - y0,
    dx02 = x2 - x0,
    dy02 = y2 - y0,
    dx12 = x2 - x1,
    dy12 = y2 - y1;
  int32_t
    sa   = 0,
    sb   = 0;

  // For upper part of triangle, find scanline crossings for segments
  // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
  // is included here (and second loop will be skipped, avoiding a /0
  // error there), otherwise scanline y1 is skipped here and handled
  // in the second loop...which also avoids a /0 error here if y0=y1
  // (flat-topped triangle).
  if(y1 == y2) last = y1;   // Include y1 scanline
  else         last = y1-1; // Skip it

  for(y=y0; y<=last; y++) {
    a   = x0 + sa / dy01;
    b   = x0 + sb / dy02;
    sa += dx01;
    sb += dx02;
    /* longhand:
    a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
    */
   if(a > b)
   {
     z=a;
   a=b;
   b=z;
   }
    tft_FastHLine(a, y, b-a+1, color);
  }

  // For lower part of triangle, find scanline crossings for segments
  // 0-2 and 1-2.  This loop is skipped if y1=y2.
  sa = dx12 * (y - y1);
  sb = dx02 * (y - y0);
  for(; y<=y2; y++) {
    a   = x1 + sa / dy12;
    b   = x0 + sb / dy02;
    sa += dx12;
    sb += dx02;
    /* longhand:
    a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
    */
    if(a > b) {
     z=a;
   a=b;
   b=z;
   }
    tft_FastHLine(a, y, b-a+1, color);
  }
}

void tft_oscBackground( uint16_t color) {
  tft_FillScreen(0);
 for(int j=0;j<240;j+=40)
  {  
    tft_FastVLine(j,0,320,color);  
  }
 for(int j=0;j<320;j+=40)
  {    
      tft_FastHLine(0,j,240,color);
  }
  
}

void tft_oscRefill( uint16_t color) {
  setAddrWindow(110,0, 190, TFTHEIGHT - 1);
  flood(0, 240*320);
 for(int j=120;j<200;j+=40)
  {  
    tft_FastVLine(j,0,320,color);  
  }
 for(int j=0;j<320;j+=40)
  {    
      tft_FastHLine(0,j,240,color);
  }
  
}

void setRotation(uint8_t x) {
  
   // MEME, HX8357D uses same registers as 9341 but different values
   uint16_t t,rotation;
   rotation=x & 3;
   
   switch(rotation) {
   case 0:
   case 2:
    TFTWIDTH  = WIDTH;
    TFTHEIGHT = HEIGHT;
    break;
   case 1:
   case 3:
    TFTWIDTH  = HEIGHT;
    TFTHEIGHT = WIDTH;
    break;
  }
  
  CS_ACTIVE;
   switch (rotation) {
   case 2:
     t = ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR;
     break;
   case 3:
     t = ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR;
     break;
  case 0:
    t = ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR;
    break;
   case 1:
     t = ILI9341_MADCTL_MX | ILI9341_MADCTL_MY | ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR;
     break;
  }
   writeRegister8(ILI9341_MADCTL, t ); // MADCTL
   // For 9341, init default full-screen address window:
   setAddrWindow(0, 0, TFTWIDTH - 1, TFTHEIGHT - 1); // CS_IDLE happens here
  
}