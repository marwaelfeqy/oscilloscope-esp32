#include <Arduino.h>

#include "tft.h"

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



void tft_DrawLine(uint16_t color, uint16_t x1, uint16_t y1,uint16_t x2, uint16_t y2)
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



void tft_oscBackground( uint16_t color) {
  tft_FillScreen(0);
 for(int j=0;j<240;j+=40)
  {    
      for(int i=0;i<320;i++)
      {
        tft_drawPixel(j,i,color);
      }
  }
 for(int j=0;j<320;j+=40)
  {    
      for(int i=0;i<240;i++)
      {
        tft_drawPixel(i,j,color);
      }
  }
  
}
