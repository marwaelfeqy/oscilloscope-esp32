#include <Arduino.h>

#include "tft.h"
#include "game.h"
/*#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
#define LCD_RESET A4    // optional*/
void osc_draw();
// Assign human-readable names to some common 16-bit color values:
#define  BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

 unsigned long test_FillScreen(void)
{
  unsigned long start = micros();
  tft_FillScreen(RED);
  tft_FillScreen(GREEN);
  tft_FillScreen(BLUE);
  return (micros()-start);  
}

void Animation(void)
{
    for(int y = 0; y<320-92;  y+=10 )
    {
      tft_drawRGBBitmap( 10 ,  y, Game, 83 , 92);
      tft_FillScreen(WHITE);
    }
}

uint8_t sin_table[100]=
{

0x32,0x35,0x38,0x3b,0x3e,0x41,0x44,0x47,0x4a,0x4d,0x4f,0x52,0x54,0x56,0x59,0x5a,0x5c,0x5e,0x5f,0x60,
0x62,0x62,0x63,0x64,0x64,0x64,0x64,0x64,0x63,0x62,0x62,0x60,0x5f,0x5e,0x5c,0x5a,0x59,0x56,0x54,0x52,
0x4f,0x4d,0x4a,0x47,0x44,0x41,0x3e,0x3b,0x38,0x35,0x32,0x2f,0x2c,0x29,0x26,0x23,0x20,0x1d,0x1a,0x17,
0x15,0x12,0x10,0xe,0xb,0xa,0x8,0x6,0x5,0x4,0x2,0x2,0x1,0x0,0x0,0x0,0x0,0x0,0x1,0x2,
0x2,0x4,0x5,0x6,0x8,0xa,0xb,0xe,0x10,0x12,0x15,0x17,0x1a,0x1d,0x20,0x23,0x26,0x29,0x2c,0x2f


};

void DrawSineWave(int x , int y , uint16_t Color)
{
  int SineX = 0;
  for(int counter = x; counter < 320; counter++)
  {
    tft_drawPixel( sin_table[SineX++]+y ,counter , Color);
    if(SineX == 100)  
    {
      SineX = 0;
    }
    
    delay(10);
  }
  
}

void Set_BackGroundText(void)
{/*
  tft.setCursor(7, 10);
  tft.println("CH1"); 
  tft.setCursor(7, 20);
  tft.println("Vpp");
  tft.setCursor(7, 30); 
  tft.println("3.3V");

  tft.setCursor(7, 70);
  tft.println("CH1"); 
  tft.setCursor(7, 80);
  tft.println("Freq");
  tft.setCursor(7, 90); 
  tft.println("50Hz");

  tft.setCursor(7, 130);
  tft.println("CH1"); 
  tft.setCursor(7, 140);
  tft.println("Period");
  tft.setCursor(7, 150); 
  tft.println("1.98ms");

  tft.setCursor(7, 190);
  tft.println("CH1"); 
  tft.setCursor(7, 200);
  tft.println ("+Duty");
  tft.setCursor(7, 210); 
  tft.println("49.2%");
*/
  tft_FastVLine(185, 0, 240, RED);
  tft_FastHLine(50, 120, 320, RED);
  tft_DrawRect(0 , 0 , 320 , 240 , MAGENTA);
}

void SetBackGround(uint16_t color) {
 
 tft_FillScreen( 0);

  for(uint16_t x=50; x<240; x+=10) 
  {
      tft_FastVLine(x,0,320,color); 

  } 
    tft_FastVLine(0, 320, 240, color);
  
  for(uint16_t y=0; y<320; y+=10) 
  {
      tft_FastHLine(50,y,240,color);
  }
 // Set_BackGroundText();
}

void SetBackGround2(uint16_t color) {
 uint16_t divisionFactor = 60;

 tft_DrawRect( 50, 60, 270 , 100 , BLACK);

  for(uint16_t y=0; y<240; y+=10) 
  {
    if(y == divisionFactor)
    {
      divisionFactor += 60;
       tft_FastHLine(0, y, 320, color);

    }
    else 
    tft_FastHLine(50, y, 320, color);
  }
  for(uint16_t x=50; x<320; x+=10) 
  {
      tft_FastVLine(x, 0, 240, color);
  }

  tft_FastVLine(185, 0, 240, RED);
  tft_FastHLine(50, 120, 320, RED);
  tft_DrawRect(0 , 0 , 320 , 240 , MAGENTA);
}


int r,x=120,y;
float r0=0,r1=0,r2=0,r3=0,n=0;
void setup() {

  tft_lcd(LCD_CS , LCD_CD ,LCD_WR ,LCD_RD ,LCD_RESET);
  tft_init();
  tft_reset();
  tft_begin();
  setRotation(0);
  tft_oscBackground(YELLOW);
  /*
  tft_FillCircle(120,120,50,RED);
  tft_DrawCircle(170,120,50,YELLOW);
  tft_FillRect(10,200,50,50,MAGENTA);
  tft_DrawRoundRect(60,280,40,40,5,GREEN);
  tft_FillRoundRect(120,200,50,50,5,GREEN);
  tft_FillTriangle ( 240,320,240,280,210,300,BLUE);*/
}
void loop() {
osc_draw();

}
void osc_draw()
{
  r0=analogRead(36);
  r0=r0/4095*66;
  r=(int)r0;
  tft_DrawLine(x,y,r+120,y,RED);
  y++;
delayMicroseconds(1000);
  x=r+120;
  if(y>=320)
  {
    tft_oscRefill(YELLOW);
    y=0;
  }
}