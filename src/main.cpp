#include <Arduino.h>

#include "tft.h"
#include "game.h"
/*#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
#define LCD_RESET A4    // optional*/

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

void setup() {

  tft_lcd(LCD_CS , LCD_CD ,LCD_WR ,LCD_RD ,LCD_RESET);
  tft_init();
  tft_reset();
  tft_begin();
  
  tft_FillScreen(BLACK);
}
int r,x,y;
float r0=0,r1=0,r2=0,r3=0,n=0;
void loop() {
 
  r0=analogRead(36);
  r0=r0/4095*66;
  r1=analogRead(36);
  r1=r1/4095*66;
  r2=analogRead(36);
  r2=r2/4095*66;
  r3=analogRead(36);
  r3=r3/4095*66;
  r0=(r0+r1+r2+r3)/4;
  r=(int)r0;
  tft_DrawLine(RED,x,y,r+120,y);
  n+=0.125;
  y+=n;
  //delayMicroseconds(20);
  x=r+120;
  if(y>=320)
  {
    tft_FillScreen(BLACK);
    y=0;
  }
 if(n>=1)
 n=0;
   
}