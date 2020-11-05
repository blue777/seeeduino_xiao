#include <TimerTC3.h>
#include <Wire.h>

#include "_common/ctrl_i2c.h"
#include "_common/ctrl_pmoni.h"
#include "_common/display_ssd1306_i2c.h"

#include "_common/bitmap_font_render.h"
#include "_common/bitmap_font16.h"
#include "_common/bitmap_font32.h"
#include "_common/bitmap_font48.h"

#include "_common/bitmap_font24.h"
#include "_common/bitmap_font40.h"


#define GPIO_LED_R       7
#define GPIO_LED_G       8
#define GPIO_LED_B       9
#define GPIO_ROTARY_SW   1
#define GPIO_ROTARY_A    2
#define GPIO_ROTARY_B    3
#define GPIO_ALERT       6

#define OVER_CURRENT_PROTECT     8 // [A]



class i2c_mcp4726 : public ctrl_i2c
{
  public:
    i2c_mcp4726() : ctrl_i2c( 0x60 )
    {
    }

    void  SetValue( uint16_t value )
    {
      uint8_t reg = 0x40;

      // Vref
      // 0: VDD                           (6.4v max)
      // 2: Vref with internally buffer   (5.2v max)
      // 3: Vref unbuffered               (5.2v max)
      reg |= 3 << 3;

      // PD
      // 0: Normal opertion
      // 1: 1k ohm resistor to ground
      // 2: 125k
      // 3: 640k
      reg |= 0 << 1;

      // Gain
      // 0: x1
      // 1: x2
      reg |= 1 << 0;

      // 6.2 Write Volatile Memory
      // 0
      // 1
      // 0
      // VREF1
      // VREF0
      // PD1
      // PD0
      // G
      uint8_t  w_data[3]  = { reg, (uint8_t)(0xFF & (value >> 8)), (uint8_t)(0xFF & value) };
      write( w_data, sizeof(w_data) );
    }
};






int   g_nRotaryA = 0;
int   g_nCtrlFine = 0;
int   g_nDacOut = 0;
int   g_isUpdateDac = 0;
bool  g_bRotarySwState  = false;
bool  g_bRotarySwIgnore  = false;

PMoni_INA226        g_iPowerMon(0x40);
Display_SSD1306_i2c g_iSSD1306;
i2c_mcp4726         g_iMCP4726;


void  UpdateLED( int value4095 )
{
//  int color_table[] = { 0x000000, 0xFF0000, 0xFFFF00, 0x00FF00, 0x00FFFF, 0x0000FF, 0xFF00FF, 0xFFFFFF };
  int color_table[] = { 0xFF0000, 0xFFFF00, 0x00FF00, 0x00FFFF, 0x0000FF, 0xFF00FF, 0xFFFFFF };
  //int color_table[] = { 0x0000FF, 0x00FF00, 0xFF0000, 0xFFFFFF };

  int color_table_cnt = sizeof(color_table) / sizeof(color_table[0]);
  int color_index = value4095 * (color_table_cnt - 1) * 256 / 4095;
  int index = color_index >> 8;
  int ratio = color_index & 0xFF;
  
  if ( index == color_table_cnt - 1 )
  {
    index = color_table_cnt - 2;
    ratio = 256;
  }
  
  int R = (((color_table[index] >> 16) & 0xFF) * (256 - ratio) + ((color_table[index + 1] >> 16) & 0xFF) * ratio) >> 8;
  int G = (((color_table[index] >>  8) & 0xFF) * (256 - ratio) + ((color_table[index + 1] >>  8) & 0xFF) * ratio) >> 8;
  int B = (((color_table[index] >>  0) & 0xFF) * (256 - ratio) + ((color_table[index + 1] >>  0) & 0xFF) * ratio) >> 8;
  
  analogWrite(GPIO_LED_R, R);
  analogWrite(GPIO_LED_G, G);
  analogWrite(GPIO_LED_B, B);
}



    
void timerIsr()
{
  TimerTc3.detachInterrupt();
  g_bRotarySwIgnore = false;

  bool  bRotarySwState = digitalRead(GPIO_ROTARY_SW);
  if( bRotarySwState != g_bRotarySwState )
  {
    g_bRotarySwState = bRotarySwState;
    if( g_bRotarySwState )
    {
      g_nCtrlFine = !g_nCtrlFine;
    }
    else
    {
      // OnButtonUp
    }
  }
}

void OnRotarySwPush()
{
  if( !g_bRotarySwIgnore )
  {
    bool  bRotarySwState = digitalRead(GPIO_ROTARY_SW);
    TimerTc3.attachInterrupt(timerIsr);

    if( bRotarySwState != g_bRotarySwState )
    {
      g_bRotarySwState = bRotarySwState;
      if( g_bRotarySwState )
      {
        g_nCtrlFine = !g_nCtrlFine;
      }
      else
      {
        // OnButtonUp
      }
    }
  }
}


void  OnRotary( int dir )
{
  if( dir != 0 )
  {
    if( g_nCtrlFine )
    {
      g_nDacOut += 0 < dir ? 1 : -1;
    }
    else
    {
      int R12 = 110;
      int R13 = 390;
      int stp = 100 * R13 / (R12+R13); // 100mV

      g_nDacOut += 0 < dir ? stp : - stp;
    }

    g_nDacOut = 0 <= g_nDacOut ? g_nDacOut < 4096 ?  g_nDacOut : 4095 : 0;
    g_isUpdateDac = 1;
    UpdateLED(g_nDacOut);
  }
}

void OnRotaryA()
{
  g_nRotaryA  = digitalRead(GPIO_ROTARY_A) ? -1 : 1;
}

void OnRotaryB()
{
  OnRotary( g_nRotaryA );
  g_nRotaryA  = 0;
}

void OnAlert()
{
  g_nDacOut = 0;
  g_isUpdateDac = 1;
  analogWrite(GPIO_LED_R, 0);
  analogWrite(GPIO_LED_G, 0);
  analogWrite(GPIO_LED_B, 0);
}

void setup()
{
  // GPIO
  pinMode(GPIO_LED_R, OUTPUT);
  pinMode(GPIO_LED_G, OUTPUT);
  pinMode(GPIO_LED_B, OUTPUT);

  pinMode(GPIO_ROTARY_SW, INPUT);
  pinMode(GPIO_ROTARY_A, INPUT);
  pinMode(GPIO_ROTARY_B, INPUT);

  attachInterrupt(GPIO_ROTARY_SW, OnRotarySwPush, CHANGE );
  attachInterrupt(GPIO_ROTARY_A , OnRotaryA, CHANGE);
  attachInterrupt(GPIO_ROTARY_B , OnRotaryB, FALLING);
  Wire.begin();

  // LED
  g_nDacOut = 0;
  UpdateLED(g_nDacOut);

  // DAC
  g_iMCP4726.SetValue(g_nDacOut << 4);

  // INA226
  g_iPowerMon.SetSamplingDuration( 64 );

  // INA226 - Over current alert
  double alert_ampare = OVER_CURRENT_PROTECT;
  g_iPowerMon.SetAlertFunc( PMoni_INA226::ALERT_SHUNT_OVER_VOLT, (int)(alert_ampare / g_iPowerMon.GetAmpereOf1LSB()) );
  pinMode(GPIO_ALERT, INPUT);
  attachInterrupt(GPIO_ALERT , OnAlert, FALLING);
 
  // OLED
  g_iSSD1306.Init();
  g_iSSD1306.DispClear();
  g_iSSD1306.DispOn();

  TimerTc3.initialize( 50 * 1000);
}

void loop()
{
  if( g_isUpdateDac )
  {
    g_isUpdateDac = 0;
    g_iMCP4726.SetValue( g_nDacOut << 4 );
  }
  
  delay(100);
 
  double  V = g_iPowerMon.GetV();
  double  A = g_iPowerMon.GetA();

  // Console
  {
    char    szBuf[64];
    char    szV[32];
    char    szA[32];
    dtostrf( V, 0, 6, szV );
    dtostrf( A, 0, 6, szA );
    sprintf( szBuf, "%4d, %s, %s", g_nDacOut, szV, szA );
    Serial.println( szBuf );
  }

  // OLED
  {
    uint8_t image[128*64];
    char    szBuf[64];
    int     w,h;

    memset( image, 0, sizeof(image) );
    
    // Draw Voltage
    {
      dtostrf( V, 0, 3, szBuf );
      BitmapFont_DrawText( g_tBitmapFont48, image, 128, 128, 64, 0, 0, szBuf );
  
      BitmapFont_CalcRect( g_tBitmapFont24, "v", w, h );
      BitmapFont_DrawText( g_tBitmapFont24, image, 128, 128, 64, 128 - w, 18, "v" );
     }
  
    if( g_nCtrlFine )
    {
      BitmapFont_DrawText( g_tBitmapFont16, image, 128, 128, 64, 0, 48, "FINE" );
    }
  
    // Draw Ampare
    {
      int left = 128; 
      sprintf( szBuf, "A" );
      BitmapFont_CalcRect( g_tBitmapFont16, szBuf, w, h );
      left -= w;
      BitmapFont_DrawText( g_tBitmapFont16, image, 128, 128, 64, left, 46, szBuf );
      left -= 2; 
  
      dtostrf( A, 0, 3, szBuf );
  
      BitmapFont_CalcRect( g_tBitmapFont24, szBuf, w, h );
      left -= w;
      BitmapFont_DrawText( g_tBitmapFont24, image, 128, 128, 64, left, 40, szBuf );
    }
    
    g_iSSD1306.WriteImageGRAY( 0, 0, image, 128, 128, 64 );
  }
}
