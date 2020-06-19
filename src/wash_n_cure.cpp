#include <Arduino.h>
#include <U8x8lib.h>
#include <configuration.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);

//https://tronixstuff.com/2019/08/29/ssd1306-arduino-tutorial/


enum State
{ IDLE,
  WASHING,
  CURING
};

unsigned int mode = WASHING; //hardcoded for now. remove when switch is added
unsigned int selectedDuration = TIME0;  //hardcoded for now. remove when switch is added

//time slicing
unsigned long interval = 1000; // tick at 1 second
unsigned long startMillis;
unsigned long currentMillis;


void setup(void)
{
  u8x8.begin();
  #if defined(FLIP_SCREEN)
    u8x8.setFlipMode(1);
  #endif
  startMillis = millis(); //initilize the start time
}

void pre(void)
{
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
  u8x8.clear();

  u8x8.inverse();
  u8x8.print("  Wash n' Cure  ");
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.noInverse();
  u8x8.setCursor(0,1);
}

void printModeName() {
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.drawString(0, 1, "Washing");
  switch (mode) {
    case 0:
      u8x8.drawString(0, 1, "Washing");
      break;
    case 1:
      u8x8.drawString(0, 1, "Curing");
      break;
    default:
      u8x8.drawString(0, 1, "Washing");
      break;
  }
}

void updateDisplay(unsigned long time) {
  printModeName();
  u8x8.setFont(u8x8_font_inb33_3x6_n);
  u8x8.setCursor(0, 2);
  u8x8.print(selectedDuration - (time / 1000));
}

void preformWash() {
  pre();
  startMillis = millis(); // we need to know when we started the cycle
  unsigned int runningDuration = 0;
  updateDisplay(0); // just to get the first time showing
  while (1) {
    currentMillis = millis();
    if(currentMillis - startMillis > interval) {
      startMillis = millis();
      runningDuration++;
      updateDisplay(currentMillis);
    }
    if (runningDuration >= selectedDuration) {
      break;
    }
  }
}

void preformCure() {

}

void loop(void)
{
  if (mode == WASHING) {
    preformWash();
  } else if (mode == CURING) {
    preformCure();
  }
  mode = IDLE;
}