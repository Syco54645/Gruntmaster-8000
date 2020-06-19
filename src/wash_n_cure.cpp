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

#define START_PIN 12
#define STOP_PIN  11
#define MODE_PIN  10

enum State
{ IDLE,
  WASHING,
  CURING
};

unsigned int operatingMode = IDLE;
unsigned int selectedMode;
unsigned int oldSelectedMode;
unsigned int selectedDuration = TIME0;  //hardcoded for now. remove when switch is added

//time slicing
unsigned long interval = 1000; // tick at 1 second
unsigned long startMillis;
unsigned long currentMillis;

void pre(void);
void printModeName();
void updateDisplay(unsigned long time);
void preformWash();
void preformCure();

void setup(void)
{
  u8x8.begin();
  #if defined(FLIP_SCREEN)
    u8x8.setFlipMode(1);
  #endif
  pre();
  startMillis = millis(); //initilize the start time
  pinMode(START_PIN, INPUT);
  pinMode(STOP_PIN, INPUT);
  pinMode(MODE_PIN, INPUT);
  oldSelectedMode = IDLE; // not really a mode that can be selected but serves its purpose
}

void pre(void)
{
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
  u8x8.clear();

  u8x8.inverse();
  u8x8.print("  Wash n' Cure  ");
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.noInverse();
  printModeName();
}

void printModeName() {
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.setCursor(0,1);
  switch (selectedMode) {
    case WASHING:
      u8x8.drawString(0, 1, "Washing");
      break;
    case CURING:
      u8x8.drawString(0, 1, "Curing");
      break;
    default:
      u8x8.drawString(0, 1, "Washing");
      break;
  }
}

void updateDisplay(unsigned long time) {
  u8x8.setFont(u8x8_font_inb33_3x6_n);
  u8x8.setCursor(0, 2);
  u8x8.print(time);
}

void preformWash() {
  pre();
  startMillis = millis(); // we need to know when we started the cycle
  int runningDuration = selectedDuration;
  updateDisplay(runningDuration); // just to get the first time showing
  while (1) {
    currentMillis = millis();
    if(currentMillis - startMillis > interval) {
      startMillis = millis();
      runningDuration--;
      updateDisplay(runningDuration);
    }
    if (runningDuration <= 0) {
      pre();
      break;
    }
    if (digitalRead(STOP_PIN) == HIGH) {
      pre();
      break;
    }
  }
}

void preformCure() {

}

void loop(void)
{
  selectedMode = digitalRead(MODE_PIN) == HIGH ? WASHING : CURING;
  #ifndef DEBUG
    if (operatingMode == WASHING) {
      preformWash();
    } else if (operatingMode == CURING) {
      preformCure();
    } else {
      u8x8.setFont(u8x8_font_chroma48medium8_r);
      u8x8.setCursor(0,1);
      u8x8.draw2x2String(0, 5, "Idle");
    }
  #endif

  if (selectedMode != oldSelectedMode) {
    oldSelectedMode = selectedMode;
    //pre();
    printModeName();
  }

  if (digitalRead(START_PIN) == HIGH) {
    operatingMode = selectedMode;
  } else {
    operatingMode = IDLE;
  }

  #if defined(DEBUG)
    u8x8.setFont(u8x8_font_inb33_3x6_n);
    u8x8.setCursor(0, 2);
    u8x8.clear();
    if (digitalRead(STOP_PIN) == HIGH) {
      u8x8.print(123);
    } else if (digitalRead(START_PIN) == HIGH) {
      u8x8.print(456);
    } else if (digitalRead(MODE_PIN) == HIGH){
      u8x8.print(100);
    } else {
      u8x8.print(789);
    }
  #endif
}