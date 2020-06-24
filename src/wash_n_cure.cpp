#include <Arduino.h>
#include <U8x8lib.h>
#include <configuration.h>
#include <AccelStepper.h>
#include <TimerOne.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);

//https://tronixstuff.com/2019/08/29/ssd1306-arduino-tutorial/

#define START_PIN   12
#define STOP_PIN    11
#define MODE_PIN    10
#define UV_LED_PIN   9
#define DIR_PIN      2
#define STEP_PIN     3
#define DRV_ENABLE  A3

#define UV_ON       255
#define UV_OFF        0
#define MOTOR_IF_TYPE 1
#define INT_TIMER    50

AccelStepper myStepper(MOTOR_IF_TYPE, STEP_PIN, DIR_PIN);

enum State
{
  IDLE,
  WASHING,
  WASHING_REV,
  WASHING_DOWN,
  CURING,
  COMPLETE,
  HALTED
};

int operatingMode = IDLE;
unsigned int selectedMode;
unsigned int oldSelectedMode;
unsigned int selectedDuration = TIME0;  //hardcoded for now. remove when switch is added

//time slicing
unsigned long interval = 1000; // tick at 1 second
unsigned long startMillis;
unsigned long currentMillis;
long int pos = 1000000;

void pre(void);
void printModeName();
void updateCountdownDisplay(unsigned long time);
void performWash();
void performCure();
void spinDown(bool disableStepper = true);
void changeStepperDir();
void setupStepper();
void stepperIsr();

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
  pinMode(UV_LED_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DRV_ENABLE, OUTPUT);
  digitalWrite(DRV_ENABLE, HIGH); // disable the a4988 driver

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
  switch (selectedMode) {
    case WASHING:
      u8x8.draw2x2String(0, 1, "Washing");
      break;
    case CURING:
      u8x8.draw2x2String(0, 1, "Curing "); // space after needed to clear the display
      break;
    default:
      u8x8.draw2x2String(0, 1, "Washing");
      break;
  }
}

void updateCountdownDisplay(unsigned long time) {
  u8x8.setFont(u8x8_font_profont29_2x3_n);
  u8x8.setCursor(0, 5);
  u8x8.print(time);
  u8x8.print("      "); // clears extra numbers from the display
}

void stepperIsr() {
  myStepper.run();
}

void performWash() {
  //pre();
  startMillis = millis(); // we need to know when we started the cycle
  int runningDuration = selectedDuration;
  long halfDuration = selectedDuration / 2;
  updateCountdownDisplay(runningDuration); // just to get the first time showing

  Timer1.initialize(INT_TIMER);// setup interrupt for the stepper
  Timer1.attachInterrupt(stepperIsr);

  while (1) {
    if (myStepper.distanceToGo() == 0) {
      myStepper.moveTo(pos);
    }

    currentMillis = millis();
    if(currentMillis - startMillis > interval) {
      startMillis = millis();
      runningDuration--;
      updateCountdownDisplay(runningDuration);
    }
    if (runningDuration <= halfDuration and pos > 0) {
      operatingMode = WASHING_REV;
      changeStepperDir();
    }
    if (runningDuration <= 0) {
      //pre();
      spinDown();
      operatingMode = COMPLETE;
      break;
    }
    if (digitalRead(STOP_PIN) == HIGH) {
      //pre();
      spinDown();
      operatingMode = HALTED;
      break;
    }
  }
}

void performCure() {
  //pre();
  startMillis = millis(); // we need to know when we started the cycle
  int runningDuration = selectedDuration;
  updateCountdownDisplay(runningDuration); // just to get the first time showing
  analogWrite(UV_LED_PIN, UV_ON);
  while (1) {
    currentMillis = millis();
    if(currentMillis - startMillis > interval) {
      startMillis = millis();
      runningDuration--;
      updateCountdownDisplay(runningDuration);
    }
    if (runningDuration <= 0) {
      //pre();
      operatingMode = COMPLETE;
      analogWrite(UV_LED_PIN, UV_OFF);
      break;
    }
    if (digitalRead(STOP_PIN) == HIGH) {
      //pre();
      operatingMode = HALTED;
      analogWrite(UV_LED_PIN, UV_OFF);
      break;
    }
  }
}

void changeStepperDir() {
  spinDown(false);
  Timer1.attachInterrupt(stepperIsr);
  pos = pos * -1;
}

void setupStepper(int maxSpeed, int acceleration) {
  digitalWrite(DRV_ENABLE, LOW); // enable the a4988 stepper driver
  delay(1); // needs a delay of 1 ms before we can step the motor
  pos = abs(pos); // since we change direction by pos * -1 just take the abs so it is always going the same direction at start
  myStepper.setMaxSpeed(maxSpeed);
  myStepper.setAcceleration(acceleration);
}

void spinDown(bool disableStepper = true) {
  Timer1.detachInterrupt();
  myStepper.stop();
  while(myStepper.speed() != 0) {
    myStepper.runToPosition();
  }
  if (disableStepper) {
    digitalWrite(DRV_ENABLE, HIGH); // disable the a4988 stepper drive
  }
}

void loop(void)
{
  selectedMode = digitalRead(MODE_PIN) == HIGH ? WASHING : CURING;
  #ifndef DEBUG
    u8x8.setFont(u8x8_font_profont29_2x3_r);
    switch (operatingMode) {
      case WASHING:
        setupStepper(WASH_MAX_SPEED, WASH_ACCELERATION);
        performWash();
        break;
      case CURING:
        performCure();
        break;
      case COMPLETE:
        u8x8.drawString(0, 5, "Complete");
        break;
      case HALTED:
        u8x8.drawString(0, 5, "Halted");
        break;
      default:
        u8x8.drawString(0, 5, "Idle");
        break;
    }
  #endif

  if (selectedMode != oldSelectedMode) {
    oldSelectedMode = selectedMode;
    printModeName();
  }

  if (digitalRead(START_PIN) == HIGH) {
    operatingMode = selectedMode;
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