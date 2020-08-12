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

#define START_PIN     13
#define STOP_PIN      11
#define MODE_PIN      10
#define WASH_MODE_LED  4
#define CURE_MODE_LED  5
#define UV_LED_PIN     9
#define DIR_PIN        2
#define STEP_PIN       3
#define DRV_ENABLE    A3
#define SPEAKER       12

#define SAFETY_PIN    A6
#define OVERRIDE_PIN  A7

#define TIMER0_LED    A2
#define TIMER1_LED    A1
#define TIMER2_LED    A0
#ifndef TIME_SEL_SWITCH
  #define TIME_SEL_PIN   6
#endif
#if defined(TIME_SEL_SWITCH)
  #define TIME_SEL_PIN0  6
  #define TIME_SEL_PIN1  7
  #define TIME_SEL_PIN2  8
#endif

#define UV_ON        255
#define UV_OFF         0
#define MOTOR_IF_TYPE  1
#define INT_TIMER     50

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
#ifndef TIME_SEL_SWITCH
  unsigned int selectedDuration = DEFAULT_TIME;
#endif
#if defined(TIME_SEL_SWITCH)
  unsigned int selectedDuration = TIME0;
#endif

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
void changeDuration();
bool isSafetyEngaged();
bool isSafetyOverrideEngaged();

void setup(void) {
  u8x8.begin();
  #if defined(FLIP_SCREEN)
    u8x8.setFlipMode(1);
  #endif
  pre();
  startMillis = millis(); //initilize the start time
  pinMode(START_PIN, INPUT);
  pinMode(STOP_PIN, INPUT);
  pinMode(MODE_PIN, INPUT);
  pinMode(WASH_MODE_LED, OUTPUT);
  pinMode(CURE_MODE_LED, OUTPUT);
  pinMode(UV_LED_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DRV_ENABLE, OUTPUT);
  pinMode(TIMER0_LED, OUTPUT);
  pinMode(TIMER1_LED, OUTPUT);
  pinMode(TIMER2_LED, OUTPUT);
  pinMode(SPEAKER, OUTPUT);
  digitalWrite(DRV_ENABLE, HIGH); // disable the a4988 driver

  digitalWrite(WASH_MODE_LED, LOW);
  digitalWrite(CURE_MODE_LED, LOW);

  tone(SPEAKER, 3000);
  delay(1000);
  noTone(SPEAKER);

  oldSelectedMode = IDLE; // not really a mode that can be selected but serves its purpose
}

void pre(void) {
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.clear();

  u8x8.inverse();
  u8x8.print("Gruntmaster 8000");
  //u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.noInverse();
  printModeName();
}

void printModeName() {
  //u8x8.setFont(u8x8_font_chroma48medium8_r);
  switch (selectedMode) {
    case WASHING:
      u8x8.draw2x2String(0, 1, "Washing");
      digitalWrite(WASH_MODE_LED, LOW);
      digitalWrite(CURE_MODE_LED, HIGH);
      break;
    case CURING:
      u8x8.draw2x2String(0, 1, "Curing "); // space after needed to clear the display
      digitalWrite(WASH_MODE_LED, HIGH);
      digitalWrite(CURE_MODE_LED, LOW);
      break;
    default:
      u8x8.draw2x2String(0, 1, "Washing");
      digitalWrite(WASH_MODE_LED, LOW);
      digitalWrite(CURE_MODE_LED, HIGH);
      break;
  }
}

void updateCountdownDisplay(unsigned long time) {
  // string is expensive but i see no other way around this. it is cheaper than including another font and using print
  //u8x8.setFont(u8x8_font_chroma48medium8_r);
  String strTime = String(time);
  while (strTime.length() < 8) {
    strTime = strTime + ' ';
  }
  u8x8.draw2x2String(0, 5, strTime.c_str());
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
    if(currentMillis - startMillis > interval && myStepper.speed() == WASH_MAX_SPEED) {
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
    if (digitalRead(STOP_PIN) == HIGH || (!isSafetyEngaged() && !isSafetyOverrideEngaged())) {
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
    if (runningDuration <= 0) {
      //pre();
      operatingMode = COMPLETE;
      analogWrite(UV_LED_PIN, UV_OFF);
      spinDown();
      break;
    }
    if (digitalRead(STOP_PIN) == HIGH || !isSafetyEngaged()) {
      //pre();
      operatingMode = HALTED;
      analogWrite(UV_LED_PIN, UV_OFF);
      spinDown();
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

void changeDuration() {
  switch (selectedDuration) {
    case TIME0:
      selectedDuration = TIME1;
      break;
    case TIME1:
      selectedDuration = TIME2;
      break;
    case TIME2:
      selectedDuration = TIME0;
      break;
    default:
      selectedDuration = TIME1;
      break;
  }
}

bool isSafetyEngaged() {
  #ifndef DISABLE_SAFETY
    if (analogRead(SAFETY_PIN) < 512) {
      return false;
    }
  #endif
  return true;
}

bool isSafetyOverrideEngaged() {
  #ifndef FAKE_SAFETY_OVERRIDE
    if (analogRead(OVERRIDE_PIN) < 512) {
      return false;
    }
  #endif
  return true;
}

void loop(void) {
  #ifndef DEBUG
    selectedMode = digitalRead(MODE_PIN) == HIGH ? WASHING : CURING;
    //u8x8.setFont(u8x8_font_chroma48medium8_r);
    switch (operatingMode) {
      case WASHING:
        if (isSafetyEngaged() || (!isSafetyEngaged() && isSafetyOverrideEngaged())) {
          setupStepper(WASH_MAX_SPEED, WASH_ACCELERATION);
          performWash();
        } else {
          // error beep
          operatingMode = IDLE;
        }
        break;
      case CURING:
        if (isSafetyEngaged()) {
          setupStepper(CURE_MAX_SPEED, CURE_ACCELERATION);
          performCure();
        } else {
          operatingMode = IDLE;
        }
        break;
      case COMPLETE:
        #if defined(DISPLAY_SELECTED_TIME)
          updateCountdownDisplay(selectedDuration);
        #else
          u8x8.draw2x2String(0, 5, "Complete");
        #endif
        break;
      case HALTED:
        #if defined(DISPLAY_SELECTED_TIME)
          updateCountdownDisplay(selectedDuration);
        #else
          u8x8.draw2x2String(0, 5, "Halted");
        #endif
        break;
      default:
        #if defined(DISPLAY_SELECTED_TIME)
          updateCountdownDisplay(selectedDuration);
        #else
          u8x8.draw2x2String(0, 5, "Idle");
        #endif
        break;
    }

    switch (selectedDuration) {
      case TIME0:
        digitalWrite(TIMER0_LED, HIGH);
        digitalWrite(TIMER1_LED, LOW);
        digitalWrite(TIMER2_LED, LOW);
        break;
      case TIME1:
        digitalWrite(TIMER1_LED, HIGH);
        digitalWrite(TIMER0_LED, LOW);
        digitalWrite(TIMER2_LED, LOW);
        break;
      case TIME2:
        digitalWrite(TIMER2_LED, HIGH);
        digitalWrite(TIMER0_LED, LOW);
        digitalWrite(TIMER1_LED, LOW);
        break;
    }

    if (digitalRead(TIME_SEL_PIN) == HIGH) {
      changeDuration();
      while (digitalRead(TIME_SEL_PIN) == HIGH) {} // simple trap to avoid needing debounce
    }

    if (selectedMode != oldSelectedMode) {
      oldSelectedMode = selectedMode;
      printModeName();
    }

    if (digitalRead(START_PIN) == HIGH) {
      operatingMode = selectedMode;
    }
  #endif


  #if defined(DEBUG)
    u8x8.setFont(u8x8_font_inb33_3x6_n);
    //u8x8.setCursor(0, 0);

    u8x8.setFont(u8x8_font_chroma48medium8_r);
    //u8x8.clear();

    //u8x8.inverse();
    //u8x8.print("Gruntmaster 8000");

    u8x8.setCursor(2, 0);
    u8x8.print("              ");
    if (digitalRead(START_PIN) == HIGH) {
      u8x8.print("START_PIN: HI");
    } else {
      u8x8.print("START_PIN: LO");
    }

    u8x8.setCursor(2, 1);
    u8x8.print("              ");
    if (digitalRead(STOP_PIN) == HIGH) {
      u8x8.print("STOP_PIN: HI");
    } else {
      u8x8.print("STOP_PIN: LO");
    }

    u8x8.setCursor(2, 2);
    u8x8.print("              ");
    if (digitalRead(MODE_PIN) == HIGH) {
      u8x8.print("MODE_PIN: HI");
    } else {
      u8x8.print("MODE_PIN: LO");
    }

    u8x8.setCursor(2, 3);
    u8x8.print("              ");
    if (digitalRead(SAFETY_PIN) == HIGH) {
      u8x8.print("SAFETY_PIN: HI");
    } else {
      u8x8.print("SAFETY_PIN: LO");
    }

    u8x8.setCursor(2, 4);
    u8x8.print("              ");
    if (digitalRead(OVERRIDE_PIN) == HIGH) {
      u8x8.print("OVERRIDE_PIN: HI");
    } else {
      u8x8.print("OVERRIDE_PIN: LO");
    }

    #ifndef TIME_SEL_SWITCH
      u8x8.setCursor(2, 5);
      u8x8.print("              ");
      if (digitalRead(TIME_SEL_PIN) == HIGH) {
        u8x8.print("TIME_SEL_PIN: HI");
      } else {
        u8x8.print("TIME_SEL_PIN: LO");
      }
    #endif

    #if defined(TIME_SEL_SWITCH)
      u8x8.setCursor(2, 5);
      u8x8.print("              ");
      if (digitalRead(TIME_SEL_PIN0) == HIGH) {
        u8x8.print("TIME_SEL_PIN0:HI");
      } else {
        u8x8.print("TIME_SEL_PIN0:LO");
      }

      u8x8.setCursor(2, 6);
      u8x8.print("              ");
      if (digitalRead(TIME_SEL_PIN1) == HIGH) {
        u8x8.print("TIME_SEL_PIN1:HI");
      } else {
        u8x8.print("TIME_SEL_PIN1:LO");
      }

      u8x8.setCursor(2, 7);
      u8x8.print("              ");
      if (digitalRead(TIME_SEL_PIN2) == HIGH) {
        u8x8.print("TIME_SEL_PIN2:HI");
      } else {
        u8x8.print("TIME_SEL_PIN2:LO");
      }
    #endif
  #endif
}