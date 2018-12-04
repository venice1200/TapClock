/*
    WordClock for WatchX

    Based on TapClock 0.5.7 / PushClock 0.5.7

    Libraries:
    -WatchX libs by ArgeX                   see http://watchx.io/downloads_en.html
    -MPU6050 Lib from Korneliusz Jarzebski  see https://github.com/jarzebski/Arduino-MPU6050
    -BasicWatch 1 from kghunt/DntPMme       see https://github.com/kghunt/Basic_Watch
    -I2CDEVLIB                              see https://github.com/jrowberg/i2cdevlib 
    -Angle calculation                      see https://electronics.stackexchange.com/questions/142037/calculating-angles-from-mpu6050
    -Set Time at "Setup"                    see http://www.l8ter.com/?p=417
    -Nick Gammon Microprocessors Infos      see http://www.gammon.com.au/power and http://www.gammon.com.au/interrupts
    -Bounce2                                see https://github.com/thomasfredericks/Bounce2
    -Pin Change Interrupt (PCINT)           see https://github.com/NicoHood/PinChangeInterrupt

    v0.1    Proof of concept
    
    v0.2    
    Added:  Wake Up from Deep Sleep with B3
            PCINT handling needed
    Added:  Send MPU to Sleep (let's sse how long the battery runs now)

    v0.2.1
    Bugfix: hrs >12 not shown 
    Added:  Indikator AM/PM
    

    State:
    0:     Start/Display On
    10-21: Intro Steps
    50-53: Steps before clock is shown, Position Detection
    60-61: Show Clock
    90:    Prepare System before Sleep, Oled/Led
    91:    Sleep and Wakeup / Power Modes / Interrupt / Sleep
    100:   Setup Time
	  120:   Stats

    ULB = Button Pin 8  = links oben / upper left
    URB = Button Pin 11 = rechts oben / upper right
    LRB = Button Pin 10 = rechts unten / lower right
*/

#include <avr/sleep.h>
#include <avr/power.h>
#include <Wire.h>
#include <MPU6050.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiSpi.h"
#include "RTClib.h"
#include <SparkFun_MAG3110.h>
#include <Bounce2.h>
#include "PinChangeInterrupt.h"

SSD1306AsciiSpi oled;
MPU6050         mpu(0x69);  //AD0 = 1
RTC_DS3231      rtc;
MAG3110         mag = MAG3110();

// Instances of Bounce objects
Bounce b1debouncer = Bounce(); 
Bounce b2debouncer = Bounce(); 
Bounce b3debouncer = Bounce(); 

#define VERSION "v0.2.1"
#define PRGNAME "WordClock"
//Debug or not Debug...
#define DEBUG 0
//#define DEBUG 1

// Oled use hardware SPI
#define OLED_DC     A3
#define OLED_CS     A5
#define OLED_RST    A4
#define BAT_PIN     A11
#define BAT_EN      4
#define CRG_STATE   5
#define BUZZER      9
#define INT6_PIN    7
#define LEDL        13
#define LEDR        6
#define BUTTON1     8 
#define BUTTON2     11
#define BUTTON3     10
#define BLE_CS      A2
#define BLE_IRQ     0
#define BLE_RST     A1
#define COMP_OFFSET 8     //Compile and Transfer Time Offset
#define DebounceTime 25 // Button Debounce Time

// Using (lot's) defines for the output (setCursor/print) to the OLED
#define xoffs 24 // X Offset
#define ITIS    oled.setCursor(xoffs+0,0);oled.print("IT");oled.setCursor(xoffs+18,0);oled.print("IS");
#define MTEN    oled.setCursor(xoffs+36,0);oled.print("TEN");
#define HALF    oled.setCursor(xoffs+54,0);oled.print("HALF");
#define QUARTER oled.setCursor(xoffs+0,1);oled.print("QUARTER");
#define TWENTY  oled.setCursor(xoffs+42,1);oled.print("TWENTY");
#define MFIVE   oled.setCursor(xoffs+0,2);oled.print("FIVE");
#define MINUTES oled.setCursor(xoffs+30,2);oled.print("MINUTES");
#define PAST    oled.setCursor(xoffs+0,3);oled.print("PAST");
#define TO      oled.setCursor(xoffs+24,3);oled.print("TO");
#define ONE     oled.setCursor(xoffs+42,3);oled.print("ONE");
#define TWO     oled.setCursor(xoffs+60,3);oled.print("TWO");
#define THREE   oled.setCursor(xoffs+0,4);oled.print("THREE");
#define FOUR    oled.setCursor(xoffs+30,4);oled.print("FOUR");
#define HFIVE   oled.setCursor(xoffs+54,4);oled.print("FIVE");
#define SIX     oled.setCursor(xoffs+0,5);oled.print("SIX");
#define SEVEN   oled.setCursor(xoffs+18,5);oled.print("SEVEN");
#define EIGHT   oled.setCursor(xoffs+48,5);oled.print("EIGHT");
#define NINE    oled.setCursor(xoffs+0,6);oled.print("NINE");
#define HTEN    oled.setCursor(xoffs+24,6);oled.print("TEN");
#define ELEVEN  oled.setCursor(xoffs+42,6);oled.print("ELEVEN");
#define TWELVE  oled.setCursor(xoffs+0,7);oled.print("TWELVE");
#define OCLOCK  oled.setCursor(xoffs+42,7);oled.print("OCLOCK");

#define L0      oled.setCursor(xoffs+0,0);oled.print("ITRISCTENHALF");
#define L1      oled.setCursor(xoffs+0,1);oled.print("QUARTERTWENTY");
#define L2      oled.setCursor(xoffs+0,2);oled.print("FIVECMINUTESH");
#define L3      oled.setCursor(xoffs+0,3);oled.print("PASTTOEONETWO");
#define L4      oled.setCursor(xoffs+0,4);oled.print("THREEFOURFIVE");
#define L5      oled.setCursor(xoffs+0,5);oled.print("SIXSEVENEIGHT");
#define L6      oled.setCursor(xoffs+0,6);oled.print("NINETENELEVEN");
#define L7      oled.setCursor(xoffs+0,7);oled.print("TWELVELOCLOCK");

volatile int State = 0;
volatile int prevState = 0;

//Display Variables
unsigned long previousclockmillis = 0;
const long clocktime = 5000;
bool ShowTime = false;
bool init_view;

//Date & Time
char datebuffer[10];
const char *months[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
int prev_min = 61, prev_sec = 61;  // 61 = to be sure the xxx_changed (see next line) booleans  will be true
bool min_changed = false, sec_changed = false; 
int  hour=0, minute=0, second=0;

//Voltage Calc Vars
float R1 = 10000;
float R2 = 10000;
float vDivider;

//USB Power Detdction
bool usbConnected = false;
bool charging = false;

//ChargeLED(R) Vars
long int chargeledmillis = 0;
const int chargeledinterval = 50;
const int chargeledmaxvalue = 255;
const int chargeledminvalue = 10;
int chargeledfadevalue = chargeledminvalue; // !!Important to initialize the startvalue with minvalue!!
int chargeledfadesteps = 5;

//250ms Blinker Vars
long int blink250millis = 0;
const int blink250interval = 250;
bool blink250 = false;

//500ms Blinker Vars
long int blink500millis = 0;
const int blink500interval = 500;
bool blink500 = false;

//Remove delays from Intro
long int waitmillis = 0;
int waittime = 0;

DateTime startTime;

int counterWakeUp = 0, counterShowTime = 0;

const char* menuLine[] ={"", "Year", "Month", "Day", "Hour", "Minute", "Save", "Exit"};
int menuLineValue[6];
const int menuValueStart = 1;
const int menuValueEnd = 5;
const int menuStart = 1;
const int menuEnd = 7;
const int menuValueYear = 1;
const int menuValueMonth = 2;
const int menuValueDay = 3;
const int menuValueHour = 4;
const int menuValueMinute = 5;
const int menuValueSave = 6;
const int menuValueExit = 7;
int menuIndex = 1;                // Start at 1 (Year)
bool menuEditMode  = false;
bool setupEditDone = false;
bool setupClockDone = false;

//--------------------------- Setup --------------------------------

void setup() 
{
  if (DEBUG) {
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo's serial, others continue immediately
  }

  // Set WatchX RTC time only if needed (Power off or uploading sketch with actual time but not on a normal reset)
  rtc.begin();
  // Variant 1
  //if (rtc.lostPower()) {

  // Variant 2
  startTime = rtc.now();
  DateTime compiledTime = DateTime(__DATE__, __TIME__);
  if (startTime.unixtime() < compiledTime.unixtime()) {   
    rtc.adjust(DateTime(__DATE__, __TIME__) + COMP_OFFSET);    // set Time, offset because of compile time
    startTime = rtc.now();                           // Re-read Start Time as it was changed
  }
  //rtc.adjust(DateTime(__DATE__, __TIME__) + COMP_OFFSET);    // Offset because of compile time offset

  // Init Display and Clear up
  oled.begin(&Adafruit128x64, OLED_CS, OLED_DC, OLED_RST);
  oled.setFont(System5x7);
  oled.clear();

  // MPU Init and settings
  mpu.initialize();
  mpu.setSleepEnabled(true);

  // Init Compass MAG3300 and send it directly into standby
  mag.initialize();
  mag.start();                  // Needed ??
  mag.enterStandby();

  // Setup Hardware
  // LEDs
  pinMode(LEDL, OUTPUT);
  pinMode(LEDR, OUTPUT);
  digitalWrite(LEDL, LOW);
  digitalWrite(LEDR, LOW);

  // Buttons
  pinMode(BUTTON1, INPUT_PULLUP);
  b1debouncer.attach(BUTTON1);
  b1debouncer.interval(DebounceTime); // interval in ms
  pinMode(BUTTON2, INPUT_PULLUP);
  b2debouncer.attach(BUTTON2);
  b2debouncer.interval(DebounceTime); // interval in ms
  pinMode(BUTTON3, INPUT_PULLUP);
  b3debouncer.attach(BUTTON3);
  b3debouncer.interval(DebounceTime); // interval in ms

  // Inputs
  pinMode(CRG_STATE, INPUT_PULLUP);

  //Interrupt on INT6/Pin7, attaching/detaching moved to State 7
  pinMode(INT6_PIN, INPUT_PULLUP);

  // For voltage calulation
  vDivider = (R2 / (R1 + R2));
  
  // Enable VBUS Pad for detecting USB Power
  USBCON|=(1<<OTGPADE); 

}

//--------------------------- Main Loop --------------------------------

void loop()
{
  // main loop variables
  int act_hr, act_min, act_sec, act_year, act_month, act_day;
  unsigned char bat_hdrs, bat_tens, bat_ones;
  unsigned long currentmillis = millis();

  // Battery
  float voltage;
  int percent;

  //MPU Values
  int16_t ax=0, ay=0, az=0;
  float arx=0, ary=0, arz=0;
  bool readclockpos = false;
  bool every10secs = false;
  bool waittimeover = false, waitclockpos = false;

  //Bouncer Values
  int b1value, b2value, b3value;
  bool b23changed;

  // Update the Bounce instances :
  b1debouncer.update();
  b2debouncer.update();
  b3debouncer.update();

  // Get the updated value :
  b1value = b1debouncer.read();
  b2value = b2debouncer.read();
  b3value = b3debouncer.read();

  //Button 2 or 3 pressed
  b23changed = b2debouncer.fell() || b3debouncer.fell();


  DateTime now = rtc.now();
  act_hr    = now.hour();
  act_min   = now.minute();
  act_sec   = now.second();
  act_year  = now.year();
  act_month = now.month();
  act_day   = now.day();

  // For the WordClock
  hour      = now.hour();
  minute    = now.minute();
  second    = now.second();
  if (hour > 12) hour -= 12;
  
  unsigned long runTime = now.unixtime() - startTime.unixtime();  // get runtime in seconds

  every10secs = ((act_sec % 10) == 0);
  
  //Secs or Mins changed?
  /*
  if (act_min != prev_min) {
    min_changed = true;
  }
  else {
    min_changed = false;
  }
  
  if (act_sec != prev_sec) {
    sec_changed = true;
  }
  else {
    sec_changed = false;
  }
  */

  // v2 Secs or Mins changed?
  min_changed = act_min != prev_min?true:false;
  sec_changed = act_sec != prev_sec?true:false;
  prev_min = act_min;
  prev_sec = act_sec;

  // LED Fader Counter
  if (currentmillis - chargeledmillis > chargeledinterval) {
    chargeledmillis = currentmillis;
    chargeledfadevalue = chargeledfadevalue + chargeledfadesteps;
    if ((chargeledfadevalue <= chargeledminvalue) || (chargeledfadevalue >= chargeledmaxvalue)) chargeledfadesteps = -chargeledfadesteps;
  }
  
  // 250ms Blinker
  if (currentmillis - blink250millis > blink250interval) {
    blink250millis = currentmillis;
    blink250 = !blink250;
  }
  
  // 500ms Blinker
  if (currentmillis - blink500millis > blink500interval) {
    blink500millis = currentmillis;
    blink500 = !blink500;
  }
  
  // USB connected? Variant 2 USB Power
  usbConnected = (USBSTA&(1<<VBUS));       

  // Generate charging bit & show it
  charging = usbConnected && !digitalRead(CRG_STATE);
  
  // Show charging as a glowing LEDR
  if (charging) {
    analogWrite(LEDR, chargeledfadevalue);  
  }
  else {
    analogWrite(LEDR, 0);
  }
  
  // waittimeover ??
  if (currentmillis - waitmillis >= waittime) {
    waittimeover = true; 
  }
  else {
    waittimeover = false;
  }

  // State changes
  // Intro steps, see states now
  // Jump from Intro to Clock, see states now
 
  // Jump to Power Display off
  if ((State == 61) && (currentmillis - previousclockmillis >= clocktime)) setState(90);
  
  // If we are on USB Power do not run the "Show Clock" Timer so update the "previous" with the actual millis eaach cycle
  if ((State == 61) && usbConnected) {
    previousclockmillis = currentmillis;
  }
  
  // Setup Tools Starting at 100
  // Setup Clock
  if ((State == 61) && b1debouncer.fell()) {
    setState(100);
  }

  if ((State == 100) && b1debouncer.fell() && getMenuFuncMode(menuIndex)) {
    setState(110);
  }

  if ((State == 61) && b2debouncer.fell()) {
    setState(120);
  }

  if ((State == 121) && b1debouncer.fell()) {
    setState(122);
  }

  // State Switcher
  switch (State) {
    case 0: 
      // Intro :-) Just for Fun
      oled.ssd1306WriteCmd(SSD1306_DISPLAYON);
      oled.clear();
      setState(10);
    break;

    case 10:
      oled.set1X();
      oled.setCursor(32,0);
      oled.print(F("Welcome to"));
      waitmillis = currentmillis;
      waittime = 2000;
      setState(11);
    break;
    
    case 11:
      // Wait for timer
      if (waittimeover) setState(12);
    break;
    
    case 12:
      oled.set2X();
      oled.setCursor(26,3);
      oled.print(F("WatchX"));
      waitmillis = currentmillis;
      waittime = 3000;
      setState(13);
    break;
    
    case 13:
      // Wait for timer
      if (waittimeover) setState(16);
    break;
    
    case 16:
      oled.clear();
      waitmillis = currentmillis;
      waittime = 1000;
      setState(17);
    break;
    
    case 17:
      // Wait for timer
      if (waittimeover) setState(18);
    break;
    
    case 18:    
      oled.set2X();
      oled.setCursor(12,1);
      oled.print(F(PRGNAME));
      oled.set1X();
      oled.setCursor(46,5);
      oled.print(F(VERSION));
      waitmillis = currentmillis;
      waittime = 1500;
      setState(19);
    break;
    
    case 19:
      // Wait for timer
      if (waittimeover) setState(20);
    break;
    
    case 20:
      oled.clear();
      waitmillis = currentmillis;
      waittime = 1000;
      setState(21);
    break;  
    
    case 21:
      // Wait for timer
      if (waittimeover) setState(50);
    break;
   
    case 50:
      counterWakeUp++;
      setState(60);
    break;

    case 60:
      // Prepare Display and Power On ADC
      // Set ShowTime Timer
      previousclockmillis =  millis();
      // Enable Display
      oled.ssd1306WriteCmd(SSD1306_DISPLAYON);
      oled.clear();
      // Enable ADC via PRR Register Bit 0 (PRADC)
      power_adc_enable();
      // Some Variables
      init_view = true;
      ShowTime = true;
      if (prevState == 52) counterShowTime++;          // +1 only if coming from State 52
      setState(61);
    break;

    case 61:
      //Show WordClock at Init and every minute
      if (min_changed || init_view) displaytime();     

       oled.setCursor(116,0);
       if (act_hr > 12) {
         oled.print(F("PM"));
       }
       else {
         oled.print(F("AM"));
       }
      
      // Show Battery only at init or every 10secs with usb connected
      if (init_view || (sec_changed && every10secs && usbConnected)) {
        // Get Battery Values
        digitalWrite(BAT_EN, HIGH);
        delay(50);
        voltage = analogRead(BAT_PIN);
        voltage = (voltage / 1024) * 3.35;
        voltage = voltage / vDivider;
        delay(50);
        digitalWrite(BAT_EN, LOW);
        //Calculate Values
        percent = (voltage - 3.4) / 0.008;
        if (percent < 0){
          percent = 0;
        }
        if (percent > 100){
          percent = 100;
        }

        bat_hdrs = percent/100;
        //bat_tens = (percent%100)/10;
        bat_tens = percent%100/10;
        bat_ones = percent%10;
        
        // Write battery Value to Display
        //oled.setCursor(0,3);
        //oled.print(F("B"));          // "B"
        if (bat_hdrs > 0) {
          oled.setCursor(0,4);
          oled.print(bat_hdrs);        // hundred's
        }
        if (bat_tens > 0) {
          oled.setCursor(0,5);
          oled.print(bat_tens);               // tens
        }
        oled.setCursor(0,6);
        oled.print(bat_ones);         // ones
        oled.setCursor(0,7);
        oled.print(F("%"));
        
      } // endif Show Battery
      init_view = false;  //Init View done
    break;
    
    case 90:
      // Prepare System for sleep
      digitalWrite(LEDL, false);
      // Clear Display on switch off
      oled.clear();
      oled.ssd1306WriteCmd(SSD1306_DISPLAYOFF);
      ShowTime = false;
      // Next Step = Sleep
      setState(91);
    break;
    
    case 91:
      // Send System into sleeeeeeeeeep....
      // Disable ADC System via PRR Register Bit 0 (PRADC)
      power_adc_disable();
      
      // Setup SleepMode , see https://www.gammon.com.au/forum/?id=11497
      //set_sleep_mode(SLEEP_MODE_STANDBY);      // Wakeup Works with PCINT
      set_sleep_mode (SLEEP_MODE_PWR_DOWN);      // Wakeup Works from v0.5.1 also with PCINT
	  
	    // Enable Sleep Mode by setting SMCR Register Bit 0 and send the CPU to sleep
      sleep_enable();
      // Enable 32u4 MPU interrupt
      attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(BUTTON3), doInt, FALLING);
      // Go sleeping....
      sleep_cpu();

      // ***************************************************
      // ******* The Code starts here after wakeup *********
      // ***************************************************

      // Disable sleep
      sleep_disable();
      // Disable 32u4 MPU interrupt
      detachPinChangeInterrupt(digitalPinToPinChangeInterrupt(BUTTON3));
    break;
    
    // Setup Clock
    case 100:
      // Edit Mode on/off
      if ( b1debouncer.fell() && getMenuEditMode(menuIndex) && setupEditDone ) menuEditMode = !menuEditMode;
 
      // Show Active Menu Entry Inverted
      if ( b1debouncer.fell() && menuEditMode ) {
        oled.setInvertMode(true);
        oled.setCursor(6,menuIndex);
        oled.print(menuLine[menuIndex]);
        oled.setInvertMode(false);
      }
      // Show Active Entry Not Inverted
      if ( b1debouncer.fell() && !menuEditMode ) {
        oled.setCursor(6,menuIndex);
        oled.print(menuLine[menuIndex]);
      }

      // MenuIndex +/-
      if ( b2debouncer.fell() && !menuEditMode ) menuIndex--;
      if ( b3debouncer.fell() && !menuEditMode ) menuIndex++;
      
      // MenuIndex Bounds 
      if (menuIndex > menuEnd) menuIndex = menuStart;
      if (menuIndex < menuStart) menuIndex = menuEnd;

      // MenuLineValue +/-
      if (getMenuEditMode(menuIndex)) {
        if ( b2debouncer.fell() && menuEditMode ) menuLineValue[menuIndex]++;
        if ( b3debouncer.fell() && menuEditMode ) menuLineValue[menuIndex]--;

        switch(menuIndex) {
          case menuValueYear:
            if (menuLineValue[menuIndex] < 2018) menuLineValue[menuIndex] = 2030; 
            if (menuLineValue[menuIndex] > 2030) menuLineValue[menuIndex] = 2018; 
          break;
          case menuValueMonth:
            if (menuLineValue[menuIndex] < 0) menuLineValue[menuIndex] = 12; 
            if (menuLineValue[menuIndex] > 12) menuLineValue[menuIndex] = 0; 
          break;
          case menuValueDay:
            if (menuLineValue[menuIndex] < 0) menuLineValue[menuIndex] = 31; 
            if (menuLineValue[menuIndex] > 31) menuLineValue[menuIndex] = 0; 
          break;
          case menuValueHour:
            if (menuLineValue[menuIndex] < 0) menuLineValue[menuIndex] = 23; 
            if (menuLineValue[menuIndex] > 23) menuLineValue[menuIndex] = 0; 
          break;
          case menuValueMinute:
            if (menuLineValue[menuIndex] < 0) menuLineValue[menuIndex] = 59; 
            if (menuLineValue[menuIndex] > 59) menuLineValue[menuIndex] = 0; 
          break;
        } // end switch menuIndex
        
        if ( b23changed && menuEditMode ) {
          oled.setCursor(80,menuIndex);
          sprintf(datebuffer, "%02u", menuLineValue[menuIndex]);
          oled.print(datebuffer);
        }
      }

      if (!setupEditDone) {
        oled.clear();      
        //Copy Values into Array
        menuLineValue[1] = act_year;
        menuLineValue[2] = act_month;
        menuLineValue[3] = act_day;
        menuLineValue[4] = act_hr;
        menuLineValue[5] = act_min;
    
        oled.setCursor(30,0);
        oled.print(F("Time Setup"));
    
        for (int i = menuStart; i <= menuEnd; i++) {
          oled.setCursor(6,i);
          oled.print(menuLine[i]);
          if (getMenuEditMode(i)) {
            oled.setCursor(80,i);
            sprintf(datebuffer, "%02u", menuLineValue[i]);
            oled.print(datebuffer);
          }
        }  // endfor i
      } //endif setupEditDone

      if ((b23changed  && !menuEditMode) || !setupEditDone) {
        for (int i=menuStart; i<=menuEnd; i++) 
        {
          oled.setCursor(0,i);
          if (i == menuIndex) {
            oled.print(F(">"));
          }
          else {
            oled.print(F(" "));  
          }  // endelse
        }  // endfor i
      }  // endif b23changed
      setupEditDone = true;
    break;

    case 110:
      setupEditDone = false;
      oled.clear();
      if (menuIndex == menuValueSave) 
      {
        oled.print(F("Save'd...")); // Save Time to RTC
        rtc.adjust(DateTime(menuLineValue[1], menuLineValue[2], menuLineValue[3], menuLineValue[4], menuLineValue[5],0));
      }
      if (menuIndex == menuValueExit) 
      {
        oled.print(F("Exit'd...")); // Just exited
      }
      menuIndex = 1;  // Back to Menu start
      waitmillis = currentmillis;
      waittime = 1500;
      setState(111);
    break;

    case 111:
      if (waittimeover) setState(60);
    break;

    case 120:
      oled.clear();
      // Header
      oled.setCursor(44,0);
      oled.print(F(">Stats<"));

      oled.setCursor(6,2);
      oled.print(F("Uptime:"));
      
      oled.setCursor(6,3);
      oled.print(F("Wakes: "));
      oled.print(counterWakeUp);
      oled.setCursor(72,3);
      oled.print(F("Views: "));
      oled.print(counterShowTime);

      setState(121);
    break;

    case 121:
      // Show values updated each minute
      if (sec_changed) {
        //Show Uptime
        sprintf(datebuffer, "%03u:%02u", int(runTime/3600), int((runTime/60)%60));
        //sprintf(datebuffer, "%06u", runTime);
        oled.setCursor(72,2);
        oled.print(datebuffer);
      } //endif min_changed
    break;

    case 122:
      setState(60);
    break;
  }
}

//--------------- Functions ----------------------------------

// Interrupt Routine
void doInt()
{
  // Cancel sleep as a precaution here as well
  // Normal the code starts in state 7 with "sleep_disable()"
  sleep_disable();
  // Disable MPU interrupt again
  detachPinChangeInterrupt(digitalPinToPinChangeInterrupt(BUTTON3));
  // Let's show the time again
  setState(50);
}

bool getMenuEditMode(int value) 
{
  if ((value >= menuValueStart) && (value <= menuValueEnd)) return true; 
  else return false;
}

bool getMenuFuncMode(int value) 
{
  if ((value == menuValueSave) || (value == menuValueExit)) return true; 
  else return false;
}

void setState(int value)
{
  prevState = State;
  State = value;
}

/*
void setwaittime(int value)
{
  waitmillis = currentmillis;
  waittime = value;
}
*/

void displaytime(void){
  // Start by setting the display to a known state

  L0;
  L1;
  L2;
  L3;
  L4;
  L5;
  L6;
  L7;

  oled.setInvertMode(true);  
  ITIS;
  if (DEBUG) Serial.print("It Is ");
  
  // now we display the appropriate minute counter
  if ((minute>4) && (minute<10)) { 
    MFIVE; 
    MINUTES; 
    if (DEBUG) Serial.print("Five Minutes ");
  } 
  if ((minute>9) && (minute<15)) { 
    MTEN; 
    MINUTES; 
    if (DEBUG) Serial.print("Ten Minutes ");
  }
  if ((minute>14) && (minute<20)) {
    QUARTER; 
      if (DEBUG) Serial.print("Quarter ");
  }
  if ((minute>19) && (minute<25)) { 
    TWENTY; 
    MINUTES; 
    if (DEBUG) Serial.print("Twenty Minutes ");
  }
  if ((minute>24) && (minute<30)) { 
    TWENTY; 
    MFIVE; 
    MINUTES;
    if (DEBUG) Serial.print("Twenty Five Minutes ");
  }  
  if ((minute>29) && (minute<35)) {
    HALF;
    if (DEBUG) Serial.print("Half ");
  }
  if ((minute>34) && (minute<40)) { 
    TWENTY; 
    MFIVE; 
    MINUTES;
    if (DEBUG) Serial.print("Twenty Five Minutes ");
  }  
  if ((minute>39) && (minute<45)) { 
    TWENTY; 
    MINUTES; 
    if (DEBUG) Serial.print("Twenty Minutes ");
  }
  if ((minute>44) && (minute<50)) {
    QUARTER; 
    if (DEBUG) Serial.print("Quarter ");
  }
  if ((minute>49) && (minute<55)) { 
    MTEN; 
    MINUTES; 
    if (DEBUG) Serial.print("Ten Minutes ");
  } 
  if (minute>54) { 
    MFIVE; 
    MINUTES; 
    if (DEBUG) Serial.print("Five Minutes ");
  }

  if ((minute <5))
  {
    switch (hour) {
    case 1: 
      ONE; 
      if (DEBUG) Serial.print("One ");
      break;
    case 2: 
      TWO; 
      if (DEBUG) Serial.print("Two ");
      break;
    case 3: 
      THREE; 
      if (DEBUG) Serial.print("Three ");
      break;
    case 4: 
      FOUR; 
      if (DEBUG) Serial.print("Four ");
      break;
    case 5: 
      HFIVE; 
      if (DEBUG) Serial.print("Five ");
      break;
    case 6: 
      SIX; 
      if (DEBUG) Serial.print("Six ");
      break;
    case 7: 
      SEVEN; 
      if (DEBUG) Serial.print("Seven ");
      break;
    case 8: 
      EIGHT; 
      if (DEBUG) Serial.print("Eight ");
      break;
    case 9: 
      NINE; 
      if (DEBUG) Serial.print("Nine ");
      break;
    case 10: 
      HTEN; 
      if (DEBUG) Serial.print("Ten ");
      break;
    case 11: 
      ELEVEN; 
      if (DEBUG) Serial.print("Eleven ");
      break;
    case 12: 
      TWELVE; 
      if (DEBUG) Serial.print("Twelve ");
      break;
  }
  OCLOCK;
  if (DEBUG) Serial.println("O'Clock");
  }
  else
    if ((minute < 35) && (minute >4))
    {
      PAST;
      if (DEBUG) Serial.print("Past ");
      switch (hour) {
    case 1: 
      ONE; 
      if (DEBUG) Serial.println("One ");
      break;
    case 2: 
      TWO; 
      if (DEBUG) Serial.println("Two ");
      break;
    case 3: 
      THREE; 
      if (DEBUG) Serial.println("Three ");
      break;
    case 4: 
      FOUR; 
      if (DEBUG) Serial.println("Four ");
      break;
    case 5: 
      HFIVE; 
      if (DEBUG) Serial.println("Five ");
      break;
    case 6: 
      SIX; 
      if (DEBUG) Serial.println("Six ");
      break;
    case 7: 
      SEVEN; 
      if (DEBUG) Serial.println("Seven ");
      break;
    case 8: 
      EIGHT; 
      if (DEBUG) Serial.println("Eight ");
      break;
    case 9: 
      NINE; 
      if (DEBUG) Serial.println("Nine ");
      break;
    case 10: 
      HTEN; 
      if (DEBUG) Serial.println("Ten ");
      break;
    case 11: 
      ELEVEN; 
      if (DEBUG) Serial.println("Eleven ");
      break;
    case 12: 
      TWELVE; 
      if (DEBUG) Serial.println("Twelve ");
      break;
      }
  }
  else
  {
      // if we are greater than 34 minutes past the hour then display
      // the next hour, as we will be displaying a 'to' sign
      TO;
      if (DEBUG) Serial.print("To ");
      switch (hour) {
      case 1: 
        TWO; 
       if (DEBUG) Serial.println("Two ");
       break;
      case 2: 
        THREE; 
      if (DEBUG) Serial.println("Three ");
        break;
      case 3: 
        FOUR; 
      if (DEBUG) Serial.println("Four ");
        break;
      case 4: 
        HFIVE; 
      if (DEBUG) Serial.println("Five ");
        break;
      case 5: 
        SIX; 
      if (DEBUG) Serial.println("Six ");
        break;
      case 6: 
        SEVEN; 
      if (DEBUG) Serial.println("Seven ");
        break;
      case 7: 
        EIGHT; 
      if (DEBUG) Serial.println("Eight ");
        break;
      case 8: 
        NINE; 
      if (DEBUG) Serial.println("Nine ");
        break;
      case 9: 
        HTEN; 
      if (DEBUG) Serial.println("Ten ");
        break;
      case 10: 
        ELEVEN; 
      if (DEBUG) Serial.println("Eleven ");
        break;
      case 11: 
        TWELVE; 
        if (DEBUG) Serial.println("Twelve ");
      break;
      case 12: 
        ONE; 
      if (DEBUG) Serial.println("One ");
      break;
    }
  }
  oled.setInvertMode(false);
}
