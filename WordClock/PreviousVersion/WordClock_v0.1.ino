

/* 
 WordClock for WatchX 

 The beginning...
 
 Based on Word Clock from:
  Original Copyright (C) 2009  Doug Jackson
  Modifications Copyright (C) 2010 Scott Bezek

 Used Libs
  -WatchX Libs
  -OLED Library to SSD1306Ascii by Greiman

 History
   v0.1 Initial Version, just OLED & RTC
 
*/

#include "SSD1306Ascii.h"
#include "SSD1306AsciiSpi.h"
#include "RTClib.h"

SSD1306AsciiSpi oled;
RTC_DS3231      rtc;

#define OLED_DC     A3
#define OLED_CS     A5
#define OLED_RST    A4
#define LEDL        13
#define LEDR        6
#define BUTTON1     8 
#define BUTTON2     11
#define BUTTON3     10
//#define DEBUG       0  // Debug or not Debug
#define DEBUG       1  // Thats the question

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

int  hour=0, minute=0, second=0;
int prev_min = 61, prev_sec = 61;  // 61 = to be sure the xxx_changed (see next line) booleans  will be true
bool min_changed = false, sec_changed = false;

void setup()
{
  // Initialise
  if (DEBUG) Serial.begin(19200);

  rtc.begin();

  // Init Display and Clear up
  oled.begin(&Adafruit128x64, OLED_CS, OLED_DC, OLED_RST);
  oled.setFont(System5x7);
  oled.clear();

  displaytime();        // display the current time
}

void loop(void)
{ 

  DateTime now = rtc.now();
  hour      = now.hour();
  minute    = now.minute();
  second    = now.second();

  if (hour > 12) hour = hour-12;

    //Secs or Mins changed?
  if (minute != prev_min) {
    min_changed = true;
  }
  else {
    min_changed = false;
  }

  if (second != prev_sec) {
    sec_changed = true;
  }
  else {
    sec_changed = false;
  }
  
  prev_min = minute;
  prev_sec = second;
  
  if (min_changed) displaytime();
}     

//-------------------------------------------

void displaytime(void){
  // start by clearing the display to a known state

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




