/* 
 * WordWatchface.h 
 * 
 */

#ifndef WordWatchface_h
#define WordWatchface_h

// You need to program/define the function "void showWatchface()" 
// which ist started from the Main Sketch if the Watch is shown.
// The following Variables are set from the Main Sketch and can be used here.

extern SSD1306AsciiSpi oled;               // The OLED Object 
extern bool AlarmOn;                       // Alarm active
extern bool AlarmEnabled;                  // Alarm enabled
extern uint8_t act_hr;                     // 8-Bit Integer (Byte) contains RTC Hour value
extern uint8_t act_min;                    // 8-Bit Integer (Byte) contains RTC Minute value
extern uint8_t act_sec;                    // 8-Bit Integer (Byte) contains RTC Second value 
extern uint8_t act_year;                   // 8-Bit Integer (Byte) contains RTC Year value "two digit lenght"
extern uint8_t act_month;                  // 8-Bit Integer (Byte) contains RTC Month value, see also char *months[13];
extern uint8_t act_day;                    // 8-Bit Integer (Byte) contains RTC Day value
extern uint8_t act_dayofweek;              // 8-Bit Integer (Byte) contains RTC Day Of Week value, 1-7, 1 = Sunday, see char *days[8];
extern uint8_t act_hr12;                   // 8-Bit Integer (Byte) contains RTC Hour Value 1-12 value for AM/PM based Watchfaces
extern bool min_changed;                   // Bool available one MCU cycle indicates "Minute has changed" 
extern bool sec_changed;                   // Bool available one MCU cycle indicates "Second has changed" 
extern bool every10secs;                   // Bool available one MCU cycle indicates "Second has changed" but only all 10 seconds 
extern const char *months[13];             // Array containing Month Names (0..12, 0 is empty)
extern const char *days[8];                // Array containing Day Names (0..7, 0 is empty)
extern float voltage;                      // Float containing Battery voltage, actualized each cycle if Watchface is shown
extern uint8_t percent;                    // 8-Bit Integer (Byte) containing the Battery level in Percentage, actualized each cycle if Watchface is shown
extern bool usbConnected;                  // Bool indicate USB Power is connected
extern bool init_view;                     // Bool can be used for Watchface Init 
                                           // Is set to true from Main Sketch before "showWatchface()" runs, should be set to false at the end of "showWatchface()
                                           // Normally Used for printing text immediately which is normally actualized each second/minute
extern char datebuffer[10];                // String/Char buffer for formated printed strings

//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------

// Using (lot's) defines for the output (setCursor/print) to the OLED
#define xoffs 24 // X Offset
#define ITIS    oled.setCursor(xoffs+0,0);oled.print("IT");oled.setCursor(xoffs+18,0);oled.print("IS");
#define MTEN    oled.setCursor(xoffs+36,0);oled.print("TEN");
#define HALF    oled.setCursor(xoffs+54,0);oled.print("HALF");
#define QUARTER oled.setCursor(xoffs+66,0);oled.print("A");oled.setCursor(xoffs+0,1);oled.print("QUARTER");
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

// Initial Lines 0-7 
#define L0      oled.setCursor(xoffs+0,0);oled.print("ITRISCTENHALF");
#define L1      oled.setCursor(xoffs+0,1);oled.print("QUARTERTWENTY");
#define L2      oled.setCursor(xoffs+0,2);oled.print("FIVECMINUTESH");
#define L3      oled.setCursor(xoffs+0,3);oled.print("PASTTOEONETWO");
#define L4      oled.setCursor(xoffs+0,4);oled.print("THREEFOURFIVE");
#define L5      oled.setCursor(xoffs+0,5);oled.print("SIXSEVENEIGHT");
#define L6      oled.setCursor(xoffs+0,6);oled.print("NINETENELEVEN");
#define L7      oled.setCursor(xoffs+0,7);oled.print("TWELVELOCLOCK");


void showWordClock(int hour, int minute, int second){
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
  ITIS;   // print it is
  
  // now we display the appropriate minute counter
  if ((minute>4) && (minute<10)) { 
    MFIVE; 
    MINUTES; 
  } 
  if ((minute>9) && (minute<15)) { 
    MTEN; 
    MINUTES; 
  }
  if ((minute>14) && (minute<20)) {
    QUARTER; 
  }
  if ((minute>19) && (minute<25)) { 
    TWENTY; 
    MINUTES; 
  }
  if ((minute>24) && (minute<30)) { 
    TWENTY; 
    MFIVE; 
    MINUTES;
  }  
  if ((minute>29) && (minute<35)) {
    HALF;
  }
  if ((minute>34) && (minute<40)) { 
    TWENTY; 
    MFIVE; 
    MINUTES;
  }  
  if ((minute>39) && (minute<45)) { 
    TWENTY; 
    MINUTES; 
  }
  if ((minute>44) && (minute<50)) {
    QUARTER; 
  }
  if ((minute>49) && (minute<55)) { 
    MTEN; 
    MINUTES; 
  } 
  if (minute>54) { 
    MFIVE; 
    MINUTES; 
  }

  if ((minute <5))
  {
    switch (hour) {
    case 1: 
      ONE; 
      break;
    case 2: 
      TWO; 
      break;
    case 3: 
      THREE; 
      break;
    case 4: 
      FOUR; 
      break;
    case 5: 
      HFIVE; 
      break;
    case 6: 
      SIX; 
      break;
    case 7: 
      SEVEN; 
      break;
    case 8: 
      EIGHT; 
      break;
    case 9: 
      NINE; 
      break;
    case 10: 
      HTEN; 
      break;
    case 11: 
      ELEVEN; 
      break;
    case 12: 
      TWELVE; 
      break;
  }
  OCLOCK;
  }
  else
    if ((minute < 35) && (minute >4))
    {
      PAST;
      switch (hour) {
    case 1: 
      ONE; 
      break;
    case 2: 
      TWO; 
      break;
    case 3: 
      THREE; 
      break;
    case 4: 
      FOUR; 
      break;
    case 5: 
      HFIVE; 
      break;
    case 6: 
      SIX; 
      break;
    case 7: 
      SEVEN; 
      break;
    case 8: 
      EIGHT; 
      break;
    case 9: 
      NINE; 
      break;
    case 10: 
      HTEN; 
      break;
    case 11: 
      ELEVEN; 
      break;
    case 12: 
      TWELVE; 
      break;
      }
  }
  else
  {
      // if we are greater than 34 minutes past the hour then display
      // the next hour, as we will be displaying a 'to' sign
      TO;
      switch (hour) {
      case 1: 
        TWO; 
       break;
      case 2: 
        THREE; 
        break;
      case 3: 
        FOUR; 
        break;
      case 4: 
        HFIVE; 
        break;
      case 5: 
        SIX; 
        break;
      case 6: 
        SEVEN; 
        break;
      case 7: 
        EIGHT; 
        break;
      case 8: 
        NINE; 
        break;
      case 9: 
        HTEN; 
        break;
      case 10: 
        ELEVEN; 
        break;
      case 11: 
        TWELVE; 
      break;
      case 12: 
        ONE; 
      break;
    }
  }
  oled.setInvertMode(false);
}

void showAmPmIcon(byte x, byte y, byte hr) {
  // Save current font
  const uint8_t* currentFont = oled.font();

  // Set Clock5x7 Font
  oled.setFont(Clock5x7);
  //Show AM/PM
  oled.setCursor(x,y);
  if (hr >= 12) {
    oled.print(char(33));
  }
  else {
    oled.print(char(34));
  }
  // Set Font back
  oled.setFont(currentFont);
}

void printSpecialChar(byte x, byte y, byte c) {
  // Save current font
  const uint8_t* currentFont = oled.font();
  // Set Clock5x7 Font
  oled.setFont(Clock5x7);
  // Set Position
  oled.setCursor(x,y);
  // Print Character
  oled.print(char(c));
  // Set Font back
  oled.setFont(currentFont);
}

void showBatteryIcon(byte x, byte y) {
  if ((percent >=  0) && (percent < 25)) printSpecialChar(x,y,35);
  if ((percent >= 25) && (percent < 50)) printSpecialChar(x,y,36);
  if ((percent >= 51) && (percent < 75)) printSpecialChar(x,y,37);
  if ((percent >= 75) && (percent < 100)) printSpecialChar(x,y,38);
  if (percent == 100) printSpecialChar(x,y,39);
}
void showWatchface() {
  // Vars for WordClock
  int act_hr12  = act_hr;         // 0..23
  if (act_hr12 > 12) act_hr12 -= 12;  // 0..23 => 0..12
  if (act_hr12 == 0) act_hr12 = 12;   // 0 = 12
  
  //Show WordClock at Init and every minute
  if (min_changed || init_view) showWordClock(act_hr12, act_min, act_sec);
    
  // Show Battery only at init or every 10secs with usb connected
  if (every10secs || init_view) {
    showBatteryIcon(0,0);

    // Show
    unsigned char bat_hdrs = percent/100;
    unsigned char bat_tens = percent%100/10;
    unsigned char bat_ones = percent%10;
     
    // Hundred's
    oled.setCursor(0,4);
    if (bat_hdrs > 0) {
      oled.print(bat_hdrs);
    }
    else {
      oled.print(" ");
    }
    // Ten's
    oled.setCursor(0,5);
    //if (bat_tens >= 0) {
      oled.print(bat_tens);
    //}
    if ((bat_tens == 0) && (bat_hdrs == 0)) {
      oled.print(" ");
    }
    // One's 
    oled.setCursor(0,6);
    oled.print(bat_ones);
    
    oled.setCursor(0,7);
    oled.print("%");
    //Show AM/PM
    showAmPmIcon(122,0,act_hr);
  } // endif Show Battery
  init_view = false;  //Init View done
}  // end showWatchface

#endif
