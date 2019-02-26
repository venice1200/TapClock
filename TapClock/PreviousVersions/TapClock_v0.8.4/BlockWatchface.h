/* 
 * BlockWatchface.h 
 * 
 */

#ifndef BlockWatchface_h
#define BlockWatchface_h

// You need to program the function "void showWatchface()" 
// which is started from the Main Sketch if the Watch is shown.
// The following Variables are set from the Main Sketch and can be used here.

extern SSD1306AsciiSpi oled;               // The OLED Object 
extern bool AlarmOn;                       // Alarm active
extern bool AlarmEnabled;                  // Alarm enabled
extern bool batteryWarning;                // Battery is below warning
extern bool MPUEnabled;                    // MPU Enabled
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
extern bool blink500;                      // 500ms Blinker
extern bool blink250_pos;                  // 250ms Blinker Positive Flanc  
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

void showBlockDPoint (uint8_t x, uint8_t y) {
  // Save current font
  const uint8_t* currentFont = oled.font();
  
  // Set Clock5x7 Font
  oled.setFont(Clock5x7);

  oled.setCursor(x,y);
  oled.print("1");
  //oled.setCursor(x,y+1);
  //oled.print(" ");
  oled.setCursor(x,y+2);
  oled.print("1");

  // Set Font back
  oled.setFont(currentFont);
}

void showBlockNumber (uint8_t x, uint8_t y, uint8_t value) {
  // Save current font
  const uint8_t* currentFont = oled.font();
  
  // Set Clock5x7 Font
  oled.setFont(Clock5x7);
  
  switch (value) {
    case 0:
      oled.setCursor(x,y);
      oled.print("(**)");
      oled.setCursor(x,y+1);
      oled.print("* .*");
      oled.setCursor(x,y+2);
      oled.print("*./*");
      oled.setCursor(x,y+3);
      oled.print("*/ *");
      oled.setCursor(x,y+4);
      oled.print("+**,");    
    break;
    case 1:
      oled.setCursor(x,y);
      oled.print("  (*");
      oled.setCursor(x,y+1);
      oled.print("   *");
      oled.setCursor(x,y+2);
      oled.print("   *");
      oled.setCursor(x,y+3);
      oled.print("   *");
      oled.setCursor(x,y+4);
      oled.print("   *");
    break;

    case 2:
      oled.setCursor(x,y);
      oled.print("(**)");
      oled.setCursor(x,y+1);
      oled.print("   *");
      oled.setCursor(x,y+2);
      oled.print("(**,");
      oled.setCursor(x,y+3);
      oled.print("*   ");
      oled.setCursor(x,y+4);
      oled.print("****");    
    break;

    case 3:
      oled.setCursor(x,y);
      oled.print("(**)");
      oled.setCursor(x,y+1);
      oled.print("   *");
      oled.setCursor(x,y+2);
      oled.print(" ***");
      oled.setCursor(x,y+3);
      oled.print("   *");
      oled.setCursor(x,y+4);
      oled.print("+**,");    
    break;

    case 4:
      oled.setCursor(x,y);
      oled.print("*   ");
      oled.setCursor(x,y+1);
      oled.print("* * ");
      oled.setCursor(x,y+2);
      oled.print("+***");
      oled.setCursor(x,y+3);
      oled.print("  * ");
      oled.setCursor(x,y+4);
      oled.print("  * ");    
    break;

    case 5:
      oled.setCursor(x,y);
      oled.print("****");
      oled.setCursor(x,y+1);
      oled.print("*   ");
      oled.setCursor(x,y+2);
      oled.print("***)");
      oled.setCursor(x,y+3);
      oled.print("   *");
      oled.setCursor(x,y+4);
      oled.print("+**,");
    break;

    case 6:
      oled.setCursor(x,y);
      oled.print("(**)");
      oled.setCursor(x,y+1);
      oled.print("*   ");
      oled.setCursor(x,y+2);
      oled.print("***)");
      oled.setCursor(x,y+3);
      oled.print("*  *");
      oled.setCursor(x,y+4);
      oled.print("+**,");    
    break;

    case 7:
      // Original 7
      oled.setCursor(x,y);
      oled.print("(**)");
      oled.setCursor(x,y+1);
      oled.print("   *");
      oled.setCursor(x,y+2);
      oled.print("   *");
      oled.setCursor(x,y+3);
      oled.print("   *");
      oled.setCursor(x,y+4);
      oled.print("   *"); 
    break;

    case 8:
      oled.setCursor(x,y);
      oled.print("(**)");
      oled.setCursor(x,y+1);
      oled.print("*  *");
      oled.setCursor(x,y+2);
      oled.print("****");
      oled.setCursor(x,y+3);
      oled.print("*  *");
      oled.setCursor(x,y+4);
      oled.print("+**,");    
    break;

    case 9:
      oled.setCursor(x,y);
      oled.print("(**)");
      oled.setCursor(x,y+1);
      oled.print("*  *");
      oled.setCursor(x,y+2);
      oled.print("+***");
      oled.setCursor(x,y+3);
      oled.print("   *");
      oled.setCursor(x,y+4);
      oled.print("+**,");    
    break;
  }
  // Set Font back
  oled.setFont(currentFont);
}

void printSpecialChar(uint8_t x, uint8_t y, uint8_t c) {
  // Save current font
  const uint8_t* currentFont = oled.font();

  // Set Clock5x7 Font
  oled.setFont(Clock5x7);
  
  oled.setCursor(x,y);
  oled.print(char(c));

  // Set Font back
  oled.setFont(currentFont);
}

void showBatteryIcon(uint8_t x, uint8_t y) {
  if ((percent >=  0) && (percent < 25)) printSpecialChar(x,y,35);
  if ((percent >= 25) && (percent < 50)) printSpecialChar(x,y,36);
  if ((percent >= 50) && (percent < 75)) printSpecialChar(x,y,37);
  if ((percent >= 75) && (percent < 100)) printSpecialChar(x,y,38);
  if (percent == 100) printSpecialChar(x,y,39);
}

void showWatchface() {
  uint8_t hr10  = act_hr / 10;
  uint8_t hr1   = act_hr % 10;
  uint8_t min10 = act_min / 10;
  uint8_t min1  = act_min % 10;
  uint8_t sec2  = act_sec % 2; 

  //if (sec_changed || init_view) {
  if (min_changed || init_view) {
    showBlockNumber (0,   2, hr10);
    showBlockNumber (30,  2, hr1);
    showBlockNumber (73,  2, min10);
    showBlockNumber (103, 2, min1);
  }
  
  // Double-Point & Battery Symbol blinker
  if (sec2 && sec_changed) {
    showBlockDPoint (60, 3);
  }
  if (!sec2 && sec_changed) {
    oled.clear(60, 64, 3, 5);
  } //endif sec2
  
  // Show/Actualize Battery Icon only at init or every 10secs if Battery Warning Level is not reached
  if ((!batteryWarning && sec_changed) || init_view) {
    showBatteryIcon(0,0);
  }
  // Battery Warning = Blinking Battery Icon
  if (batteryWarning & blink500 && blink250_pos) {
    printSpecialChar(0,0,32);   // Clear Battery Symbol Position
  }
  if (batteryWarning & !blink500 && blink250_pos) {
    showBatteryIcon(0,0);       // Show Battery Symbol
  } //endif sec2

  // Show Bell Icon, no need to clean this as you have 
  // to leave the Menu for disabling the Alarm
  if (init_view) {
    if (AlarmEnabled) {
      printSpecialChar(6,0,45);  // Bell/Space Shuttle Icon
    }
    else {
      printSpecialChar(6,0,32);  // Clear the Area
    }
  }
  // MPU Icin
  if (init_view && MPUEnabled) {
    printSpecialChar(12,0,51);
  }
      
  if (min_changed || init_view) {
    // Show date
    oled.setCursor(24,0);
    oled << act_day << "." << months[act_month] << " " << act_year+2000 << " (" << days[act_dayofweek] << ")";
  }  //endif min_changed

  // Show seconds indicator
  // Build the line
  if (init_view) {
    for (uint8_t i=0; i <act_sec; i++) {
      printSpecialChar(1+i*2,7,50);
    }
  }
  // Clear line at 0 secs
  if ((act_sec == 0) && sec_changed) {
    oled.setCursor(0,7);
    oled.clearToEOL();
  }
  // Show sign
  if (sec_changed || init_view) {
    printSpecialChar(1+act_sec*2,7,50);
  }
  
  init_view = false;  //Init View done
}  // End Watchface

#endif
