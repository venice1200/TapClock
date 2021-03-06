/* 
 * BasicWatchface.h 
 * 
 */

#ifndef BasicWatchface_h
#define BacicWatchface_h

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


void showWatchface() {
  // Add your code here
  oled.setCursor(0,3);
  oled.print("Add your own Code :-)");

  // Set init_view to false
  init_view = false;  //Init View done
}

#endif
