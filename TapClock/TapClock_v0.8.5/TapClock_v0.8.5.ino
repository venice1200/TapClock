/*
    TapClock for watchX v1.2 (v1.3 uses a different Magnetometer)
    The beginning...
	  The MPU6050 is (for me) a really complex device with lot's of registers and the possibilty to programm the internal DMP.
    I took the demo freefall sktech which uses the interrupt and changed it from freefall to motion, added a few values, thats all.
    Maybe I get sometime, after ten years of reading docs, gestures to work.

    Arduino IDE 1.8.5/1.8.8

    Libraries:
    -watchX libs by ArgeX                   see http://watchx.io/downloads_en.html
    -MPU6050 Lib from Korneliusz Jarzebski  see https://github.com/jarzebski/Arduino-MPU6050
    -BasicWatch 1 from kghunt/DntPMme       see https://github.com/kghunt/Basic_Watch
    -I2CDEVLIB                              see https://github.com/jrowberg/i2cdevlib 
    -Bounce2                                see https://github.com/thomasfredericks/Bounce2
    -Pin Change Interrupt (PCINT)           see https://github.com/NicoHood/PinChangeInterrupt
    -SSD1306 Text Library                   see https://github.com/greiman/SSD1306Ascii
    -DS3232RTC RTC Library                  see https://github.com/JChristensen/DS3232RTC
    -Streaming by Mikal Hart                see https://github.com/geneReeves/ArduinoStreaming fork of Mikal Hart's streaming5.zip
    -NewTone by Tim Eckel                   see https://bitbucket.org/teckel12/arduino-new-tone/wiki/Home

    Tips & Tricks:
    -Angle calculation                      see https://electronics.stackexchange.com/questions/142037/calculating-angles-from-mpu6050
    -Set Time at "Setup"                    see http://www.l8ter.com/?p=417
    -Nick Gammon Microprocessors Infos      see http://www.gammon.com.au/power and http://www.gammon.com.au/interrupts
    
    v0.1 
    -Initial Version
    -Show only Time in 4x Letters
    
    v0.2
    -Change OLED Library to SSD1306Ascii by Greiman to get (a lot) more Space, see https://github.com/greiman/SSD1306Ascii
    -Add: "init_view" and "min_changed/sec_changed" booleans for updating screen values only if they are changed (see Note) or when the display is activated
    -Add: battery and date values shown on display
    Note: If you update the display each "main loop" the chars on the display are flickering

    v0.3
    -Add: Uptime in hrs:min

    v0.3.1
    -Fix: Uptime, Seconds

    v0.4 (~6 hrs runtime until 1%)
    -Add: First try of power saving, see http://www.gammon.com.au/power
    -Add: Using INT6/Pin7/IMU INT for waking up
          Waking the system from MPU-INT using "rising" or "falling" mode is only working in "SLEEP_MODE_IDLE"
          See 32u4 Manual Chapter 11: "...recognition of falling or rising edge interrupts on INT6 requires the presence of an I/O clock"

    v0.4.1
    -Add: Disable ADC for more power saving via "power_adc_disable()" in State 7 (sleep)
          Enable ADC for battery measuring "power_adc_enable()" in State 5 (wakeup)
    -Change: interrupt definition back to "setup"

    v0.5 (Testing)
    -Change: MPU6050 Lib to I2CDEVLIB, see https://github.com/jrowberg/i2cdevlib 
             as of the possibilty to change MPU Interrupt handling to "Active LOW"
    -Change: 32u4 INT6 interrupt "mode" from "RISING" to "LOW"
    -Change: Power save from "SLEEP_MODE_IDLE" to "SLEEP_MODE_STANDBY". Wakeup from "SLEEP_MODE_PWR_DOWN" is not working.
	  -Modify: Dokumentation

    v0.5.1
    -Adding "disable_sleep();" after "sleep_cpu()" in State 7 for testing "SLEEP_MODE_PWR_DOWN"
     "SLEEP_MODE_PWR_DOWN" needs MPU setting "setInterruptLatch(1)" to latch the interrupt signal.
     If the MPU latch'es the interrupt you have to clear the interrupt manually by reading the interrupt status usiung "[boolean] = getIntMotionStatus()".
     Reading the motion int bit clears it to 0.
     An non latched MPU Interrupt is only a 50us signal which seems to short to wake from "SLEEP_MODE_PWR_DOWN".

    v0.5.2
    -Added: Found at watchx reddit and kghunt's github repo some commands for power saving related to NRF51822 Bluetooth Module, just added not tested.
    -Added: rtc.lostPower() added to setup to prevent setting the clock back to compile time on reset
    -Added: Detect USB Power for an charging indicator, I use Variant 2
            Variant 1 detects USB communication lines
            Variant 2 detects USB "Power Only" cables
    -Added: Charging will be shown as LEDR glowing
    -Added: As long as USB Power is present the Display keeps showing the clock and the watch don't go to sleep
            Nice feature as you can easily transfer your sketch without the need for an reset

    v0.5.3
    -Changed: Removed Delays from Intro and replaced with millis. LEDR glowing works from the beginning
    -Changed: States; 0 Start, >=10 Intro, >=50 Clock, >=90 Sleep
    -Changed: Renaming some Variables
    -Added:   Show Gyro Values
    -Testing: readclockpos
    -Testing: Angle calculation from here: https://electronics.stackexchange.com/questions/142037/calculating-angles-from-mpu6050
    -Disabled: MAG3110 and send into standby

    v0.5.4
    -Added: 8 secs offset for "rtc.adjust" 
    -Added: Show "real" Runtime since last start
    -Testing: Setting time in "setup" like written here http://www.l8ter.com/?p=417  (Works better)
    -Testing: Init MAG3110 and send into standby
    -Issue: Sometimes the system don't start correctly after upload or reset => Bluetooth power save code disabled
    -Fix: Runtime Hrs (removed %24)

    v0.5.5
    -Added:  If clock is woken up a 2500ms timer is running
             If the timer finished completly the clock is going to sleep again
             If the MPU detects two clock positions before the system is going to sleep the clock is shown
    -Added:  LEDL shows the awaked watch
    -Added:  waittimeover bool
    -Added:  function checkposition
    -Added:  Stats counter for wake up/show display
    -Change: Scale of Accel and Gyro back to values from mpu.initialize(); 
             No need for detection of 2000°/s or 16g

    v0.5.6
    -Added:  First try of an Setup Mode but the left button breaks
    -Added:  Buttons "UnBounce" with Bounce2 lib

    v0.5.7
    -Added:  function "setState"; Copies "old" state and Set new one
    -Added:  Stats Page, open with Button 2 
             Showing uptime, counter and angles

    v0.5.8 (testing)
    -Added: First Gestures !
            Rotating through Time Menu and modfify values also with Gestures Clock Up and Down
    -Added: DayOfTheWeek added to Date

    v0.5.9 (testing)
    Changed: Enhanced the Menu code for more Values (Clocktime)
    Added:   Try again to reduce Bluetooth Power Consumption, see: https://community.watchx.io/t/basic-watch-v2-in-progress/129/34
    Changed: Intro (Hardware Init shown)
    Changed: "Display ON" Time splitted in Clocktime (35) and Multiplier (100) in [ms]
    Weird:   

    v0.6.0 (released)
    Changed: The "#define BLE" enables NRF power saving which doesn't work i think
    Added:   WordClock Watchface
    Added:   New Menu Option Watchface: 1=Clock 2=Wordclock (TapClock)
    Added:   New Menu Option Wakeup from: 1=MPU 2=Button
    ToDo:    Button Wakeup disables MPU
             Stats Page if WordClock is chosen
    
    v0.6.1 (released)
    Added:   WakeupMode 1=MPU 2=Button

    v0.6.2 (testing)
    Changed: Detect MPU Sleep Mode for "Stats" now with mpu.getSleepEnabled() to get the "real" state
    Added:   Using new self-made font "Clock5x7.h". Font System5x7.h can be reverted.
             "allFonts.h" in "src/fonts" need another line "#include "Clock5x7.h" to cover the new font.
    Changed: Menu position Exit and Save

    v0.6.3 (released)
    Added:   Array of "struct" containing values and a changed flag for the Setup Elements
             Only values changed during Setup are saved back to the "working" values.
    Added:   Watchface 3 BlockClock
    
    v0.6.4 (released) Happy New Year Edition
    Changed: Stats
             Reset Runtime and wake counters with URB
             Showing Battery Level & Voltage
    Added:   EEPROM reading and writing
             At setup() the EEPROM Byte 0 is read at first.
             If Byte 0 is not Decimal 97, initial values for Clocktime (40), Watchface (1), Wakeup Mode (1) and Contrast (10) 
             are written to EEPROM Bytes 2-5 and Byte 0 get the 97. 
             The Oled shows an -Init- at Boot Time
             If Byte 0 has got the 97, values for Clocktime, Watchface, Wakeup Mode and Contrast are read and saved.
             Setup->Save saves only changed values to EEPROM.
    Changed: Cosmetics at System Start and "Save Setup"
    Changed: Re-Write Setup Menu Rotation
    Changed: Setup Exit and Save are splitted
    Changed: Include of "Clock5x7.h" Font file through the sketch instead of modifying the SSD1306ASCii library.
    Added:   100ms Blinker
    Added:   Delay of Button URB/ULB (2+3)
             If you press Button URB or ULB for a longer time (see buttonDelay) the Values in the Setup menu changing 10x faster

    v0.7.x (testing a lot)
    
    v0.8.0 (released) Make the Watchface Modular
             All changes since v0.6.4
    Change:  RTC Lib DS3232 with RTC Alarm & Interrupt support
    Change:  As off low Memory (>95%) Watchface Code is moved out from the Main Sketch into separate .h files
             The Sketch himself supports only one Watchface, you need to comment the others out.
             The .h files got (external) Variables from the Main Sketch and must contain the function "void showWatchface()" which is called from the Main Sketch.
    Added:   Slightly modificated Clock5x7 Block Font v0.2
    Added:   Setup Alarm Parameter
    Change:  Lot's of cosmetics (Boot/Setup)
    Change:  All needed files included in the Main Sketch's folder (Watchface & Font)
    Added:   New Alarm Parameter (On-Off/Mode/Date/Day/Hour/Minute), new Alarm Blinker, Buzzer
    Change:  A lot "int's" changed to "uint8_t" 
             oled.print(F("abc123")) => oled.print("abc123") freering up some Program Storage
    Change:  Using the integrated Arduino Tone library results in Compiler Errors or the watchX Clock stops working after the Intro.
             Swaped over to the "NewTone" Library from Tim Eckel, Code is also smaller and no compiler error.                
    Change:  Values of "setupElementName[]" are now part of the Struct-Array "setupElement[]"
    Change:  Reset Counter in "Stats" need URB long pressed now
    Added:   Browsing Up or Down in Setup Menu with longer Button press
    Info:    It looks like Arduino 1.8.8 is responsible for the above Compiler errors. 
             IDE 1.8.5 works better.   

    BlockWatchface 
    Added:   Seconds Star Indicator

    v0.8.1 (Testing(
    Change:  Alarm handling changed
             "AlarmOn" prevents the system from send to sleep, "State 50" not set.
             New: isAlarmDay 

    v0.8.2 (Released)
    Added:   Wakeup via Button is permanently active
    Change:  Button and MPU Interrupt handling separated
    Change:  Rename and Re-Programm some Setup Value-Names
    Added:   Alarm Mode 6 (see Readme)
    Fix:     MPU Wakeup was not working because the fixed values for the two wakeup positions are not working with uint8_t. Values needs to be float.

    v0.8.3 (Testing)
    Added:   Battery Warning and Critical Alarms
    Deleted: Removed a lot "cosmetic" stuff to get free space (oled.print's/intro shorted/variables modified)
    Added:   Master Alarm (all RTC & Battery Alarms)
    
    v0.8.4 (Released)
    Added:   Icon for "MPU Enabled"

    v0.8.5 (Released)
    Changed: Menu Position for "MPU enabled", makes change easier.
    Bugfix:  Changed EEPROM.write(4,2); to EEPROM.write(4,0); Init Value for MPU Enabled/Disabled
    
    
    ToDo:   
    -Power saving by sending more unneeded sensors to sleep or switch off

    Font:
    This sketch uses an SSD1306ASCii compatible font file "Clock5x7.h".
    You have two possibities to add the font to the sketch:
    1) Add the font to the SSD1306ASCii Library and add the line #include "Clock5x7.h" to the library file "allFonts.h" to cover the font.
       The font must be part of the SSD1306ASCii Library file and folder structure.
    2) !!Preferred Method!!
       Add the font to this (or your sketch) by add an #include "Clock5x7.h" 
       and make the file "Clock5x7.h" available through the libraries or sketch folders.
       Since TapClock v0.7.0 I have all needed files in the sketch folder. 

    State:
    0     Start/Display On
    10..  Intro Steps
    50..  Steps before clock is shown, Position Detection
    60    Prepare Watchface
    61    Show Watchface
    90    Prepare System before Sleep, Oled/Led
    91    Sleep and Wakeup / Power Modes / Interrupt / Sleep
    100.. Setup
    110.. Stats

    EEPROM Bytes
    00:  Fixed value for detecting initial Setup done or not. Search for EEPROM_PRESETVAL
    01:  free            free
    02:  Clocktime       in 1/10sec
    03:  Watchface       unused
    04:  MPU Enabled     0=Off, 1=On
    05:  Contrast        0..25 (*10)
    06:  Alarm Enabled   0=Off, 1=On
    07:  Alarm Hour      0..23
    08:  Alarm Minute    0..59
    09:  Alarm Day       Date, DayOfWeek or Binary Coded Multi-Day of Week (see ReadMe)
    10:  Alarm Mode      1..6 see below

    Runtimes:
    MPU Wakeup:    ~20-22hrs
    Button Wakeup: ~48-50hrs

    RTC Alarm Modes:
    1:  ALM1_MATCH_SECONDS  = Match Seconds (Alarm once each Minute)
    2:  ALM1_MATCH_MINUTES  = ..and Minutes (Alarm once each Hours)
    3:  ALM1_MATCH_HOURS    = ..and Hours (Alarm once each Day) (also Default Mode)
    4:  ALM1_MATCH_DATE     = ..plus this Date or (Alarm once each Month)
    5:  ALM1_MATCH_DAY      = ..or plus this DayOfWeek (1-7) (Alarm once each Week)
    6:  ALM1_MATCH_HOURS    = Mode 3 plus Binary Coded Days of Week so you can choose more than one Day for the weekly Alarms (Mix of Hard- & Software)
    
*/

#include <avr/sleep.h>             // Needed for Sleep Modes
#include <avr/power.h>             // Needed for Power Off System Resources
#include <Wire.h>                  // I2C
#include <MPU6050.h>               // MPU Library including Interrupt functions
#include "SSD1306Ascii.h"          // OLED Lib Main Part
#include "SSD1306AsciiSpi.h"       // OLED Lib SPI Part
//#include "RTClib.h"                // Needed for the RTC Chip, no functions for HW Alarm
#include <SparkFun_MAG3110.h>      // Needed for MAG3110
#include <Bounce2.h>               // Needed to unbounce the Buttons
#include "PinChangeInterrupt.h"    // Needed for Button Interrupts
#include <EEPROM.h>                // Needed for accessing the EEPROM
#include <DS3232RTC.h>             // Alternative RTC Lib with Interrupt Support https://github.com/JChristensen/DS3232RTC
#include "Clock5x7.h"              // You need to have the file available in the Sketch or Library folders
#include <Streaming.h>             // Very usefull library, can be used for someting like oled << "Text" << variable << "Text2"
#include "NewTone.h"               // Used as off issues with the Arduino Tone Library during compile, generates also smaller code 

// ======================== Watchfaces ==========================
// Chose only one Watchface                                     Program/Global Vars used
//#include "WatchfaceSkeleton.h"       // Base Watchface File       87%/51%
//#include "BasicWatchface.h"        // Original TapClock Watchface 89%/55%
//#include "WordWatchface.h"         // WordClock Watchface         95%/57%
#include "BlockWatchface.h"        // Block Clock Watchface       93%/58%
// ==============================================================

// Objects
SSD1306AsciiSpi oled;
MPU6050         mpu(0x69);         // Chip's AD0 = 1
MAG3110         mag = MAG3110();
//RTC_DS3231      rtc;             // No need to define an RTC Object with DS3232RTC.h 


// "defines"
#define PRG_NAME "TapClock"
#define PRG_VERSION "v0.8.5"
#define EEPROM_PRESETVAL 95
//Debug or not Debug...
#define DEBUG 0                    
//#define DEBUG 1                  // Shows actually additional infos, maybe overwrite other data

#define OLED_DC            A3
#define OLED_CS            A5
#define OLED_RST           A4
#define BLE_CS             A2
#define BLE_IRQ            0
#define BLE_RST            A1
#define BAT_PIN            A11
#define BAT_EN             4
#define CRG_STATE          5        // PullUp, Low = Charging, High = Fully charged
#define BUZZER             9
#define MPU_INT            7        // PullUp
#define RTC_INT            1        // PullUp, Pin 1/ INT3/TX1
#define LEDL               13
#define LEDR               6
#define BUTTON1            8        // PullUp
#define BUTTON2            11       // PullUp
#define BUTTON3            10       // PullUp
#define COMP_OFFSET        8        // Compile and Transfer Time Offset in secs
#define DebounceTime       25       // Button Debounce Time
#define ALARMONTIME        100      // Alarm Blinker x milis on
#define ALARMOFFTIME       50       // Alarm Blinker x millis off
#define ALARMSEQCNT        8        // Alarm Blinker after x blink cycles
#define ALARMDELAYTIME     400      // Alarm Blinker delay for x millis
#define BUZZER_FREQ        1000     // Buzzer Alarm Frequency
#define BATTERY_CRITCAL    10       // Battery Critical Level 
#define BATTERY_WARNING    15       // Battery Warning Level 
#define BATTERY_OK         30       // Battery OK Level

// Instances of Bounce objects
Bounce b1debouncer = Bounce(); 
Bounce b2debouncer = Bounce(); 
Bounce b3debouncer = Bounce(); 

// Interrupt used Vars
volatile uint8_t State = 0;
volatile uint8_t prevState = 0;
volatile bool wokenFromBUT = false;
volatile bool wokenFromMPU = false;
volatile bool wokenFromRTC = false;

// Button Vars 
unsigned long b2pressed = 0;
unsigned long b3pressed = 0;
bool b2delayed = false;
bool b3delayed = false;
bool prev_b3delayed = false;
bool b3delayed_pos = false;
//const int buttonDelay = 1000;
const int buttonDelay = 1250;
//const int buttonDelay = 1500;

// Display Variables
unsigned long previousclockmillis = 0;
unsigned long currentmillis = 0;

uint8_t clocktime = 40;               // in 1/10 sec
const uint8_t clockmultiplier = 100;  // Clocktime Multiplier
bool ShowTime = false;
bool init_view;

//Date & Time
char datebuffer[10];
const char *months[13] = {"", "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
const char *days[8] = {"", "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
uint8_t prev_min = 61, prev_sec = 61;  // 61 = to be sure the xxx_changed (see next line) booleans  will be true when loop starts
bool min_changed = false, sec_changed = false; 

//Voltage Vars
//float R1 = 10000;
//float R2 = 10000;
//float vDivider;

//Interrupt Status, needed to clear Interrupt Bit
bool mpuintstatus = false;

//USB Power Detection
bool usbConnected = false;
bool charging = false;

// Battery
float voltage;
uint8_t percent;
bool batteryWarning = false;
bool prev_batteryWarning = false;
bool batteryAlarmWarning = false;
bool batteryCritical = false;
bool prev_batteryCritical = batteryCritical;
bool batteryAlarmCritical = false;
bool MasterAlarm = false;

//ChargeLED(R) Vars
long int chargeledmillis = 0;
const uint8_t chargeledinterval = 50;
const uint8_t chargeledmaxvalue = 255;
const uint8_t chargeledminvalue = 10;
uint8_t chargeledfadevalue = chargeledminvalue;         // !!Important to initialize the startvalue with minvalue!!
uint8_t chargeledfadesteps = 5;
bool LEDL_VAR = false;

//100ms Blinker Vars
long int blink100millis = 0;
const uint8_t blink100interval = 100;
bool blink100 = false;
bool prev_blink100 = false;
bool blink100_pos = false;  // Flanc

//250ms Blinker Vars
long int blink250millis = 0;
const uint8_t blink250interval = 250;
bool blink250 = false;
bool prev_blink250 = false;
bool blink250_pos = false;  // Flanc

//500ms Blinker Vars
long int blink500millis = 0;
const int blink500interval = 500;
bool blink500 = false;
bool prev_blink500 = false;
bool blink500_pos = false;  // Flanc

//Remove delays from Intro
uint32_t waitmillis = 0;
uint16_t waittime = 0;

// Clockpos/Waitpos (Z can be ignored)
const float xclockposmin =  15;
const float xclockposmax =  90;
const float yclockposmin = -30;
const float yclockposmax =  30;
const float xwaitposmin =  -90;
const float xwaitposmax =  -20;
const float ywaitposmin =  -30;
const float ywaitposmax =   30;

// startTime compileTime
//time_t startTime, compileTime, runTime;
time_t startTime, runTime;

// Counter
uint16_t counterWakeUp = 0, counterShowTime = 0;

// Menu Vars
// Setup-Element Structure
typedef struct {
  const char *name;
  uint8_t value;
  bool changed;
} setupStruct;

const uint8_t numberOfSetupElements = 16;  // 16 = 0..15
setupStruct setupElement[numberOfSetupElements] = {
    "",0,false,               //  0
    "Exit",0,false,           //  1
    "Save",0,false,           //  2
    "Hour",0,false,           //  3
    "Minute",0,false,         //  4
    "Day",0,false,            //  5
    "Month",0,false,          //  6
    "Year",0,false,           //  7
    "Alarm Enabled",0,false,   //  8
    "Alarm Hour",0,false,     //  9
    "Alarm Minute",0,false,   // 10
    "Alarm Day",0,false,      // 11
    "Alarm Mode",0,false,     // 12
    "Clocktime",0,false,      // 13
    "Contrast",0,false,       // 14
    "MPU Enabled",0,false     // 15
};

// Needed for SetupValue changed
uint8_t prev_setupElementValue;
bool setupElementValue_changed;

const uint8_t menuStart = 1;                         // First Menu Value from Array
const uint8_t menuEnd = numberOfSetupElements - 1;   // Last Menu Value from Array
const uint8_t menuValueStart = 3;                    // First real Menu Entry from Array
const uint8_t menuValueEnd = menuEnd;                // Last real Menu Entry from Array
const uint8_t menuValueExit = 1;                     // menuValuesX as Numerics
const uint8_t menuValueSave = 2;
const uint8_t menuValueHour = 3;
const uint8_t menuValueMinute = 4;
const uint8_t menuValueDay = 5;
const uint8_t menuValueMonth = 6;
const uint8_t menuValueYear = 7;
const uint8_t menuValueAlarmEnabled = 8;   // 0=Off, 1=On
const uint8_t menuValueAlarmHour = 9;      // 0..23
const uint8_t menuValueAlarmMinute = 10;   // 0..59 
const uint8_t menuValueAlarmDay = 11;      // Date or DayOfWeek Depending on Alarm Mode
const uint8_t menuValueAlarmMode = 12;     // See Alarm Doku
const uint8_t menuValueClocktime = 13;
const uint8_t menuValueContrast = 14;
const uint8_t menuValueMPUEnabled = 15;    // 0=Off, 1=On

const uint8_t menuEditLine = 3;                     // This is the Line where the values are editable
uint8_t menuIndex = 1;                              // Start at 1 (actual Exit)
uint8_t prev_menuIndex = menuIndex;                 // Helper for menuIndex
bool menuIndex_changed = false;
bool menuEditMode  = false;
bool setupEditDone = false;
const uint8_t menuTextPos = 6;
const uint8_t menuValuePos = 90;
float xMenu = 0, yMenu = 0;

// MPU Enabled, Watchface, Oled Contrast
bool MPUEnabled, prev_MPUEnabled;
uint8_t watchface, oledContrast;

// Alarm
bool RTCAlarm = false;
bool AlarmEnabled = false;
bool alarmBlink = false;
bool prev_AlarmBlink = false;
byte alarmCnt = 0;
uint16_t alarmInterval = ALARMONTIME;
unsigned long prevAlarmMillis = 0;
bool isAlarmDay = false;

// Variables from Loop
uint8_t act_hr, act_min, act_sec, act_year, act_month, act_day, act_dayofweek, act_hr12;
bool every10secs = false;

// -------------------------------------------------------------------
// ---------------------------- Setup --------------------------------
// -------------------------------------------------------------------

void setup() 
{
/*  
  if (DEBUG) {
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo's serial, others continue immediately
  }
*/
  // Init Display and Clear up
  oled.begin(&Adafruit128x64, OLED_CS, OLED_DC, OLED_RST);
  oled.setFont(System5x7);
  oled.clear();

  // RTC New lib
  oled.print("RTC");
  // initialize the alarms to known values, clear the alarm flags, clear the alarm interrupt flags
  RTC.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);
  oled.print(".");
  RTC.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);
  oled.print(".");
  RTC.alarm(ALARM_1);
  oled.print(".");
  RTC.alarm(ALARM_2);
  oled.print(".");
  RTC.alarmInterrupt(ALARM_1, false);
  oled.print(".");
  RTC.alarmInterrupt(ALARM_2, false);
  oled.print(".");
  RTC.squareWave(SQWAVE_NONE);
  oled.print(".");

  startTime = RTC.get();
  oled.print(".");
  time_t compileTime = getCompileTime();
  if (startTime < compileTime) {
    RTC.set(compileTime);
    oled.print(".");
    startTime = RTC.get();
    oled.print(".");
  }
  oled.println("Ok");

  // MPU Init and settings
  /*
  mpu.reset();                                            // On SPI you need to reset ACCL/TEMP/GYRO Signal Path as well (Register 104)
  delay(100);                                             // See MPU Register Doku Register 107 
  */
  //if (DEBUG) Serial.println("Initializing MPU...");
  mpu.initialize();
  mpu.setAccelerometerPowerOnDelay(3);                    // MPU6050_DELAY_3MS
  mpu.setDHPFMode(MPU6050_DHPF_5);                        // MPU6050_DHPF_5HZ
  //mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);        // MPU Gyro_Config Register 1B/27, after init 250°/s
  //mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);        // MPU Accel_Config Register 1C/28, after init 2g
  mpu.setIntFreefallEnabled(false);                       // Disable Freefall Interrupt
  mpu.setIntZeroMotionEnabled(false);                     // Disable Zero Motion Interrupt
  mpu.setIntMotionEnabled(true);                          // Enable Motion Interrupt
  mpu.setInterruptMode(1);                                // Interrupt Mode; 1= active low / 0 = active high
  //mpu.setInterruptDrive(1);                              // open drain, not needed here !?
  mpu.setInterruptLatch(1);                               // Interrupt Latch Mode; 1 = signal until int bit manually cleared / 0 = 50us signal
  
  // Offsets got from IMU_Zero Sketch
  mpu.setXAccelOffset(106);
  mpu.setYAccelOffset(-397);
  mpu.setZAccelOffset(1255);
  mpu.setXGyroOffset(68);
  mpu.setYGyroOffset(-9);
  mpu.setZGyroOffset(29);
  
  //Modify the following for your needs
  mpu.setMotionDetectionThreshold(80); //80 
  mpu.setMotionDetectionDuration(4);   //4
 

  // Init Compass MAG3300 and send it directly into standby
  mag.initialize();
  mag.start();                  // Needed ??
  mag.enterStandby();

  // Setup Hardware
  // LEDs & Buzzer
  pinMode(LEDL, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(LEDL, LOW);
  digitalWrite(LEDR, LOW);
  digitalWrite(BUZZER, LOW);

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

  // Other Inputs
  pinMode(CRG_STATE, INPUT_PULLUP);
  // MPU Interrupt on INT6/Pin7, attaching/detaching moved to State 7
  pinMode(MPU_INT, INPUT_PULLUP);
  // RTC Interrupt
  pinMode(RTC_INT, INPUT_PULLUP);
  

  // For voltage calulation 
  //vDivider = (R2 / (R1 + R2));
  
  // Enable VBUS Pad for detecting USB Power
  USBCON|=(1<<OTGPADE); 
   
  // Check EEPROM Byte 0 for the value EEPROM_PRESETVAL (see defines)
  // If the check is false the preset values are stored to EERPOM no need to set them up again
  if (EEPROM.read(0) != EEPROM_PRESETVAL) { //
    EEPROM.write(0,EEPROM_PRESETVAL);       // Set Byte 0 to EEPROM_PRESETVAL Value
    EEPROM.write(1,0);        // Clear Byte 1
    EEPROM.write(2,40);       // Clocktime [*100ms]
//    EEPROM.write(3,1);        // Watchface (unused in this version)
    EEPROM.write(4,0);        // MPU Enabled:1 Disabled:0
    EEPROM.write(5,10);       // Contrast [*10]
    EEPROM.write(6,0);        // Alarm On/Off
    EEPROM.write(7,10);       // Alarm Hour
    EEPROM.write(8,0);        // Alarm Minute
    EEPROM.write(9,1);        // Alarm Day
    EEPROM.write(10,3);       // Alarm Mode 3 = match hours *and* minutes *and* seconds
  }

  // Get some values from EEPROM or set defaults
  // Clocktime 10-50 (*100)
  setupElement[menuValueClocktime].value = EEPROM.read(2);
  clocktime = setupElement[menuValueClocktime].value;
  
  // Wakeup Mode 0-1
  setupElement[menuValueMPUEnabled].value = EEPROM.read(4);
  MPUEnabled = setupElement[menuValueMPUEnabled].value;
  prev_MPUEnabled = MPUEnabled;
  
  //Contrast 1-25 (*10)
  setupElement[menuValueContrast].value = EEPROM.read(5);
  oledContrast = setupElement[menuValueContrast].value;

  //Alarm On/Off
  setupElement[menuValueAlarmEnabled].value = EEPROM.read(6);

  //Alarm Hour
  setupElement[menuValueAlarmHour].value = EEPROM.read(7);

  //Alarm Minute
  setupElement[menuValueAlarmMinute].value = EEPROM.read(8);

  //Alarm Day
  setupElement[menuValueAlarmDay].value = EEPROM.read(9);

  //Alarm Mode
  setupElement[menuValueAlarmMode].value = EEPROM.read(10);

  // ****** MPU *****
  // Enable MPU sleep if MPU is not enabled
  if ( MPUEnabled == 0 ) mpu.setSleepEnabled(true);

  // ***** Alarm *****
  // Set Alarm Time with EERPOM Values
  RTC.setAlarm(getAlarmMode(setupElement[menuValueAlarmMode].value), 0, setupElement[menuValueAlarmMinute].value, setupElement[menuValueAlarmHour].value, setupElement[menuValueAlarmDay].value);
  if ( setupElement[menuValueAlarmEnabled].value ) { 
    // Attach RTC Int at MCU, enable INT at RTC
    enableRTCInt();
    // Clear the Alarm1 Alarm flag
    RTC.alarm(ALARM_1);
  }

  getBattery();
  
  delay(2000);
}

// -------------------------------------------------------------------
// ------------------------ Main Loop --------------------------------
// -------------------------------------------------------------------

void loop()
{
  // main loop variables
  time_t sysTime;
  currentmillis = millis();

  // MPU Values
  int16_t ax=0, ay=0, az=0;
  float arx, ary, arz;
  bool readclockpos = false;
  bool waittimeover = false, waitclockpos = false;
  //  bool menuLeft = false, menuRight = false, menuUp = false, menuDown = false;
  bool menuUp = false, menuDown = false;

  // De-Bouncer Values
  //int b1, b2, b3;
  int b2, b3;

  // Update the De-Bounce instances
  b1debouncer.update();
  b2debouncer.update();
  b3debouncer.update();

  // Get the updated value from bounce:
  //b1 = b1debouncer.read();
  b2 = b2debouncer.read();
  b3 = b3debouncer.read();

  // Buttons delayed press
  //if (b1debouncer.fell()) b1pressed = currentmillis;             // set B1 delay time on "fell"
  //b1delayed = !b1 & (currentmillis - b1pressed > buttonDelay);
  if (b2debouncer.fell()) b2pressed = currentmillis;             // set B2 delay time on "fell"
  b2delayed = !b2 & (currentmillis - b2pressed > buttonDelay);
  if (b3debouncer.fell()) b3pressed = currentmillis;             // set B3 delay time on "fell"
  b3delayed = !b3 & (currentmillis - b3pressed > buttonDelay);

  //b3delayed_pos = b3delayed != prev_b3delayed?true:false;      // Positive & Negative Flanc (!=) !!!!!
  b3delayed_pos = b3delayed && !prev_b3delayed?true:false;       // Only Positive Flanc (&& !)
  prev_b3delayed = b3delayed;

  // Set Oled Contrast
  oled.setContrast(oledContrast * 10);

  sysTime = RTC.get();
  act_hr    = hour(sysTime);
  act_min   = minute(sysTime);
  act_sec   = second(sysTime);
  act_year  = year(sysTime) - 2000;
  act_month = month(sysTime);
  act_day   = day(sysTime);
  act_dayofweek = weekday(sysTime);

  // Runtime
  runTime = sysTime - startTime;

  // Secs or Mins changed?
  sec_changed = act_sec != prev_sec?true:false;
  min_changed = act_min != prev_min?true:false;
  // Update prev's
  prev_sec = act_sec;
  prev_min = act_min;

  // Special Ticks
  every10secs = ((act_sec % 10) == 0) && sec_changed;  // 10secs

// ---------------------- Alarm -----------------------------

  // Check if the present Day is Alarm Day for Alarm Mode 6
  isAlarmDay = ((1 << (act_dayofweek-1)) == (setupElement[menuValueAlarmDay].value & (1 << (act_dayofweek-1))))?true:false;
  
  if ( RTC.alarm(ALARM_1) && AlarmEnabled && ((isAlarmDay && (setupElement[menuValueAlarmMode].value == 6)) || (setupElement[menuValueAlarmMode].value != 6)) ) {
  //if (RTC.alarm(ALARM_1) && AlarmEnabled) {
    RTCAlarm = true;
  }
  
  // Clear Alarm with B3/LRB if Alarm is on and show the Clock
  if (RTCAlarm && b3debouncer.fell()) {
    RTCAlarm = false;    // Clear Alarm
    setState(60);     // Enable to show Clock
  }

  // Switch Alarm On/Off with B3 longpressed
  if (State==61) {                                    
    if (b3delayed_pos) {
      if (setupElement[menuValueAlarmEnabled].value) {
        setupElement[menuValueAlarmEnabled].value=false;
        disableRTCInt();
      }
      else {
        setupElement[menuValueAlarmEnabled].value=true;
        enableRTCInt();
      }
    init_view=true;                                 // Update Display
    }
  }
  AlarmEnabled = setupElement[menuValueAlarmEnabled].value;
  
// ---------------------------------------------------------


  // LED Fader
  if (currentmillis - chargeledmillis > chargeledinterval) {
    chargeledmillis = currentmillis;
    chargeledfadevalue = chargeledfadevalue + chargeledfadesteps;
    if ((chargeledfadevalue <= chargeledminvalue) || (chargeledfadevalue >= chargeledmaxvalue)) chargeledfadesteps = -chargeledfadesteps;
  }

  // 100ms Blinker
  if (currentmillis - blink100millis > blink100interval) {
    blink100millis = currentmillis;
    blink100 = !blink100;
  }

  // Positive flanc blink100 v2
  blink100_pos = blink100 != prev_blink100?true:false;
  prev_blink100 = blink100;

  // 250ms Blinker
  if (currentmillis - blink250millis > blink250interval) {
    blink250millis = currentmillis;
    blink250 = !blink250;
  }

  // Positive flanc blink250 v2
  blink250_pos = blink250 != prev_blink250?true:false;
  prev_blink250 = blink250;
  
  // 500ms Blinker
  if (currentmillis - blink500millis > blink500interval) {
    blink500millis = currentmillis;
    blink500 = !blink500;
  }

  // Positive flanc blink500 v2
  blink500_pos = blink500 != prev_blink500?true:false;
  prev_blink500 = blink500;

  // Alarm Blinker is a variant of blink without delay
  // uses onTime, offTime and a delayTime with an counter
  if(currentmillis - prevAlarmMillis >= alarmInterval) {           // Normal for blinking with millis
    if (alarmBlink) {                                          // Change interval from onTime to offTime if the bool is on
      if (alarmCnt < ALARMSEQCNT) {                            // As long as sequenz running
        alarmInterval = ALARMOFFTIME;                          // use "normal" offTime as Interval
      }
      else {                                                   // Sequenz done
        alarmInterval = ALARMDELAYTIME;                        // use other offTime (delay)
        alarmCnt = 0;                                          // reset sequence
      }
    }
    else {                                                     // 
      alarmInterval = ALARMONTIME;                             // if the bool is off use onTime as interval 
    }
    alarmBlink = !alarmBlink;                                  // an toggle the bool
    alarmCnt++;                                                // inc sequence counter
    prevAlarmMillis = currentmillis;                           // update buffer var
  }
  
  // Buzzer, Variant 2 with Tone Timer
  if ( (RTCAlarm || batteryAlarmCritical) && alarmBlink && !prev_AlarmBlink ) {
    NewTone(BUZZER, BUZZER_FREQ, ALARMONTIME);
  }

  // Update Prev from Alarm
  prev_AlarmBlink = alarmBlink;

   // USB connected? Variant 2 USB Power
  usbConnected = (USBSTA&(1<<VBUS));       

  // Generate charging bit & show it
  charging = usbConnected && !digitalRead(CRG_STATE);
  
  // Show LEDR charging
  if (charging) {
    analogWrite(LEDR, chargeledfadevalue);  
  }
  else {
    analogWrite(LEDR, 0);
  }
  
  // LEDL (Alarm + Watch Position MPU Wakeup)
  digitalWrite(LEDL, LEDL_VAR || (MasterAlarm && alarmBlink));
  
  // Read MPU Accl Values
  mpu.getAcceleration(&ax, &ay, &az);

   // Calculate accelerometer angles
  arx = (180/3.141592) * atan(ax / sqrt(square(ay) + square(az))); 
  ary = (180/3.141592) * atan(ay / sqrt(square(ax) + square(az)));
  arz = (180/3.141592) * atan(sqrt(square(ay) + square(ax)) / az);
  
  // Check Positions, waitpos = position before clock can be shown
  waitclockpos = checkposition(arx, ary, xwaitposmin, xwaitposmax, ywaitposmin, ywaitposmax);
  readclockpos = checkposition(arx, ary, xclockposmin, xclockposmax, yclockposmin, yclockposmax);
  
  // waittimeover ?? in v2
  waittimeover = currentmillis - waitmillis >= waittime?true:false;

  // State changes, Intro steps, see states now
  // Jump from Intro to Clock, see states now

  // If System is up and timer is still running and clock is in the right position show time (60), USB Power bypasses the position checks
  // From waitposition to readposition
  if ((State == 51) && ((!waittimeover && waitclockpos) || usbConnected)) setState(52);
  
  // From readposition to clock
  if ((State == 52) && ((!waittimeover && readclockpos) || usbConnected)) setState(60);
  
  // If timer is over without right position sleep again (90)
  if (((State == 51) || (State == 52)) && waittimeover) setState(90);
  
  // Jump to Power Display off
  if ((State == 61) && ((currentmillis - previousclockmillis) >= (clocktime * clockmultiplier))) setState(90);
  
  // If we are on USB Power do not run the "Show Clock" Timer so update the "previous" with the actual millis eaach cycle
  if ( ((State == 61) && usbConnected) || MasterAlarm) {
    previousclockmillis = currentmillis;
  }
  
  // Setup Tools Starting at 100
  // Setup Menu/Clock
  if ((State == 61) && b1debouncer.fell()) {
    setState(100);
  }
  // Setup Finished jump to save/exit
  if ((State == 101) && b1debouncer.fell() && getMenuFuncMode(menuIndex)) {
    setState(105);
  }
  
  // Jump to Stats
  if ((State == 61) && b2debouncer.fell()) {
    setState(110);
  }
  // Get back from Stats
  if ((State == 111) && b1debouncer.fell()) {
    setState(112);
  }

  //--------------------------------- MPU ---------------------------------------
  // If MPU Mode has changed enable/disable the MPU
  if ( MPUEnabled != prev_MPUEnabled ) {
    // 0: Disable MPU sleep
    if ( MPUEnabled == 0 ) {
      mpu.setSleepEnabled(true);      
    }
    // 1: Send MPU to Sleep
    if ( MPUEnabled == 1 ) {
      mpu.setSleepEnabled(false);
    }
  }
  // Update WakeupMode
  prev_MPUEnabled = MPUEnabled;

//--------------------------------- Battery ---------------------------------------
  // Battery handling; Info: Sleep Mode disables ADC conversion
  // Get Battery Values
  //if ( (State == 60) || (((State == 111) || (State == 101)) && sec_changed) )  {       // Read Battery
  if ( sec_changed )  {                                                               // Battery is read each second but only if watchX is woken up
	  getBattery();
  }
  // Set Level Variables
  if (percent > BATTERY_OK) {
    batteryWarning = false;
    batteryCritical = false;
  }
  if (percent < BATTERY_WARNING) {
	  batteryWarning = true;
  }
  if (percent < BATTERY_CRITCAL) {
    batteryCritical = true;
  }
  
  // Warning Level
  if (batteryWarning != prev_batteryWarning) {
    if (batteryWarning) {                           // batteryWarning 0-> 1
      batteryAlarmWarning = true;
    }
  }  
  // Critical Level
  if (batteryCritical != prev_batteryCritical) {
    if (batteryCritical) {                          // batteryCritical 0-> 1
      batteryAlarmCritical = true;
      oled.invertDisplay(true);
    }
    else {
      oled.invertDisplay(false);                    // batteryCritical 1 -> 0
    }
  }  
  
  // Reset Battery Alarms 
  if (batteryAlarmWarning && b3debouncer.fell()) {
    batteryAlarmWarning = false;
  }
  if (batteryAlarmCritical && b3debouncer.fell()) {
    batteryAlarmCritical = false;
  }
  // Update Prevs's
  prev_batteryWarning = batteryWarning;
  prev_batteryCritical = batteryCritical;

//--------------------------------- Master Alarm ---------------------------------------
  // Master Alarm
  MasterAlarm = RTCAlarm || batteryAlarmWarning || batteryAlarmCritical;
  
// --------------------- State Switcher ------------------------------

  switch (State) {
    case 0: 
      // Intro :-) Just for Fun
      oled.ssd1306WriteCmd(SSD1306_DISPLAYON);
      oled.clear();
      setState(10);
    break;

    case 10:
      oled.setCursor(32,0);
      oled.set1X();
      oled.print("Welcome to");
      oled.setCursor(19,7);
      oled << F(PRG_NAME) << " " << F(PRG_VERSION);
      oled.setCursor(28,3);
      oled.set2X();
      oled.print("watchX");
      oled.set1X();
      setWaittime(2500);
      setState(11);
    break;
    
    case 11:
      // Wait for timer
      if (waittimeover) setState(12);
    break;
    
    case 12:
      oled.clear();
      setWaittime(1000);
      setState(13);
    break;  
    
    case 13:
      // Wait for timer
      if (waittimeover) setState(60);
    break;
   
    case 50:  // Jumped to here from MPU WakeUp
      LEDL_VAR = true;
      counterWakeUp++;
      setWaittime(2500);
      setState(51);        // Two Position System
      //setState(60);
    break;

    case 51:
      // Wait for the "Wait-Position"
    break;

    case 52:
      // Wait for the "Read-Position"
      LEDL_VAR = blink250;
    break;

    case 60: // Jumped to here from Button WakeUp
      // Power off LEDL
      LEDL_VAR = false;
      // Prepare Display and Power On ADC
      // Set Clocktime Timer
      previousclockmillis =  millis();
      // Enable Display
      oled.ssd1306WriteCmd(SSD1306_DISPLAYON);
      oled.clear();
      // Some Variables
      init_view = true;
      ShowTime = true;
      if (prevState == 52) counterShowTime++;          // +1 only if coming from State 52 (MPU WakeUp)
      setState(61);
   break; // case 60

// ------------------------ Show Watchface ---------------------------

    case 61:
      // Show Time, Date, AM/PM/Battery etc. depending on included Watchface .h file
      showWatchface();
    break;  // case 61

    // Prepare System for Sleep
    case 90:
      // Prepare System for sleep
      // Clear Display on switch off
      oled.clear();
      oled.ssd1306WriteCmd(SSD1306_DISPLAYOFF);
      ShowTime = false;
      // Next Step => Sleep
      setState(91);
    break;

    // Sleep System and Wakeup
    case 91:
      // Send System into sleeeeeeeeeep....
      power_adc_disable();                          // Disable ADC System via PRR Register Bit 0 (PRADC)
      set_sleep_mode (SLEEP_MODE_PWR_DOWN);         // Setup SleepMode , see https://www.gammon.com.au/forum/?id=11497
      sleep_enable();                               // Enable Sleep Mode by setting SMCR Register Bit 0 and send the CPU to sleep
      mpuintstatus = mpu.getIntMotionStatus();      // Read MPU Motion INT Status before we attach the Interrupt. The bit clears to 0 after the register has been read

      // Enable/Disable 32u4 MPU interrupt
      // If no Alarm is active go for sleep
      if (!MasterAlarm) {
        digitalWrite(LEDL, false);  // Power LEDL off only to sure
        attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(BUTTON3), wakeFromBUTTON, FALLING);
        if ( MPUEnabled == 1 ) {
          attachInterrupt(digitalPinToInterrupt(MPU_INT), wakeFromMPU, LOW);
        }
        // Go sleeping....
        sleep_cpu();
      }

      // ***************************************************
      // ******* The Code starts here after wakeup *********
      // ***************************************************

      // Disable sleep
      sleep_disable();
      // Disable 32u4 MPU interrupt(s)
      detachInterrupt(digitalPinToInterrupt(MPU_INT));
      detachPinChangeInterrupt(digitalPinToPinChangeInterrupt(BUTTON3));
      // Read MPU Motion Int Register to clear it; The bit clears to 0 after the register has been read
      //mpuintstatus = mpu.getIntMotionStatus();
      // Enable ADC via PRR Register Bit 0 (PRADC)
      power_adc_enable();
    break;

// ------------------------- Setup Menu ------------------------------

    case 100:
      //
      menuIndex = 1;
      
      // Save current MPU X/Y Values
      xMenu = arx; 
      yMenu = ary;

      //Copy current Values into Array
      setupElement[menuValueYear].value       = act_year;
      setupElement[menuValueMonth].value      = act_month;
      setupElement[menuValueDay].value        = act_day;
      setupElement[menuValueHour].value       = act_hr;
      setupElement[menuValueMinute].value     = act_min;
      setupElement[menuValueClocktime].value  = clocktime;
      setupElement[menuValueMPUEnabled].value    = MPUEnabled;
      setupElement[menuValueContrast].value   = oledContrast;
      
      // Clear the setupElements changed state
      for (int i=0; i < numberOfSetupElements; i++) {
        setupElement[i].changed = false;
      }
      setupEditDone = false;  //
      setState(101);
    break;

    case 101:
      //Trying Gestures for Up/Down/Left/Right (Z can be ignored)
      menuUp =    checkposition(arx, ary, xMenu-40, xMenu-10, yMenu-5, yMenu+5);
      menuDown =  checkposition(arx, ary, xMenu+10, xMenu+40, yMenu-5, yMenu+5);

      // Edit Mode on/off
      if ( b1debouncer.fell() && getMenuEditMode(menuIndex) && setupEditDone ) menuEditMode = !menuEditMode;
 
      // Show Active Menu Entry Inverted
      if ( b1debouncer.fell() && menuEditMode ) {
        oled.setInvertMode(true);
        oled.setCursor(menuTextPos,menuEditLine);
        oled.print(setupElement[menuIndex].name);
        oled.setInvertMode(false);
      }
      // Show Active Entry Not Inverted
      if ( b1debouncer.fell() && !menuEditMode ) {
        oled.setCursor(menuTextPos,menuEditLine);
        oled.print(setupElement[menuIndex].name);
      }

      // MenuIndex +/- by pressing Left Side Buttons shortly or hold for a longer time 
      // or rotating your of Arm/watchX if MPU is enabeld
      if (!menuEditMode) {
        if (b2debouncer.fell() || ((menuUp || b2delayed) && blink250_pos)) menuIndex--;
        if (b3debouncer.fell() || ((menuDown || b3delayed) && blink250_pos)) menuIndex++;
      }

      // MenuIndex Bounds 
      if (menuIndex > menuEnd) menuIndex = menuStart;
      if (menuIndex < menuStart) menuIndex = menuEnd;

      // MenuIndex changed ?? => Update Menu & Cursor
      // v2 New version
      menuIndex_changed = menuIndex != prev_menuIndex?true:false;
      prev_menuIndex = menuIndex;
      
      // SetupElement Value +/- by pressing Left Side Buttons shortly or hold for a longer time 
      // or rotating your of Arm/watchX if MPU is enabeld
      if ( menuEditMode && getMenuEditMode(menuIndex) ) {
        if (b2debouncer.fell() || ((menuUp || b2delayed) && blink100_pos)) {
          setupElement[menuIndex].value++;
          setupElement[menuIndex].changed = true;
        }
        if (b3debouncer.fell() || ((menuDown || b3delayed) && blink100_pos)) { 
          setupElement[menuIndex].value--;
          setupElement[menuIndex].changed = true;
        }

        // Change Vakue Ranges
        switch(menuIndex) {
          case menuValueHour:
            //if (setupElement[menuIndex].value < 0) setupElement[menuIndex].value = 0; 
            if (setupElement[menuIndex].value > 23) setupElement[menuIndex].value = 0; 
          break;
          case menuValueMinute:
            //if (setupElement[menuIndex].value < 0) setupElement[menuIndex].value = 0; 
            if (setupElement[menuIndex].value > 59) setupElement[menuIndex].value = 0; 
          break;
          case menuValueDay:
            if (setupElement[menuIndex].value < 1) setupElement[menuIndex].value = 31; 
            if (setupElement[menuIndex].value > 31) setupElement[menuIndex].value = 1; 
          break;
          case menuValueMonth:
            if (setupElement[menuIndex].value < 1) setupElement[menuIndex].value = 12; 
            if (setupElement[menuIndex].value > 12) setupElement[menuIndex].value = 1; 
          break;
          case menuValueYear:
            if (setupElement[menuIndex].value < 19) setupElement[menuIndex].value = 99; 
            if (setupElement[menuIndex].value > 99) setupElement[menuIndex].value = 19; 
          break;
          case menuValueClocktime:
            if (setupElement[menuIndex].value < 1) setupElement[menuIndex].value = 99; 
            if (setupElement[menuIndex].value > 99) setupElement[menuIndex].value = 1; 
          break;
          case menuValueMPUEnabled:
            //if (setupElement[menuIndex].value < 0) setupElement[menuIndex].value = 0; 
            if (setupElement[menuIndex].value > 1) setupElement[menuIndex].value = 0; 
          break;
          case menuValueContrast:
            if (setupElement[menuIndex].value < 1) setupElement[menuIndex].value = 25; 
            if (setupElement[menuIndex].value > 25) setupElement[menuIndex].value = 1; 
          break;
          case menuValueAlarmEnabled:
            //if (setupElement[menuIndex].value < 0) setupElement[menuIndex].value = 0; 
            if (setupElement[menuIndex].value > 1) setupElement[menuIndex].value = 0; 
          break;
          case menuValueAlarmHour:
            //if (setupElement[menuIndex].value < 0) setupElement[menuIndex].value = 0; 
            if (setupElement[menuIndex].value > 23) setupElement[menuIndex].value = 0; 
          break;
          case menuValueAlarmMinute:
            //if (setupElement[menuIndex].value < 0) setupElement[menuIndex].value = 0; 
            if (setupElement[menuIndex].value > 59) setupElement[menuIndex].value = 0; 
          break;
          case menuValueAlarmDay:
            if (setupElement[menuIndex].value < 1) setupElement[menuIndex].value = 127; 
            if (setupElement[menuIndex].value > 127) setupElement[menuIndex].value = 1; 
          break;
          case menuValueAlarmMode:
            if (setupElement[menuIndex].value < 1) setupElement[menuIndex].value = 6; 
            if (setupElement[menuIndex].value > 6) setupElement[menuIndex].value = 1; 
          break;

        } // end switch menuIndex

        // Update Oled Contrast Value for a live change
        oledContrast = setupElement[menuValueContrast].value; 

        // Actual setupElement[x].value changed ? => Update only the Value
        setupElementValue_changed = setupElement[menuIndex].value != prev_setupElementValue?true:false;
        prev_setupElementValue = setupElement[menuIndex].value;
        
        if ( setupElementValue_changed && menuEditMode ) {
          oled.setCursor(menuValuePos,menuEditLine);
          sprintf(datebuffer, "%02u", setupElement[menuIndex].value);
          oled.print(datebuffer);
        }
      }

      if (!setupEditDone) {
        oled.clear();      
        //oled.setCursor(43,0);
        oled.print("<Setup>");
      } //endif setupEditDone

      // V2
      if ((menuIndex_changed  && !menuEditMode) || !setupEditDone) {
        // Show ">"
        oled.setCursor(0,menuEditLine);
        oled.print(">");

        for (int i=-2; i<=2; i++) {
          oled.setCursor(menuTextPos, menuEditLine + i);
          oled.clearToEOL();
          if ((menuIndex + i >= menuStart) && (menuIndex + i <= menuEnd)) {
          //if ((menuIndex + i > 0) && (menuIndex + i <= 11)) {
            oled.print(setupElement[menuIndex + i].name);
            if (getMenuEditMode(menuIndex + i)) {
              oled.setCursor(menuValuePos, menuEditLine + i);
              sprintf(datebuffer, "%02u", setupElement[menuIndex + i].value);
              oled.print(datebuffer);
            }  // endif getMenuEditMode
          }  // endif menuIndex
        }  // endfor
      }  // endif menuIndex_changed

      setupEditDone = true;

      // Little debugging and test "streaming"
      oled.setCursor(0,7); 
      oled << "isAlarmDay: " << isAlarmDay;
    break;

    case 105:
      oled.clear();
      if (menuIndex == menuValueSave) setState(106);  // Save
      if (menuIndex == menuValueExit) setState(108);  // Exit
    break;

    case 106:
      // Save Setup Values back which are changed
      oled.println("Saving Values..."); 
      
      // Save Time to RTC if something's changed
      if ( setupElement[menuValueHour].changed ||
	       setupElement[menuValueMinute].changed ||
           setupElement[menuValueDay].changed || 
           setupElement[menuValueMonth].changed || 
           setupElement[menuValueYear].changed ) {
             tmElements_t rtcAdjustTime;
             rtcAdjustTime.Hour = setupElement[menuValueHour].value; 
             rtcAdjustTime.Minute = setupElement[menuValueMinute].value;
             rtcAdjustTime.Second = 0;
             rtcAdjustTime.Day = setupElement[menuValueDay].value; 
             rtcAdjustTime.Month = setupElement[menuValueMonth].value; 
             rtcAdjustTime.Year = setupElement[menuValueYear].value +2000-1970;
             RTC.write(rtcAdjustTime);
      }

      // Save Alarm Hour
      if ( setupElement[menuValueAlarmHour].changed ) {
        EEPROM.write(7,setupElement[menuValueAlarmHour].value);      
      }
      
      // Save Alarm Minute
      if ( setupElement[menuValueAlarmMinute].changed ) {
        EEPROM.write(8,setupElement[menuValueAlarmMinute].value);      
      }

      // Save Alarm Day 
      if ( setupElement[menuValueAlarmDay].changed ) {
        EEPROM.write(9,setupElement[menuValueAlarmDay].value);      
      }
	  
      // Save Alarm Mode
       if ( setupElement[menuValueAlarmMode].changed ) {
        EEPROM.write(10,setupElement[menuValueAlarmMode].value);      
      }

      // Update RTC Alarm 1 Settings
      if ( setupElement[menuValueAlarmHour].changed ||
	         setupElement[menuValueAlarmMinute].changed ||
           setupElement[menuValueAlarmDay].changed || 
           setupElement[menuValueAlarmMode].changed ) {
		    RTC.setAlarm(getAlarmMode(setupElement[menuValueAlarmMode].value), 0, setupElement[menuValueAlarmMinute].value, setupElement[menuValueAlarmHour].value, setupElement[menuValueAlarmDay].value);
        // Clear Alarm1 flag
        RTC.alarm(ALARM_1);
      }

      // Save Alarm On/Off and disable/enable RTC Interrupt
      if ( setupElement[menuValueAlarmEnabled].changed ) {
        EEPROM.write(6,setupElement[menuValueAlarmEnabled].value);
        
        // Enable/Disable IRQ Handling if Alarm is enabled/disabled
        if ( setupElement[menuValueAlarmEnabled].value ) { 
          enableRTCInt();
        }
        else {
          disableRTCInt();
        }
      }  

      // Save Clock-Show-Time
      if ( setupElement[menuValueClocktime].changed ) {
        clocktime = setupElement[menuValueClocktime].value;
        EEPROM.write(2,setupElement[menuValueClocktime].value);
      }

      // Save WakeupMode
      if ( setupElement[menuValueMPUEnabled].changed ) {
        MPUEnabled = setupElement[menuValueMPUEnabled].value;
        EEPROM.write(4,setupElement[menuValueMPUEnabled].value);
        if (MPUEnabled) {
        }
        else {
        }
      }
      
      if ( setupElement[menuValueContrast].changed ) {
        oledContrast = setupElement[menuValueContrast].value; 
        EEPROM.write(5,setupElement[menuValueContrast].value);
      }
      setWaittime(3000);
      setState(109);
    break; // 106

    case 108:
      oled.println("Exit'd..."); // Just exited
      setWaittime(1000);
      setState(109);
    break; // 108
    
    case 109:
      if (waittimeover) setState(60);
    break;

// ------------------------- Stats Menu ------------------------------

    case 110:
      // Init
      init_view = true;
      oled.clear();
      
      // Draw Header
      //oled.setCursor(44,0);
      oled.print("<Stats>");

      oled.setCursor(6,2);
      oled.print("Uptime: ");
      
      // Show MPU Angles
      if ( !mpu.getSleepEnabled() ) {
        oled.setCursor(6,3);
        oled.print("Wakes: ");
        oled.setCursor(72,3);
        oled.print("Views: ");
        // Angles
        oled.setCursor(6,5);
        oled.print("AX:    AY:    AZ:");
      }
      
      // MPU enabled?
      if ( mpu.getSleepEnabled() ) {
        oled.setCursor(12,5);
        oled.print("MPU6050 disabled!");
      }
      setState(111);
    break;

    case 111:
      // Clear Runtime and Counter to 0 if B2 long pressed
      if (b2delayed) {
        startTime = RTC.get();
        counterWakeUp = 0;
        counterShowTime = 0;
      }
      
      // Show values updated each second
      if (sec_changed || init_view) {
        sprintf(datebuffer, "%02u:%02u:%02u", act_hr, act_min, act_sec);
        oled.setCursor(80,0);
        oled.print(datebuffer);
        
        //Show Uptime
        //sprintf(datebuffer, "%02u:%02u", hour(runTime), minute(runTime));
        sprintf(datebuffer, "%02u:%02u", int(runTime/3600), int((runTime/60)%60));
        oled.setCursor(54,2);
        oled.print(datebuffer);

        // MPU enabled?
        if ( !mpu.getSleepEnabled() ) {
          // Show counter
          oled.setCursor(48,3);
          oled.print(counterWakeUp);
          oled.setCursor(114,3);
          oled.print(counterShowTime);
          // Show Angles
          oled.setCursor(6,6);
          oled.print("       ");
          oled.setCursor(48,6);
          oled.print("       ");
          oled.setCursor(90,6);
          oled.print("       ");
          oled.setCursor(6,6);
          oled.print(arx);
          oled.setCursor(48,6);
          oled.print(ary);
          oled.setCursor(90,6);
          oled.print(arz);
        }
        // Actualize Battery values
        oled.setCursor(10,7);
        oled << "Battery " << voltage << "V " << percent << "%  ";
      } //endif sec changed
      init_view = false;
    break;

    case 112:
      setState(60);
    break;
  }
}

//------------------------------------------------------------
//----------------------- Functions --------------------------
//------------------------------------------------------------

// Wakeup Interrupt MPU
void wakeFromMPU() {
  // Let's show the time
  wokenFromMPU = true;
  setState(50);           // Check MPU Positions
}

// Wakeup Interrupt Button
void wakeFromBUTTON() {
  // Let's show the time
  wokenFromBUT = true;
  setState(60);
}

// RTC Alarm Interrupt
void wakeFromRTC() {
  wokenFromRTC = true;
}

// Checkposition
// x,y floats to be checked against the Min & Max values (z don't need to be checked)
bool checkposition(float x, float y, float xmin, float xmax, float ymin, float ymax) {
  if ((x >= xmin) && (x <= xmax) && 
      (y >= ymin) && (y <= ymax)) {
    return true;  // Position is OK
  }
  else {
    return false; // Position is NOK
  }
}

bool getMenuEditMode(uint8_t value) {
  if ((value >= menuValueStart) && (value <= menuValueEnd)) return true; 
  else return false;
}

bool getMenuFuncMode(uint8_t value) {
  if ((value == menuValueExit) || (value == menuValueSave)) return true; 
  else return false;
}

void setState(uint8_t value) {
  prevState = State;
  State = value;
}

void setWaittime(int value) {
  waitmillis = currentmillis;
  waittime = value;
}

void getBattery() {
  // Get Battery Values
  digitalWrite(BAT_EN, HIGH);
  delay(50);
  voltage = analogRead(BAT_PIN);
  voltage = (voltage / 1024) * 3.35;
  voltage = voltage / 0.5;
  delay(50);
  digitalWrite(BAT_EN, LOW);
  //Calculate Values
  percent = (voltage - 3.4) / 0.008;
  if (percent > 100){
    percent = 100;
  }
}

// function to return the compile date and time as a time_t value
time_t getCompileTime() {
    const time_t FUDGE(COMP_OFFSET);    //fudge factor to allow for upload time, etc. (seconds, YMMV)
    const char *compDate = __DATE__, *compTime = __TIME__, *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
    char compMon[3], *m;

    strncpy(compMon, compDate, 3);
    compMon[3] = '\0';
    m = strstr(months, compMon);

    tmElements_t tm;
    tm.Month = ((m - months) / 3 + 1);
    tm.Day =    atoi(compDate + 4);
    tm.Year =   atoi(compDate + 7) - 1970;
    tm.Hour =   atoi(compTime);
    tm.Minute = atoi(compTime + 3);
    tm.Second = atoi(compTime + 6);

    time_t t = makeTime(tm);
    return t + FUDGE;        //add fudge factor to allow for compile time
    //return t;
}

ALARM_TYPES_t getAlarmMode(uint8_t m) {
  switch (m) {
    case 1:
	    return ALM1_MATCH_SECONDS;  // Match Seconds (Once each Minute)
    break;
    case 2:
	    return ALM1_MATCH_MINUTES;  // ..and Minutes (Once each Hours)
    break;
    case 3:
	    return ALM1_MATCH_HOURS;    // ..and Hours (Once each Day)
    break;
    case 4:
	    return ALM1_MATCH_DATE;     // ..plus this Date or (Once each Month)
    break;
    case 5:
	    return ALM1_MATCH_DAY;      // ..plus this DayOfWeek (1-7) (Once each Week)
    break;
    case 6:
      return ALM1_MATCH_HOURS;    // Alarm once each Day but manually modified by sketch to support more than one day each week (Binary Matrix Son=Bit 0, Sat=Bit6)
    break;
    default:
      return ALM1_MATCH_HOURS;    // Same as 3 to prevent Compiler Warnings
    break;
  }
}

void enableRTCInt() {
  attachInterrupt(digitalPinToInterrupt(RTC_INT), wakeFromRTC, FALLING);  // Attach Interrupt Routine to MCU Pin 1
  RTC.alarmInterrupt(ALARM_1, true);                                         // Enable RTC IRQ
}

void disableRTCInt() {
  detachInterrupt(digitalPinToInterrupt(RTC_INT));                           // Detach Interrupt Routine to MCU Pin 1
  RTC.alarmInterrupt(ALARM_1, false);                                         // Disable RTC IRQ
}
