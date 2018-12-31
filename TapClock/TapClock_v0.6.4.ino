/*
    TapClock for WatchX
    The beginning...
	  The MPU6050 is (for me) a really complex device with lot's of registers and the possibilty to programm the internal DMP.
    I took the demo freefall sktech which uses the interrupt and changed it from freefall to motion, added a few values, thats all.
    Maybe I get sometime, after ten years of reading docs, gestures to work.

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
    -SSD1306 Text Library                   see https://github.com/greiman/SSD1306Ascii
    
    Libs not in use
    -Streaming by Mikal Hart                see https://github.com/geneReeves/ArduinoStreaming fork of Mikal Hart's streaming5.zip
    
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
    -Changed: Removed Delays from Intro and replaced with millis. LEDR glowing works now from the beginning
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
    Changed: Menuposition Exit and Save

    v0.6.3 (released)
    Added:   Array of "struct" containing values and a changed flag for the Setup Elements
             Only values changed during Setup are saved back to the "working" values.
    Added:   Watchface 3 BlockClock
    
    v0.6.4 (testing)
    Changed: Stats
             Reset Runtime and wake counters with URB
             Showing Battery Level & Voltage
    Added:   EEPROM reading and writing
             At setup() the EEPROM Byte 0 is read at first.
             If Byte 0 is not Decimal 97, initial values for Clocktime (40), Watchface (1), Wakeup Mode (1) and Contrast (20) 
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
             If you press Button URB or ULB longer than 1500ms the Values in the Setup menu changing 10x faster
    
    ToDo:   
    -Modify disabled function "check_seetings" to work with the new MPU Library
    -Power saving by sending more unneeded sensors to sleep or switch off
    -Nicer UI?
    -RTC Alarm, change of Setup Menu, Days (Mo-Su as Bits), Hrs, Minute
    -Reduce Sketch Size (Divider,check Vars)

    Font:
    This sketch uses an SSD1306ASCii compatible font file "Clock5x7.h".
    You have two possibities to add the font to the sketch:
    1) Add the font to the SSD1306ASCii Library and add the line #include "Clock5x7.h" to the library file "allFonts.h" to cover the font.
       The font must be part of the SSD1306ASCii Library file and folder structure.
    2) Add the font to this (or your sketch) by added an #include "Clock5x7.h" 
       and make the file "Clock5x7.h" available through the libraries or sketch folders.
       I keep the font file in libraries\OLEDFonts to use it more than once.

    State:
    0:     Start/Display On
    10-21: Intro Steps
    50-53: Steps before clock is shown, Position Detection
    60:    Prepare Watchface
    61:    TapClock Watchface
    62:    WordClock Watchface
    63:    BlockClock Watchface
    90:    Prepare System before Sleep, Oled/Led
    91:    Sleep and Wakeup / Power Modes / Interrupt / Sleep
    100:   Setup
	  110:   Stats

    EEPROM Bytes
    00: Fix Dec 97 for detecting initial Setup already done
    01: free
    02: Clocktime
    03: Watchface
    04: Wakeup Mode
    05: Contrast

    Runtime
    MPU Wakeup:    20-22hrs
    Button Wakeup: 48-50hrs

*/

#include <avr/sleep.h>             // Needed for Sleep Modes
#include <avr/power.h>             // Needed for Power Off System Resources
#include <Wire.h>                  // I2C
#include <MPU6050.h>               // MPU Library including Interrupt functions
#include "SSD1306Ascii.h"          // OLED Lib Main Part
#include "SSD1306AsciiSpi.h"       // OLED Lib SPI Part
#include "RTClib.h"                // Needed for the RTC Chip, no functions for HW Alarm
#include <SparkFun_MAG3110.h>      // Needed for MAG3110
#include <Bounce2.h>               // Needed to unbounce the Buttons
#include "PinChangeInterrupt.h"    // Needed for Button Interrupts
#include <EEPROM.h>                //
#include "Clock5x7.h"              // You need to have the file available in the Sketch or Library folders
//#include <Streaming.h>             // Very usefull library, can be used for oled << "Text" << variable << "Text2"


//#define BLE
#ifdef BLE
  #define BLE_CS      A2
  #define BLE_IRQ     0
  #define BLE_RST     A1
  #include <Adafruit_BLE.h>
  #include <Adafruit_BluefruitLE_SPI.h>
  //#include <Adafruit_BluefruitLE_UART.h>  // Not needed
  Adafruit_BluefruitLE_SPI ble(BLE_CS, BLE_IRQ, BLE_RST);
#endif

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
#define COMP_OFFSET 8     //Compile and Transfer Time Offset in secs
#define DebounceTime 25 // Button Debounce Time
#define Watchfaces  3

SSD1306AsciiSpi oled;
MPU6050         mpu(0x69);  //AD0 = 1
RTC_DS3231      rtc;
MAG3110         mag = MAG3110();

// Instances of Bounce objects
Bounce b1debouncer = Bounce(); 
Bounce b2debouncer = Bounce(); 
Bounce b3debouncer = Bounce(); 

unsigned long b2pressed = 0;
unsigned long b3pressed = 0;
bool b2delayed = false;
bool b3delayed = false;
int buttonDelay = 1500;


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

#define PROGRAM "TapClock v0.6.4"  // 15 Chars, Start oled.print at 19
//Debug or not Debug...
#define DEBUG 0
//#define DEBUG 1

volatile int State = 0;
volatile int prevState = 0;

//Display Variables
unsigned long previousclockmillis = 0;
unsigned long currentmillis = 0;

int clocktime = 40;               // in 1/10 sec
const int clockmultiplier = 100;  // clocktime Multiplier
bool ShowTime = false;
bool init_view;

//Date & Time
char datebuffer[10];
const char *months[13] = {"", "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
const char *days[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
int prev_min = 61, prev_sec = 61;  // 61 = to be sure the xxx_changed (see next line) booleans  will be true
bool min_changed = false, sec_changed = false; 

//Voltage Calc Vars
float R1 = 10000;
float R2 = 10000;
float vDivider;

//Interrupt Status, needed to clear Interrupt Bit
bool mpuintstatus = false;

//USB Power Detdction
bool usbConnected = false;
bool charging = false;

// Battery
float voltage;
int percent;

//ChargeLED(R) Vars
long int chargeledmillis = 0;
const int chargeledinterval = 50;
const int chargeledmaxvalue = 255;
const int chargeledminvalue = 10;
int chargeledfadevalue = chargeledminvalue; // !!Important to initialize the startvalue with minvalue!!
int chargeledfadesteps = 5;

//100ms Blinker Vars
long int blink100millis = 0;
const int blink100interval = 100;
bool blink100 = false;
bool prev_blink100 = false;
bool blink100_pos = false;  // Flanc

//250ms Blinker Vars
long int blink250millis = 0;
const int blink250interval = 250;
bool blink250 = false;

//500ms Blinker Vars
long int blink500millis = 0;
const int blink500interval = 500;
bool blink500 = false;
bool prev_blink500 = false;
bool blink500_pos = false;  // Flanc

//Remove delays from Intro
long int waitmillis = 0;
int waittime = 0;

// Clockpos/Waitpos (Z can be ignored)
const float xclockposmin = 15;
const float xclockposmax = 90;
const float yclockposmin = -30;
const float yclockposmax = 30;
const float zclockposmin = 15;
const float zclockposmax = 90;

const float xwaitposmin = -90;
const float xwaitposmax = -20;
const float ywaitposmin = -30;
const float ywaitposmax = 30;
const float zwaitposmin = 20;
const float zwaitposmax = 90;

const float xposdiff = 15;
const float yposdiff = 15;
const float zposdiff = 15;

DateTime startTime;

int counterWakeUp = 0, counterShowTime = 0;

// Menu Vars
// Setup-Element Structure
typedef struct {
  // I have tried to add an string "Name" here but i lost 5% storage
  int  value;
  bool changed;
} setupStructure;

const int numberOfSetupElements = 12;  // 12 = 0..11
setupStructure setupElement[numberOfSetupElements];

//                               0   1       2       3       4        5      6       7         8            9            10            11
const char* setupElementName[] ={"", "Exit", "Save", "Year", "Month", "Day", "Hour", "Minute", "Clocktime", "Watchface", "Wakeupmode", "Contrast" };

//int setupElementValue[11];               // 0..10 OLD
int prev_setupElementValue;
bool setupElementValue_changed;

const int menuStart = 1;                         // First Menu Value from Array
const int menuEnd = numberOfSetupElements - 1;   // Last Menu Value from Array
const int menuValueStart = 3;                    // First real Menu Entry from Array
const int menuValueEnd = menuEnd;                // Last real Menu Entry from Array
const int menuValueExit = 1;                     // menuValuesX as Numerics
const int menuValueSave = 2;
const int menuValueYear = 3;
const int menuValueMonth = 4;
const int menuValueDay = 5;
const int menuValueHour = 6;
const int menuValueMinute = 7;
const int menuValueClocktime = 8;
const int menuValueWatchface = 9;
const int menuValueWakeupMode = 10;
const int menuValueContrast = 11;
const int menuEditLine = 3;                     // This is the Line where the values are editable
int menuIndex = 1;                              // Start at 1 (actual Exit)
int prev_menuIndex = menuIndex;                 // Helper for menuIndex
bool menuIndex_changed = false;
bool menuEditMode  = false;
bool setupEditDone = false;
float xMenu = 0, yMenu = 0, zMenu = 0;

// Wakeupmode, Watchface, Oled Contrast
int wakeupMode, prev_wakeupMode;
int watchface, oledContrast;

// -------------------------------------------------------------------
// ---------------------------- Setup --------------------------------
// -------------------------------------------------------------------

void setup() 
{
  if (DEBUG) {
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo's serial, others continue immediately
  }

  // Init Display and Clear up
  oled.begin(&Adafruit128x64, OLED_CS, OLED_DC, OLED_RST);
  oled.setFont(System5x7);
  oled.clear();
  oled.println(F("OLED...OK"));

  // Set WatchX RTC time only if needed (Power off or uploading sketch with actual time but not on a normal reset)
  oled.print(F("RTC"));
  rtc.begin();
  oled.print(F("."));
  // Variant 1
  //if (rtc.lostPower()) {

  // Variant 2
  startTime = rtc.now();
  oled.print(F("."));
  DateTime compiledTime = DateTime(__DATE__, __TIME__);
  if (startTime.unixtime() < compiledTime.unixtime()) {   
    rtc.adjust(DateTime(__DATE__, __TIME__) + COMP_OFFSET);    // set Time, offset because of compile time
    oled.print(F("."));
    startTime = rtc.now();                                     // Re-read Start Time as it was changed
    oled.print(F("."));
  }
  //rtc.adjust(DateTime(__DATE__, __TIME__) + COMP_OFFSET);    // Offset because of compile time offset
  oled.println(F("OK"));


  // MPU Init and settings
  oled.print(F("MPU...."));
  /*
  mpu.reset();                                            // On SPI you need to reset ACCL/TEMP/GYRO Signal Path as well (Register 104)
  delay(100);                                             // See MPU Register Doku Register 107 
  */
  if (DEBUG) Serial.println(F("Initializing MPU..."));
  mpu.initialize();
  mpu.setAccelerometerPowerOnDelay(3);                    // MPU6050_DELAY_3MS
  mpu.setDHPFMode(MPU6050_DHPF_5);                        // MPU6050_DHPF_5HZ
  //mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);        // MPU Gyro_Config Register 1B/27, after init 250°/s
  //mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);        // MPU Accel_Config Register 1C/28, after init 2g
  mpu.setIntFreefallEnabled(false);                       // Disable Freefall Interrupt
  mpu.setIntZeroMotionEnabled(false);                     // Disable Zero Motion Interrupt
  mpu.setIntMotionEnabled(true);                          // Enable Motion Interrupt
  mpu.setInterruptMode(1);                                // Interrupt Mode; 1= active low / 0 = active high
  //mpu.setInterruptDrive(1);                              // open drain, not needed here
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
  //if (DEBUG) checkSettings(); // Send a few MPU settings to serial
  oled.println(F("OK"));

  // Init Compass MAG3300 and send it directly into standby
  oled.print(F("MAG...."));
  mag.initialize();
  mag.start();                  // Needed ??
  mag.enterStandby();
  oled.println(F("OK"));

#ifdef BLE
  // Trying to educe Bluetooth Power Consumption
  oled.print(F("BLE."));
  ble.begin();
  oled.print(F("."));
  ble.factoryReset();
  oled.print(F("."));
  ble.sendCommandCheckOK(F( "AT+GAPDEVNAME=WatchX" ));   // Change the name of the Ble device.
  oled.print(F("."));
  ble.sendCommandCheckOK(F( "AT+HWMODELED=DISABLE" ));   // Disabling this will reduce power consup.
  oled.print(F("."));
  ble.sendCommandCheckOK(F( "AT+UARTFLOW=OFF" ));        // Disabling this will also increase battery life
  oled.print(F("."));
  ble.sendCommandCheckOK(F( "AT+BLEPOWERLEVEL=-12" ));   // This sets broadcast power. It has a significant effect in battery life. Please see PAGE-95 for more detail
  oled.print(F("."));
  ble.sendCommandCheckOK(F( "AT+BLEMIDIEN=OFF" ));       // Disabling this will also help in battery life.
  oled.print(F("."));
  ble.sendCommandCheckOK(F( "AT+GAPSTOPADV" ));          // We can start and stop Bluetooth advertisement, this line will stop it.
  oled.print(F("."));
  ble.reset();
  oled.println(F("OK"));
#endif

  // Setup Hardware
  oled.print(F("In/Outputs..."));
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
  
  oled.println(F("OK"));

  // For voltage calulation
  vDivider = (R2 / (R1 + R2));
  
  // Enable VBUS Pad for detecting USB Power
  USBCON|=(1<<OTGPADE); 
   
  oled.print(F("EEPROM"));
  // Check EEPROM Byte 0
  // If it's a 97 the inital values are set before and no need to set them up again
  // Preset Values if 0 != 97
  if (EEPROM.read(0) != 97) { //
    EEPROM.write(0,97);       // Set Byte 0 to 97
    EEPROM.write(1,0);        // Clear Byte 1
    EEPROM.write(2,40);       // Clocktime [*100ms]
    EEPROM.write(3,1);        // Watchface
    EEPROM.write(4,1);        // Wakeup Mode
    EEPROM.write(5,20);       // Contrast [*10]
    oled.print(F("-Init-"));
  }
  // Get some values from EEPROM or set defaults
  // Clocktime 10-50 (*100)
  setupElement[menuValueClocktime].value = EEPROM.read(2);
  clocktime = setupElement[menuValueClocktime].value;
  oled.print(F("."));
  
  // Watchface 1-3
  setupElement[menuValueWatchface].value = EEPROM.read(3);
  watchface = setupElement[menuValueWatchface].value;
  oled.print(F("."));
  
  // Wakeup Mode 1-2
  setupElement[menuValueWakeupMode].value = EEPROM.read(4);
  wakeupMode = setupElement[menuValueWakeupMode].value;
  oled.print(F("."));
  prev_wakeupMode = wakeupMode;
  
  //Contrast 1-25 (*10)
  setupElement[menuValueContrast].value = EEPROM.read(5);
  oledContrast = setupElement[menuValueContrast].value;
  oled.print(F("."));
  oled.println(F("OK"));

  // Disable MPU if needed
  if ( wakeupMode == 2 ) mpu.setSleepEnabled(true);
  
  oled.println(F("Starting..."));
  delay(2500);
}

// -------------------------------------------------------------------
// ------------------------ Main Loop --------------------------------
// -------------------------------------------------------------------

void loop()
{
  // main loop variables
  int act_hr, act_min, act_sec, act_year, act_month, act_day, act_dayofweek, act_hr12;
  int hr10, hr1, min10, min1, sec2;
  unsigned char bat_hdrs, bat_tens, bat_ones;
  currentmillis = millis();

  //MPU Values
  int16_t ax=0, ay=0, az=0;
  float arx, ary, arz;
  bool readclockpos = false;
  bool every10secs = false;
  bool waittimeover = false, waitclockpos = false;
  bool menuLeft = false, menuRight = false, menuUp = false, menuDown = false;

  //Bouncer Values
  int b1, b2, b3;

  // Update the Bounce instances :
  b1debouncer.update();
  b2debouncer.update();
  b3debouncer.update();

  // Get the updated value from bounce:
  b1 = b1debouncer.read();
  b2 = b2debouncer.read();
  b3 = b3debouncer.read();

  // Button 2+3 delayed
  if (b2debouncer.fell()) b2pressed = currentmillis;
  b2delayed = !b2 & (currentmillis - b2pressed > buttonDelay);
  if (b3debouncer.fell()) b3pressed = currentmillis;
  b3delayed = !b3 & (currentmillis - b3pressed > buttonDelay);

  // Set Oled Contrast
  oled.setContrast(oledContrast * 10);

  DateTime now = rtc.now();
  act_hr    = now.hour();
  act_min   = now.minute();
  act_sec   = now.second();
  act_year  = now.year();
  act_month = now.month();
  act_day   = now.day();
  act_dayofweek = now.dayOfTheWeek();

  // Vars for WordClock
  act_hr12  = act_hr;         // 0..23
  if (act_hr12 > 12) act_hr12 -= 12;  // 0..23 => 0..12
  if (act_hr12 == 0) act_hr12 = 12;   // 0 = 12

  // Vars for BlockClock
  hr10  = act_hr / 10;
  hr1   = act_hr % 10;
  min10 = act_min / 10;
  min1  = act_min % 10;
  sec2  = act_sec % 2; 

  // Runtime
  unsigned long runTime = now.unixtime() - startTime.unixtime();  // get runtime in seconds

  // A tick each 10secs
  every10secs = ((act_sec % 10) == 0);
  
  // v2 Secs or Mins changed?
  min_changed = act_min != prev_min?true:false;
  sec_changed = act_sec != prev_sec?true:false;
  // Update prev's
  prev_min = act_min;
  prev_sec = act_sec;

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

  // Positive flanc blink500 v2
  blink100_pos = blink100 != prev_blink100?true:false;
  prev_blink100 = blink100;

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

  // Positive flanc blink500 v2
  blink500_pos = blink500 != prev_blink500?true:false;
  prev_blink500 = blink500;
    
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

  // Get MPU Accl Values
  mpu.getAcceleration(&ax, &ay, &az);

   // Calculate accelerometer angles
  arx = (180/3.141592) * atan(ax / sqrt(square(ay) + square(az))); 
  ary = (180/3.141592) * atan(ay / sqrt(square(ax) + square(az)));
  arz = (180/3.141592) * atan(sqrt(square(ay) + square(ax)) / az);
  
  // Check Positions, waitpos = position before clock can be shown
  waitclockpos = checkposition(arx, ary, arz, xwaitposmin, xwaitposmax, ywaitposmin, ywaitposmax, zwaitposmin, zwaitposmax);
  readclockpos = checkposition(arx, ary, arz, xclockposmin, xclockposmax, yclockposmin, yclockposmax, zclockposmin, zclockposmax);
  
  // waittimeover ?? in v2
  waittimeover = currentmillis - waitmillis >= waittime?true:false;

  // State changes
  // Intro steps, see states now
  
  // Jump from Intro to Clock, see states now

  // If System is up and timer is still running and clock is in the right position show time (60)
  // USB Power bypasses the position checks
  // From waitposition to readposition
  if ((State == 51) && ((!waittimeover && waitclockpos) || usbConnected)) setState(52);
  
  // From readposition to clock
  if ((State == 52) && ((!waittimeover && readclockpos) || usbConnected)) setState(60);
  
  // If timer is over without right position sleep again (90)
  if (((State == 51) || (State == 52)) && waittimeover) setState(90);
  
  // Jump to Power Display off
  if (((State == 61) || (State == 62) || (State == 63)) && ((currentmillis - previousclockmillis) >= (clocktime * clockmultiplier))) setState(90);
  
  // If we are on USB Power do not run the "Show Clock" Timer so update the "previous" with the actual millis eaach cycle
  if (((State == 61) || (State == 62) || (State == 63)) && usbConnected) {
    previousclockmillis = currentmillis;
  }
  
  // Setup Tools Starting at 100
  // Setup Menu/Clock
  if (((State == 61) || (State == 62) || (State == 63)) && b1debouncer.fell()) {
    setState(100);
  }
  // Setup Finished jump to save/exit
  if ((State == 101) && b1debouncer.fell() && getMenuFuncMode(menuIndex)) {
    setState(105);
  }
  
  // Jump to Stats
  if (((State == 61) || (State == 62) || (State == 63)) && b2debouncer.fell()) {
    setState(110);
  }
  // Get back from Stats
  if ((State == 111) && b1debouncer.fell()) {
    setState(112);
  }

  // If Wakeup Mode has changed enable/disable the MPU
  // 1= MPU Wakeup
  // 2= Button Wakeup
  // WakeupMode changed !
  if ( wakeupMode != prev_wakeupMode ) {
    // 1: Disable MPU sleep
    if ( wakeupMode == 1 ) {
      mpu.setSleepEnabled(false);
    }
    // 2: Send MPU to Sleep
    if ( wakeupMode == 2 ) {
      mpu.setSleepEnabled(true);      
    }
  }
  // Update WakeupMode
  prev_wakeupMode = wakeupMode;
  
  
  // -------------------------------------------------------------------
  // --------------------- State Switcher ------------------------------
  // -------------------------------------------------------------------
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
      setWaittime(2000);
      setState(11);
    break;
    
    case 11:
      // Wait for timer
      if (waittimeover) setState(12);
    break;
    
    case 12:
      oled.set2X();
      oled.setCursor(28,3);
      oled.print(F("WatchX"));
      oled.set1X();
      setWaittime(2500);
      setState(13);
    break;
    
    case 13:
      // Wait for timer
      if (waittimeover) setState(14);
    break;
    
    case 14:
      oled.setCursor(19,7);
      oled.print(F(PROGRAM));
      setWaittime(2000);
      setState(15);
    break;
 
    case 15:
      // Wait for timer
      if (waittimeover) setState(16);
    break;
    
    case 16:
      oled.setCursor(19,7);
      oled.print(F("Tap or Shake me"));
      setWaittime(2000);
      setState(17);
    break;
    
    case 17:
      // Wait for timer
      if (waittimeover) setState(20);
    break;
    
    
    case 20:
      oled.clear();
      setWaittime(1000);
      setState(21);
    break;  
    
    case 21:
      // Wait for timer
      if (waittimeover) setState(50);
    break;
   
    case 50:
      // Starting Watch
      // WakeupMode MPU
      switch (wakeupMode) {
        case 1:
          // MPU Wakeup Mode
          digitalWrite(LEDL, true);
          setWaittime(2500);
          counterWakeUp++;
          setState(51);
        break;
        // Button Wakeup Mode, jump straight to "WakeOn"
        case 2:
          setState(60);
        break;
      } // switch wakeupMode
    break;

    case 51:
      // Wait for "Wait-Position"
    break;

    case 52:
      // Wait the correct "Read-Position"
      digitalWrite(LEDL, blink250);
    break;

    case 60:
      // Power off LEDL
      digitalWrite(LEDL, false);
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
      //setState(61);

      // Jump to the Watchface
      switch (watchface) {
        case 1:
          setState(61);
        break;
        case 2:
          setState(62);
        break;
        case 3:
          setState(63);
        break;
      }  // switch Watchface
    break; // case 60

    // -------------------------------------------------------------------
    // -------------------- TapClock Watchface ---------------------------
    // -------------------------------------------------------------------
    case 61:
      // Show Time, Date and other Stuff
      // Show Hrs & Mins
      if (min_changed || init_view) {
        //Send Time to Display
        sprintf(datebuffer, "%02u:%02u", act_hr, act_min);
        oled.set2X();                           // maybe needs to be disabled if different hr font is used
        oled.setCursor(30,3);
        oled.print(datebuffer);
      } //endif min_changed)

      //oled.setFont(System5x7);                // only needed if different hr font is used
      // Show secs
      if (sec_changed || init_view) {
        sprintf(datebuffer, "%02u", act_sec);
        oled.set1X();
        oled.setCursor(91,3);
        oled.print(datebuffer);
      } //endif sec changed

      // Show Date & Uptime only at init or each min with usb connected
      if (init_view || (min_changed && usbConnected)) {
        // Show date
        oled.setCursor(18,0);
        oled.print(days[act_dayofweek]);  //0-6, 0=Sunday
        oled.print(" ");
        oled.print(act_day, DEC);
        oled.print(F("."));
        oled.print(months[act_month]);
        oled.print(F("."));
        oled.print(act_year, DEC);
        
      }
      
      // Show Battery only at init or every 10secs with usb connected
      if (init_view || (sec_changed && every10secs && usbConnected)) {
        // Get Battery Values
        getBattery();
        // Write battery Value to Display
        oled.setCursor(10,7);
        oled.print(F("Battery "));
        oled.print(voltage);
        oled.print(F("V "));
        oled.print(percent);
        oled.print(F("%"));
      } // endif Battery
      init_view = false;  //Init View done
    break;

    // -------------------------------------------------------------------
    // ------------------- WordClock Watchface ---------------------------
    // -------------------------------------------------------------------
    case 62:
      //Show WordClock at Init and every minute
      if (min_changed || init_view) showWordClock(act_hr12, act_min, act_sec);
      
      // Show Battery only at init or every 10secs with usb connected
      if ((sec_changed && every10secs && usbConnected) || init_view) {
        getBattery();
        showBatteryIcon(0,0); 

        bat_hdrs = percent/100;
        bat_tens = percent%100/10;
        bat_ones = percent%10;
        
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
        oled.print(F("%"));

        //Show AM/PM
        showAmPmIcon(122,0,act_hr);
      } // endif Show Battery
      init_view = false;  //Init View done
    break;

    // -------------------------------------------------------------------
    // ------------------ BlockClock Watchface ---------------------------
    // -------------------------------------------------------------------
    case 63:
      if (sec_changed || init_view) {
        showBlockNumber (0,   2, hr10);
        showBlockNumber (30,  2, hr1);
        showBlockNumber (73,  2, min10);
        showBlockNumber (103, 2, min1);

        //oled.setCursor(116,0);
        //sprintf(datebuffer, "%02u", act_sec);
        //oled.print(datebuffer);

        if (sec2) {
          showBlockDPoint (60, 3);
        }
        else {
          oled.clear(60, 64, 3, 5);
        } //endif sec2
      
        // Battery Icon
        getBattery();
        showBatteryIcon(0,0);
      }  // endif sec_changed
      
      if (min_changed || init_view) {
        // Show date
        oled.setCursor(18,0);
        oled.print(days[act_dayofweek]);  //0-6, 0=Sunday
        oled.print(" ");
        oled.print(act_day, DEC);
        oled.print(F("-"));
        oled.print(months[act_month]);
        oled.print(F("-"));
        oled.print(act_year, DEC);

      }  //endif min_changed
      init_view = false;  //Init View done
    break;  // BlockClock

    // Prepare System for Sleep
    case 90:
      // Prepare System for sleep
      digitalWrite(LEDL, false);
      // Clear Display on switch off
      oled.clear();
      oled.ssd1306WriteCmd(SSD1306_DISPLAYOFF);
      ShowTime = false;
      // Next Step => Sleep
      setState(91);
    break;

    // Sleep System
    case 91:
      // Send System into sleeeeeeeeeep....
      // Disable ADC System via PRR Register Bit 0 (PRADC)
      power_adc_disable();
      
      // Setup SleepMode , see https://www.gammon.com.au/forum/?id=11497
      //set_sleep_mode (SLEEP_MODE_IDLE);        // Wakeup Works
      //set_sleep_mode(SLEEP_MODE_EXT_STANDBY);  // Wakeup Works
      //set_sleep_mode(SLEEP_MODE_STANDBY);      //Wakeup Works
      set_sleep_mode (SLEEP_MODE_PWR_DOWN);      // Wakeup Works from v0.5.1
	  
	    // Enable Sleep Mode by setting SMCR Register Bit 0 and send the CPU to sleep
      sleep_enable();
      // Read here MPU Motion INT Status before we attach the Interrupt. The bit clears to 0 after the register has been read
      mpuintstatus = mpu.getIntMotionStatus();
      // Enable 32u4 MPU interrupt
      // WakeupMode MPU
      if ( wakeupMode == 1 ) {
        attachInterrupt(digitalPinToInterrupt(INT6_PIN), doInt, LOW);
      }
      // WakeupMode Button
      if ( wakeupMode == 2 ) {
        attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(BUTTON3), doInt, FALLING);
      }
      
      // Go sleeping....
      sleep_cpu();

      // ***************************************************
      // ******* The Code starts here after wakeup *********
      // ***************************************************

      // Disable sleep
      sleep_disable();
      // Disable 32u4 MPU interrupt
      detachInterrupt(digitalPinToInterrupt(INT6_PIN));
      detachPinChangeInterrupt(digitalPinToPinChangeInterrupt(BUTTON3));
      // Read MPU Motion Int Register to clear it; The bit clears to 0 after the register has been read
      //mpuintstatus = mpu.getIntMotionStatus();
    break;

    // -------------------------------------------------------------------
    // ------------------------- Setup Menu ------------------------------
    // -------------------------------------------------------------------
    case 100:
      //
      menuIndex = 1;
      
      // Save current MPU X/Y/Z Values
      xMenu = arx; 
      yMenu = ary;
      zMenu = arz;

      //Copy current Values into Array
      setupElement[menuValueYear].value       = act_year;
      setupElement[menuValueMonth].value      = act_month;
      setupElement[menuValueDay].value        = act_day;
      setupElement[menuValueHour].value       = act_hr;
      setupElement[menuValueMinute].value     = act_min;
      setupElement[menuValueClocktime].value  = clocktime;
      setupElement[menuValueWatchface].value  = watchface;
      setupElement[menuValueWakeupMode].value = wakeupMode;
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
      menuLeft =  checkposition(arx, ary, arz, xMenu-5, xMenu+5, yMenu-40, yMenu-10, -90, 90);
      menuRight = checkposition(arx, ary, arz, xMenu-5, xMenu+5, yMenu+10, yMenu+40, -90, 90);
      menuUp =    checkposition(arx, ary, arz, xMenu-40, xMenu-10, yMenu-5, yMenu+5, -90, 90);
      menuDown =  checkposition(arx, ary, arz, xMenu+10, xMenu+40, yMenu-5, yMenu+5, -90, 90);

      // Edit Mode on/off
      if ( b1debouncer.fell() && getMenuEditMode(menuIndex) && setupEditDone ) menuEditMode = !menuEditMode;
 
      // Show Active Menu Entry Inverted
      if ( b1debouncer.fell() && menuEditMode ) {
        oled.setInvertMode(true);
        oled.setCursor(6,menuEditLine);
        oled.print(setupElementName[menuIndex]);
        oled.setInvertMode(false);
      }
      // Show Active Entry Not Inverted
      if ( b1debouncer.fell() && !menuEditMode ) {
        oled.setCursor(6,menuEditLine);
        oled.print(setupElementName[menuIndex]);
      }

      // MenuIndex +/- by
      // Pressing Left Side Buttons shortly
      // MPU enabled rotating of Arm/WatchX
      if (!menuEditMode) {
        if ( b2debouncer.fell() || (menuUp && blink500_pos) ) menuIndex--;
        if ( b3debouncer.fell() || (menuDown && blink500_pos) ) menuIndex++;
      }

      // MenuIndex Bounds 
      if (menuIndex > menuEnd) menuIndex = menuStart;
      if (menuIndex < menuStart) menuIndex = menuEnd;

      // MenuIndex changed ?? => Update Menu & Cursor
      // v2 New version
      menuIndex_changed = menuIndex != prev_menuIndex?true:false;
      prev_menuIndex = menuIndex;
      
      // setupElement Value +/- by
      // Pressing Left Side Buttons shortly
      // MPU enabled rotating of Arm/WatchX
      // Pressing Left Side Buttons longer than 1500ms
      if ( menuEditMode && getMenuEditMode(menuIndex) ) {
        if (b2debouncer.fell() || (menuUp && blink500_pos) || (b2delayed && blink100_pos)) {
          setupElement[menuIndex].value++;
          setupElement[menuIndex].changed = true;
        }
        if (b3debouncer.fell() || (menuDown && blink500_pos) || (b3delayed && blink100_pos)) { 
          setupElement[menuIndex].value--;
          setupElement[menuIndex].changed = true;
        }

        switch(menuIndex) {
          case menuValueYear:
            if (setupElement[menuIndex].value < 2018) setupElement[menuIndex].value = 2040; 
            if (setupElement[menuIndex].value > 2040) setupElement[menuIndex].value = 2018; 
          break;
          case menuValueMonth:
            if (setupElement[menuIndex].value < 1) setupElement[menuIndex].value = 12; 
            if (setupElement[menuIndex].value > 12) setupElement[menuIndex].value = 1; 
          break;
          case menuValueDay:
            if (setupElement[menuIndex].value < 1) setupElement[menuIndex].value = 31; 
            if (setupElement[menuIndex].value > 31) setupElement[menuIndex].value = 1; 
          break;
          case menuValueHour:
            if (setupElement[menuIndex].value < 0) setupElement[menuIndex].value = 23; 
            if (setupElement[menuIndex].value > 23) setupElement[menuIndex].value = 0; 
          break;
          case menuValueMinute:
            if (setupElement[menuIndex].value < 0) setupElement[menuIndex].value = 59; 
            if (setupElement[menuIndex].value > 59) setupElement[menuIndex].value = 0; 
          break;
          case menuValueClocktime:
            if (setupElement[menuIndex].value < 1) setupElement[menuIndex].value = 60; 
            if (setupElement[menuIndex].value > 60) setupElement[menuIndex].value = 1; 
          break;
          case menuValueWatchface:
            if (setupElement[menuIndex].value < 1) setupElement[menuIndex].value = Watchfaces; 
            if (setupElement[menuIndex].value > Watchfaces) setupElement[menuIndex].value = 1; 
          break;
          case menuValueWakeupMode:
            if (setupElement[menuIndex].value < 1) setupElement[menuIndex].value = 2; 
            if (setupElement[menuIndex].value > 2) setupElement[menuIndex].value = 1; 
          break;
          case menuValueContrast:
            if (setupElement[menuIndex].value < 0) setupElement[menuIndex].value = 25; 
            if (setupElement[menuIndex].value > 25) setupElement[menuIndex].value = 0; 
          break;
        } // end switch menuIndex

        // Update Oled Contrast Value for a live change
        oledContrast = setupElement[menuValueContrast].value; 

        // Actual setupElement[x].value changed ? => Update only the Value
        setupElementValue_changed = setupElement[menuIndex].value != prev_setupElementValue?true:false;
        prev_setupElementValue = setupElement[menuIndex].value;
        
        if ( setupElementValue_changed && menuEditMode ) {
          oled.setCursor(80,menuEditLine);
          sprintf(datebuffer, "%02u", setupElement[menuIndex].value);
          oled.print(datebuffer);
        }
      }

      if (!setupEditDone) {
        oled.clear();      
        oled.setCursor(43,0);
        oled.print(F("<Setup>"));
      } //endif setupEditDone

      // V2
      if ((menuIndex_changed  && !menuEditMode) || !setupEditDone) {
        // ">"
        oled.setCursor(0,menuEditLine);
        oled.print(">");

        for (int i=-2; i<=2; i++) {
          oled.setCursor(6, menuEditLine + i);
          oled.clearToEOL();
          if ((menuIndex + i > 0) && (menuIndex + i <= 11)) {
            oled.print(setupElementName[menuIndex + i]);
            if (getMenuEditMode(menuIndex + i)) {
              oled.setCursor(80, menuEditLine + i);
              sprintf(datebuffer, "%02u", setupElement[menuIndex + i].value);
              oled.print(datebuffer);
            }  // endif getMenuEditMode
          }  // endif menuIndex
        }  // endfor
      }  // endif menuIndex_changed

      setupEditDone = true;

      // Little debugging and test "streaming"
      // oled.setCursor(0,7); oled << "Index:" << menuIndex << " ";
    break;

    case 105:
      oled.clear();
      if (menuIndex == menuValueSave) setState(106);  // Save
      if (menuIndex == menuValueExit) setState(108);  // Exit
    break;

    case 106:
      // Save Setup Values back which are changed, especially the time value
      oled.println(F("Saving Values:")); 
      oled.print(F("None!")); 
      oled.setCursor(0,1);
      
      // Save Time to RTC
      if ( setupElement[menuValueYear].changed || 
           setupElement[menuValueMonth].changed || 
           setupElement[menuValueDay].changed || 
           setupElement[menuValueHour].changed || 
           setupElement[menuValueMinute].changed ) {
             rtc.adjust(DateTime(setupElement[menuValueYear].value, setupElement[menuValueMonth].value, setupElement[menuValueDay].value, setupElement[menuValueHour].value, setupElement[menuValueMinute].value,0));
             oled.println(F("RTC...OK")); 
      }
 
      // Save Clock-Show-Time
      if ( setupElement[menuValueClocktime].changed ) {
        clocktime = setupElement[menuValueClocktime].value;
        EEPROM.write(2,clocktime);
        oled.println(F("Clocktime...OK")); 
      }

      // Save Watchface
      if ( setupElement[menuValueWatchface].changed ) {
        watchface = setupElement[menuValueWatchface].value;
        EEPROM.write(3,watchface);
        oled.println(F("Watchface...OK")); 
      }
        
      // Save WakeupMode
      if ( setupElement[menuValueWakeupMode].changed ) {
        wakeupMode = setupElement[menuValueWakeupMode].value;
        EEPROM.write(4,wakeupMode);
        oled.println(F("WakeupMode...OK")); 
      }

      if ( setupElement[menuValueContrast].changed ) {
        oledContrast = setupElement[menuValueContrast].value; 
        EEPROM.write(5,oledContrast);
        oled.println(F("Contrast...OK")); 
      }
      setWaittime(2500);
      setState(109);
    break; // 106

/*  
    case 107:
    break; // 107
*/

    case 108:
      oled.println(F("Exit'd...")); // Just exited
      setWaittime(1000);
      setState(109);
    break; // 108
    
    case 109:
      if (waittimeover) setState(60);
    break;

    // -------------------------------------------------------------------
    // ------------------------- Stats Menu ------------------------------
    // -------------------------------------------------------------------
    case 110:
      init_view = true;
      oled.clear();
      // Draw Header
      oled.setCursor(44,0);
      oled.print(F("<Stats>"));

      oled.setCursor(6,2);
      oled.print(F("Uptime:"));
      
      oled.setCursor(6,3);
      oled.print(F("Wakes: "));
      oled.print(counterWakeUp);
      oled.setCursor(72,3);
      oled.print(F("Views: "));
      oled.print(counterShowTime);
      
      if ( wakeupMode == 1 ) {
        // Angles
        oled.setCursor(6,5);
        oled.print(F("AX:    AY:    AZ:"));
      }
      
      // MPU enabled?
      if ( mpu.getSleepEnabled() ) {
        oled.setCursor(12,5);
        oled.print(F("MPU6050 disabled!"));
      }
      setState(111);
    break;

    case 111:
      // Clear Runtime and Counter to 0
      if (b2debouncer.fell()) {
        startTime = now;
        counterWakeUp = 0;
        counterShowTime = 0;
      }
      
      // Show values updated each second
      if (sec_changed || init_view) {

        //Show Uptime
        sprintf(datebuffer, "%03u:%02u", int(runTime/3600), int((runTime/60)%60));
        oled.setCursor(72,2);
        oled.print(datebuffer);

        // MPU enabled?
        if ( !mpu.getSleepEnabled() ) {
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
      } //endif sec changed

      if (min_changed || init_view) {
        // Get Battery Values
        getBattery();
        // Write battery Value to Display
        oled.setCursor(10,7);
        oled.print(F("Battery "));
        oled.print(voltage);
        oled.print(F("V "));
        oled.print(percent);
        oled.print(F("%"));
      }
      init_view = false;
    break;

    case 112:
      setState(60);
    break;
  }
}

//--------------- Functions ----------------------------------

// MPU Interrupt
void doInt()
{
  // Cancel sleep as a precaution here as well
  // Normal the code starts in state 7 with "sleep_disable()"
  sleep_disable();
  // Disable MPU interrupt again
  detachInterrupt(digitalPinToInterrupt(INT6_PIN));
  detachPinChangeInterrupt(digitalPinToPinChangeInterrupt(BUTTON3));
  // Let's show the time again
  //State = 50; 
  setState(50);
}

// Checkposition
// x,y,z floats to be checked against the Min & Max values
bool checkposition(float x, float y, float z, float xmin, float xmax, float ymin, float ymax,float zmin, float zmax)
{
  if ((x >= xmin) && (x <= xmax) && 
      (y >= ymin) && (y <= ymax) && 
      (z >= zmin) && (z <= zmax)) {
    return true;  // Position is OK
  }
  else {
    return false; // Position is NOK
  }
}

bool getMenuEditMode(int value) 
{
  if ((value >= menuValueStart) && (value <= menuValueEnd)) return true; 
  else return false;
}

bool getMenuFuncMode(int value) 
{
  if ((value == menuValueExit) || (value == menuValueSave)) return true; 
  else return false;
}

void setState(int value)
{
  prevState = State;
  State = value;
}

void setWaittime(int value)
{
  waitmillis = currentmillis;
  waittime = value;
}

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
      if (DEBUG) Serial.print("a Quarter ");
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

void showBlockDPoint (int x, int y) {
  // Save current font
  const uint8_t* currentFont = oled.font();
  
  // Set Clock5x7 Font
  oled.setFont(Clock5x7);

  oled.setCursor(x,y);
  oled.print("*");
  oled.setCursor(x,y+1);
  oled.print(" ");
  oled.setCursor(x,y+2);
  oled.print("*");

  // Set Font back
  oled.setFont(currentFont);
}

void showBlockNumber (int x, int y, int value) {
  // Save current font
  const uint8_t* currentFont = oled.font();
  
  // Set Clock5x7 Font
  oled.setFont(Clock5x7);
  
  switch (value) {
    case 0:
      oled.setCursor(x,y);
      oled.print("(**)");
      oled.setCursor(x,y+1);
      oled.print("*  *");
      oled.setCursor(x,y+2);
      oled.print("*  *");
      oled.setCursor(x,y+3);
      oled.print("*  *");
      oled.setCursor(x,y+4);
      oled.print("+**,");    
    break;
    case 1:
      oled.setCursor(x,y);
      oled.print("   *");
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
      oled.print("***,");
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
      oled.print("****");
      oled.setCursor(x,y+3);
      oled.print("   *");
      oled.setCursor(x,y+4);
      oled.print("+**,");    
    break;
  }
  // Set Font back
  oled.setFont(currentFont);
}

void getBattery() {
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
}

void showBatteryIcon(byte x, byte y) {
  // Save current font
  const uint8_t* currentFont = oled.font();

  // Set Clock5x7 Font
  oled.setFont(Clock5x7);
  
  oled.setCursor(x,y);
  if ((percent >=  0) && (percent < 25)) oled.print(char(35));
  if ((percent >= 25) && (percent < 50)) oled.print(char(36));
  if ((percent >= 51) && (percent < 75)) oled.print(char(37));
  if ((percent >= 75) && (percent < 100)) oled.print(char(38));
  if (percent == 100) oled.print(char(39));

  // Set Font back
  oled.setFont(currentFont);
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

/*
// Need to be fixed
void checkSettings()
{
  Serial.println();
  
  Serial.print(" * Sleep Mode:                ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");

  Serial.print(" * Motion Interrupt:     ");
  Serial.println(mpu.getIntMotionEnabled() ? "Enabled" : "Disabled");

  Serial.print(" * Zero Motion Interrupt:     ");
  Serial.println(mpu.getIntZeroMotionEnabled() ? "Enabled" : "Disabled");

  Serial.print(" * Free Fall Interrupt:       ");
  Serial.println(mpu.getIntFreeFallEnabled() ? "Enabled" : "Disabled");

  Serial.print(" * Free Fal Threshold:          ");
  Serial.println(mpu.getFreeFallDetectionThreshold());

  Serial.print(" * Free FallDuration:           ");
  Serial.println(mpu.getFreeFallDetectionDuration());
  
  Serial.print(" * Clock Source:              ");
  switch(mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }
  
  Serial.print(" * Accelerometer:             ");
  switch(mpu.getRange())
  {
    case MPU6050_RANGE_16G:            Serial.println("+/- 16 g"); break;
    case MPU6050_RANGE_8G:             Serial.println("+/- 8 g"); break;
    case MPU6050_RANGE_4G:             Serial.println("+/- 4 g"); break;
    case MPU6050_RANGE_2G:             Serial.println("+/- 2 g"); break;
  }  

  Serial.print(" * Accelerometer offsets:     ");
  Serial.print(mpu.getAccelOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getAccelOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getAccelOffsetZ());

  Serial.print(" * Accelerometer power delay: ");
  switch(mpu.getAccelPowerOnDelay())
  {
    case MPU6050_DELAY_3MS:            Serial.println("3ms"); break;
    case MPU6050_DELAY_2MS:            Serial.println("2ms"); break;
    case MPU6050_DELAY_1MS:            Serial.println("1ms"); break;
    case MPU6050_NO_DELAY:             Serial.println("0ms"); break;
  }  
  
  Serial.println();
}
*/
