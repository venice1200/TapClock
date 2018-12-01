/*
    TapClock for WatchX
    The beginning...
	  The MPU6050 is (for me) a really complex device with lot's of registers and the possibilty to programm the internal DMP.
    I took the demo freefall sktech which uses the interrupt and changed it from freefall to motion, added a few values, thats all.
    Maybe I get sometime, after ten years of reading docs, gestures to work.

    Based on:
    -WatchX libs by ArgeX                   see http://watchx.io/downloads_en.html
    -MPU6050 Lib from Korneliusz Jarzebski  see https://github.com/jarzebski/Arduino-MPU6050
    -BasicWatch 1 from kghunt/DntPMme       see https://github.com/kghunt/Basic_Watch
    -I2CDEVLIB                              see https://github.com/jrowberg/i2cdevlib 
    -Angle calculation                      see https://electronics.stackexchange.com/questions/142037/calculating-angles-from-mpu6050
    -Set Time at "Setup"                    see http://www.l8ter.com/?p=417
    -Nick Gammon Microprocessors Infos      see http://www.gammon.com.au/power and http://www.gammon.com.au/interrupts
    -Bounce2                                see https://github.com/thomasfredericks/Bounce2
    
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
     
    
    ToDo:   
    -Modify disabled function "check_seetings" to work with the new MPU Library
    -Power saving by sending more unneeded sensors to sleep or switch off
    -Nicer UI?

    State:
    0:     Start/Display On
    10-21: Intro Steps
    50-53: Steps before clock is shown, Position Detection
    60-61: Show Clock
    90:    Prepare System before Sleep, Oled/Led
    91:    Sleep and Wakeup / Power Modes / Interrupt / Sleep
    100:   Setup
	120:   Stats

    Button Pin 8  = links oben / upper left
    Button Pin 11 = rechts oben / upper right
    Button Pin 10 = rechts unten / lower right
*/

#include <avr/sleep.h>
#include <avr/power.h>
#include <Wire.h>
#include <MPU6050.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiSpi.h"
#include "RTClib.h"
#include <SparkFun_MAG3110.h>
//#include <Adafruit_BLE.h>
//#include <Adafruit_BluefruitLE_SPI.h>
//#include <Adafruit_BluefruitLE_UART.h>
#include <Bounce2.h>

SSD1306AsciiSpi oled;
MPU6050         mpu(0x69);  //AD0 = 1
RTC_DS3231      rtc;
MAG3110         mag = MAG3110();
//Adafruit_BluefruitLE_SPI ble(BLE_CS, BLE_IRQ, BLE_RST);

// Instances of Bounce objects
Bounce b1debouncer = Bounce(); 
Bounce b2debouncer = Bounce(); 
Bounce b3debouncer = Bounce(); 

#define VERSION "v0.5.7"
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
//#define SHOW_ANGLE  true
#define SHOW_ANGLE  false
#define DebounceTime 25 // Button Debounce Time


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

//Voltage Calc Vars
float R1 = 10000;
float R2 = 10000;
float vDivider;

//Interrupt Status, neede to clear Interrupt Bit
bool mpuintstatus = false;

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

// Clockpos/Waitpos
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

const float xposdiff = 5;
const float yposdiff = 30;
const float zposdiff = 5;

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
  //pinMode(INT6_PIN, INPUT);
  //attachInterrupt(digitalPinToInterrupt(INT6_PIN), doInt, RISING);
  //attachInterrupt(digitalPinToInterrupt(INT6_PIN), doInt, HIGH);

  pinMode(INT6_PIN, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(INT6_PIN), doInt, LOW);

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
  unsigned long runTime = now.unixtime() - startTime.unixtime();  // get runtime in seconds

  every10secs = ((act_sec % 10) == 0);
  
  //Secs or Mins changed?
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
  prev_min = act_min;
  prev_sec = act_sec;

  // LED Fader
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

  // Get MPU Accl Values
  mpu.getAcceleration(&ax, &ay, &az);

   // Calculate accelerometer angles
  arx = (180/3.141592) * atan(ax / sqrt(square(ay) + square(az))); 
  ary = (180/3.141592) * atan(ay / sqrt(square(ax) + square(az)));
  arz = (180/3.141592) * atan(sqrt(square(ay) + square(ax)) / az);
  
  // Check Positions, waitpos = position before clock can be shown
  waitclockpos = checkposition(arx, ary, arz, xwaitposmin, xwaitposmax, ywaitposmin, ywaitposmax, zwaitposmin, zwaitposmax);
  readclockpos = checkposition(arx, ary, arz, xclockposmin, xclockposmax, yclockposmin, yclockposmax, zclockposmin, zclockposmax);
  
  // waittimeover ??
  if (currentmillis - waitmillis >= waittime) {
    waittimeover = true; 
  }
  else {
    waittimeover = false;
  }

  // State changes
  // Intro steps, see states now
//  if ((State == 11) && waittimeover) setState(12);
//  if ((State == 13) && waittimeover) setState(14);
//  if ((State == 15) && waittimeover) setState(16);
//  if ((State == 17) && waittimeover) setState(18);
//  if ((State == 19) && waittimeover) setState(20);
  
  // Jump from Intro to Clock, see states now
//  if ((State == 21) && waittimeover) setState(50);

  // If System is up and timer is still running and clock is in the right position show time (60)
  // USB Poswer bypasses the position checks
  // From waitposition to readposition
  if ((State == 51) && ((!waittimeover && waitclockpos) || usbConnected)) setState(52);
  
  // From readposition to clock
  if ((State == 52) && ((!waittimeover && readclockpos) || usbConnected)) setState(60);
  
  // If timer is over without right position sleep again (90)
  if (((State == 51) || (State == 52)) && waittimeover) setState(90);
  
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
      if (waittimeover) setState(14);
    break;
    
    case 14:
      oled.set1X();
      oled.setCursor(18,7);
      oled.print(F("tap or shake me"));
      waitmillis = currentmillis;
      waittime = 2000;
      setState(15);
    break;
 
    case 15:
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
      oled.setCursor(16,1);
      oled.print(F("TapClock"));
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
      digitalWrite(LEDL, true);
      waitmillis = currentmillis;
      waittime = 2500;
      counterWakeUp++;
      setState(51);
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
      setState(61);
    break;

    case 61:
      // font testing
      //oled.setFont(Verdana_digits_24);
      //oled.setFont(Cooper26);                 //funny font
      
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
        //oled.setCursor(0,0);
        oled.setCursor(30,0);
        oled.print(act_day, DEC);
        oled.print(F("-"));
        oled.print(months[act_month-1]);
        oled.print(F("-"));
        oled.print(act_year, DEC);
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
      //set_sleep_mode (SLEEP_MODE_IDLE);        // Wakeup Works
      //set_sleep_mode(SLEEP_MODE_EXT_STANDBY);  // Wakeup Works
      //set_sleep_mode(SLEEP_MODE_STANDBY);      //Wakeup Works
      set_sleep_mode (SLEEP_MODE_PWR_DOWN);      // Wakeup Works from v0.5.1
	  
	    // Enable Sleep Mode by setting SMCR Register Bit 0 and send the CPU to sleep
      sleep_enable();
      // Read here MPU Motion INT Status before we attach the Interrupt. The bit clears to 0 after the register has been read
      mpuintstatus = mpu.getIntMotionStatus();
      // Enable 32u4 MPU interrupt
      attachInterrupt(digitalPinToInterrupt(INT6_PIN), doInt, LOW);
      // Go sleeping....
      sleep_cpu();

      // ***************************************************
      // ******* The Code starts here after wakeup *********
      // ***************************************************

      // Disable sleep
      sleep_disable();
      // Disable 32u4 MPU interrupt
      detachInterrupt(digitalPinToInterrupt(INT6_PIN));
      // Read MPU Motion Int Register to clear it; The bit clears to 0 after the register has been read
      //mpuintstatus = mpu.getIntMotionStatus();
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
      if ( b3debouncer.fell() && !menuEditMode ) menuIndex++;  // Button damaged
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
      //counterShowTime--;  // Adjust stats counter manually :-) no longer needed as of prevState
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
      
      // Angles
      oled.setCursor(6,5);
      oled.print(F("AX:    AY:    AZ:"));

      oled.setCursor(12,7);
      oled.print(F("Calibrate your MPU!"));
      setState(121);
    break;

    case 121:
      // Show values updated second
      if (sec_changed ) {

        //Show Uptime
        //sprintf(datebuffer, "%03u:%02u", int((runTime/3600)%24), int((runTime/60)%60));
        sprintf(datebuffer, "%03u:%02u", int(runTime/3600), int((runTime/60)%60));
        //sprintf(datebuffer, "%06u", runTime);
        oled.setCursor(72,2);
        oled.print(datebuffer);

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
      } //endif sec changed
    break;

    case 122:
      setState(60);
    break;
  }
}

//--------------- Functions ----------------------------------

// MOU Interrupt
void doInt()
{
  // Cancel sleep as a precaution here as well
  // Normal the code starts in state 7 with "sleep_disable()"
  sleep_disable();
  // Disable MPU interrupt again
  detachInterrupt(digitalPinToInterrupt(INT6_PIN));
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
