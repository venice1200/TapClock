/*
    TapClock for WatchX
    The beginning...
	The MPU6050 is (for me) a really complex device with lot's of registers and the possibilty to programm the internal DMP.
    I took the demo freefall sktech which uses the interrupt and changed it from freefall to motion, added a few values, thats all.
    Maybe I get sometime, after ten years of reading docs, gestures to work.

    v0.1 
    Based on:
    -WatchX libs by ArgeX, see http://watchx.io/downloads_en.html
    -MPU6050 Lib and Demo Code from Korneliusz Jarzebski, see https://github.com/jarzebski/Arduino-MPU6050
    -BasicWatch 1 from kghunt/DntPMme, see https://github.com/kghunt/Basic_Watch
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
	-ToDo:   Modify disabled function "check_seetings" to work with the new MPU Library

   v0.5.1
   -Adding "disable_sleep();" after "sleep_cpu()" in State 7 for testing "SLEEP_MODE_PWR_DOWN"
    "SLEEP_MODE_PWR_DOWN" needs MPU setting "setInterruptLatch(1)" to latch the interrupt signal.
    If the MPU latch'es the interrupt you habe to clear the interrupt manually by reading the interrupt status usiung "[boolean] = getIntMotionStatus()".
    Reading the motion int bit clears it to 0.
    An non latched MPU Interrupt is only a 50us signal which seems to short to wake from "SLEEP_MODE_PWR_DOWN".
    
*/

#include <avr/sleep.h>
#include <avr/power.h>
#include <Wire.h>
#include <MPU6050.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiSpi.h"
#include "RTClib.h"

MPU6050 mpu(0x69);  //AD0 = 1
RTC_DS3231 rtc;

#define VERSION "v0.5.1"
//Debug or not Debug...
#define DEBUG 0
//#define DEBUG 1

// Oled use hardware SPI
#define OLED_DC     A3
#define OLED_CS     A5
#define OLED_RESET  A4
#define BAT_PIN     A11
#define BAT_EN      4
#define BUZZER      9
#define INT6_PIN    7
#define LEDL        13
#define LEDR        6
#define Button1     8
#define Button2     11
#define Button3     10

SSD1306AsciiSpi display;

volatile int State = 0;
boolean ShowTime = false;

unsigned long previousMillis = 0;
const long interval = 5000;

char datebuffer[10];

float R1 = 10000;
float R2 = 10000;
float vDivider;
int prev_min = 61, prev_sec = 61;  // 61 = to be sure the xxx_changed (see next line) booleans  will be true
bool min_changed = false, sec_changed = false, init_view = false;
const char *months[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

bool mpuintstatus = false;

//--------------------------- Setup --------------------------------

void setup() 
{
  if (DEBUG) {
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
  }

/*
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_16G))
  {
    if (DEBUG) Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
*/
  
  // Set the watch time (when uploading sketch)
  rtc.begin();
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  // Init Display and Clear up
  display.begin(&Adafruit128x64, OLED_CS, OLED_DC, OLED_RESET);
  display.setFont(System5x7);
  display.clear();

  // MPU Init and settings
  if (DEBUG) Serial.println(F("Initializing MPU..."));
  mpu.initialize();
  mpu.setAccelerometerPowerOnDelay(3);                    // MPU6050_DELAY_3MS
  mpu.setDHPFMode(MPU6050_DHPF_5);                        // MPU6050_DHPF_5HZ
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  mpu.setIntFreefallEnabled(false);                       // Disable Freefall Int
  mpu.setIntZeroMotionEnabled(false);                     // Disable Zero Motion Int
  mpu.setIntMotionEnabled(true);                          // Enable Motion Int
  mpu.setInterruptMode(1);                                // Interrupt Mode 1= active low / 0 = active high
  //mpu.setInterruptDrive(1);                              // open drain
  mpu.setInterruptLatch(1);                               // Interrupt Latch Mode 1 = signal until cleared / 0 = 50us signal
  
  //Modify the following for your needs
  mpu.setMotionDetectionThreshold(80); //80 
  mpu.setMotionDetectionDuration(4);   //4
  
  //if (DEBUG) checkSettings(); // Send a few MPU settings to serial

  // Setup Hardware
  // LEDs
  pinMode(LEDL, OUTPUT);
  pinMode(LEDR, OUTPUT);
  digitalWrite(LEDL, LOW);
  digitalWrite(LEDR, LOW);
  // Buttons
  pinMode(Button1, INPUT_PULLUP);
  pinMode(Button2, INPUT_PULLUP);
  pinMode(Button3, INPUT_PULLUP);

  //Interrupt on INT6/Pin7, moved to State 7
  //pinMode(INT6_PIN, INPUT);
  //attachInterrupt(digitalPinToInterrupt(INT6_PIN), doInt, RISING);
  //attachInterrupt(digitalPinToInterrupt(INT6_PIN), doInt, HIGH);

  pinMode(INT6_PIN, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(INT6_PIN), doInt, LOW);

  // For voltage calulation
  vDivider = (R2 / (R1 + R2));
}

//--------------------------- Main Loop --------------------------------

void loop()
{
  // loop vars
  int act_hr, act_min, act_sec, act_year, act_month, act_day;
  unsigned long currentMillis = millis();
  float voltage;
  int percent;
  
  DateTime now = rtc.now();
  act_hr    = now.hour();
  act_min   = now.minute();
  act_sec   = now.second();
  act_year  = now.year();
  act_month = now.month();
  act_day   = now.day();

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
  
/*
  if (DEBUG) {
    //Vector rawAccel = mpu.readRawAccel();
    Activites act = mpu.readActivites();
    //Send Activity & State
    Serial.println(act.isActivity);
    Serial.print("State: ");
    Serial.println(State);
    //Show LEDR
    digitalWrite(LEDR, ShowTime);
  }
*/

  // State changes
  // Change State from 5 to 6, timer to Power off Diplay after Interval
  if ((State == 5) && (currentMillis - previousMillis >= interval)) State = 6;
  

  // State Switcher
  switch (State) {
    case 0: 
      // Intro :-) Just for Fun
      display.ssd1306WriteCmd(SSD1306_DISPLAYON);
      display.clear();
      display.set1X();
      display.setCursor(32,0);
      display.print(F("Welcome to"));
      delay(3000);
      display.set2X();
      display.setCursor(26,3);
      display.print(F("WatchX"));
      delay(2000);
      display.set1X();
      display.setCursor(18,7);
      display.print(F("tap or shake me"));
      delay(3000);
      display.clear();
      delay(1000);
      display.set2X();
      display.setCursor(16,1);
      display.print(F("TapClock"));
      display.set1X();
      display.setCursor(46,5);
      display.print(F(VERSION));
      delay(2000);
      display.clear();
      delay (2000);
      State = 4;
    break;  

    case 4:
      //Set ShowTime Timer
      previousMillis =  millis();
      //Enable Display
      display.ssd1306WriteCmd(SSD1306_DISPLAYON);
      display.clear();
      //Enable ADC via PRR Register Bit 0 (PRADC)
      power_adc_enable(); 
      init_view = true;
      State = 5;
    break;

    case 5:
      ShowTime = true;
    
      // Show Hrs & Mins
      if (min_changed || init_view) {
        //Send Time to Display
        sprintf(datebuffer, "%02u:%02u", act_hr, act_min);
        display.set2X();
        display.setCursor(30,3);
        display.print(datebuffer);
      } //endif min_changed)
      
      // Show secs
      if (sec_changed || init_view) {
        sprintf(datebuffer, "%02u", act_sec);
        display.set1X();
        display.setCursor(91,3);
        display.print(datebuffer);
      } //endif sec changed

      // Show Date & Battery only once
      if (init_view) {
        // Show date
        display.setCursor(0,0);
        display.print(act_day, DEC);
        display.print(F("."));
        display.print(months[act_month-1]);
        display.print(F(" "));
        display.print(act_year, DEC);

        //Show Uptime
        sprintf(datebuffer, "%03u:%02u", int((currentMillis/3600000)%24), int((currentMillis/60000)%60));
        display.setCursor(92,0);
        display.print(datebuffer);
        
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
        display.setCursor(10,7);
        display.print(F("Battery "));
        display.print(voltage);
        display.print(F("V "));
        display.print(percent);
        display.print(F("%"));
      } // endif init_view
      init_view = false;  //Init View done
    break;
    
    case 6:
      // Clear Display on switch off
      display.clear();
      display.ssd1306WriteCmd(SSD1306_DISPLAYOFF);
      ShowTime = false;
      State = 7;
    break;
    
    case 7:
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
      // Read MPU Motion INT Status before we attach the Interrupt; The bit clears to 0 after the register has been read
      mpuintstatus = mpu.getIntMotionStatus();
      // Enable 32u4 MPU interrupt
      attachInterrupt(digitalPinToInterrupt(INT6_PIN), doInt, LOW);
      sleep_cpu();
      
      // The Code starts here after wakeup
      sleep_disable();
      // Disable 32u4 MPU interrupt
      detachInterrupt(digitalPinToInterrupt(INT6_PIN));
      // Read MPU Motion Int Register to clear it; The bit clears to 0 after the register has been read
      //mpuintstatus = mpu.getIntMotionStatus();
    break;
  }
}

//--------------- Functions ----------------------------------

void doInt()
{
  // cancel sleep as a precaution here as well
  // normal the code starts in state 7 after "sleep_disable()"
  sleep_disable();
  // Disable MPU interrupt again
  detachInterrupt(digitalPinToInterrupt(INT6_PIN));
  // Let's show the time
  State = 4; 
}

/*
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
