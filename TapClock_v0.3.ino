/*
    TapClock for WatchX
    The MPU6050 is (for me) a really complex device with lot's of registers and the possibilty to programm the internal DMP.
    I took the demo freefall sktech which uses the interrupt and changed it from freefall to motion, added a few values, thats all.
    Maybe I get sometime, after ten years of reading docs, gestures to work.

    v0.1 
    Based on
    -MPU6050 Lib and Demo Code from Korneliusz Jarzebski, see https://github.com/jarzebski/Arduino-MPU6050
    -BasicWatch 1 from kghunt, see https://github.com/kghunt/Basic_Watch
    -WatchX libs by ArgeX, see http://watchx.io/downloads_en.html
    -Show only Time in 4x Latters
    
    v0.2
    -Change OLED Library to SSD1306Ascii by Greiman to get (a lot) more Space, see https://github.com/greiman/SSD1306Ascii
    -Add "init_view" and "min_changed/sec_changed" booleans for updating screen values only if they are changed (see Note) or when the display is powered on
    -Add battery and date values to bisplay
    Note: If you update the display each "main loop" the chars on the display are flickering

    v0.3
    -Add Uptime
*/

#include <Wire.h>
#include <MPU6050.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiSpi.h"
#include "RTClib.h"

//#include <Adafruit_SleepyDog.h>

MPU6050 mpu;
RTC_DS3231 rtc;

#define DEBUG 0
//#define DEBUG 1

// Oled use hardware SPI
#define OLED_DC     A3
#define OLED_CS     A5
#define OLED_RESET  A4
#define BAT_PIN     A11
#define BAT_EN      4
#define BUZZER      9
#define INT6_PIN 7
#define LEDL 13
#define LEDR 6
#define Button1 8
#define Button2 11
#define Button3 10
#define SleepState 7

SSD1306AsciiSpi display;

boolean ShowTime = false;
int State = 0;

unsigned long previousMillis = 0;
const long interval = 5000;

char datebuffer[10];

float R1 = 10000;
float R2 = 10000;
float vDivider;
int prev_min = 61, prev_sec = 61;  // 61 = to be sure the xxx_changed (see nect line) vars will be true
bool min_changed = false, sec_changed = false, init_view = false;
const char *months[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

//--------------------------- Setup --------------------------------

void setup() 
{
  if (DEBUG) {
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
  }

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_16G))
  {
    if (DEBUG) Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  // Set the watch time (when uploading sketch)
  rtc.begin();
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  // Init Display and Clear up
  display.begin(&Adafruit128x64, OLED_CS, OLED_DC, OLED_RESET);
  display.setFont(System5x7);
  display.clear();

  // MOU Init and setting
  mpu.setAccelPowerOnDelay(MPU6050_DELAY_3MS);
  mpu.setIntFreeFallEnabled(false);
  mpu.setIntZeroMotionEnabled(false);
  mpu.setIntMotionEnabled(true);
  mpu.setDHPFMode(MPU6050_DHPF_5HZ);
  
  // Testing values
  // 20 80 500 250 125
  // TAP OK 250 125 (and lower), 800 war gut besser als 80 ??
  // Shake OK 125 (and lower)
  mpu.setMotionDetectionThreshold(80); //80 
  // 2 4 8 6
  // TAP OK 2 4 8
  // Shake OK 2 4 8
  mpu.setMotionDetectionDuration(4); //4
  
  if (DEBUG) checkSettings(); // Send MPU Seeting to Serial

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
  //INTs
  pinMode(INT6_PIN, INPUT);

  //Setup Interrupt
  attachInterrupt(digitalPinToInterrupt(INT6_PIN), doInt, RISING);

  // For voltage calulation
  vDivider = (R2 / (R1 + R2));
}

//--------------------------- Main Loop --------------------------------

void loop()
{
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

  //Has Sec or Min changed
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
  
  //Vector rawAccel = mpu.readRawAccel();
  Activites act = mpu.readActivites();

  if (DEBUG) {
    //Send Activity & State
    Serial.println(act.isActivity);
    Serial.print("State: ");
    Serial.println(State);
    //Show LEDR
    digitalWrite(LEDR, ShowTime);
  }

  // State changes
  // Timer to Power off Diplay after waking up
  if ((State == 5) && (currentMillis - previousMillis >= interval)) State = 6;
  

  // State Switcher
  switch (State) {
    case 0: // Intro :-) Just for Fun
      display.ssd1306WriteCmd(SSD1306_DISPLAYON);
      display.clear();
      display.set1X();
      display.setCursor(32,0);
      display.print("Welcome to");
      delay(3000);
      display.set2X();
      display.setCursor(26,3);
      display.print("WatchX");
      delay(2000);
      display.set1X();
      display.setCursor(18,7);
      display.print("tap or shake me");
      delay(3000);
      display.clear();
      delay (2000);
      State = 4;
    break;  

    case 4:
      previousMillis =  millis(); //Set Show Time Timer
      display.ssd1306WriteCmd(SSD1306_DISPLAYON);
      display.clear();
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
        //sprintf(datebuffer, "%02u", act_sec);
        display.set1X();
        display.setCursor(90,3);
        //display.print(datebuffer);
        display.print(act_sec);
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
        sprintf(datebuffer, "%03u:%02u", int((currentMillis/360000000)%24), int((currentMillis/60000)%60));
        display.setCursor(88,0);
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
        display.setCursor(8,7);
        display.print(F("Battery "));
        display.print(voltage);
        display.print(F("V "));
        display.print(percent);
        display.print(F("%"));
      } // endif init_view
      init_view = false;  //Init View done
    break;
    
    case 6:
      display.clear();
      display.ssd1306WriteCmd(SSD1306_DISPLAYOFF);
      ShowTime = false;
      State = 7;
    break;
    
    case 7:
      //Do nothing just wait
    break;
  }
  //delay(100);  
}

//--------------- Functions ----------------------------------

void doInt()
{
  State = 4; // Set State to Show Clock State
}

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
