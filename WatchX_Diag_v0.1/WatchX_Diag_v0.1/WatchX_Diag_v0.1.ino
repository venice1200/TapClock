/*
  WatchX Diag v0.1
  Simple Sketch for checking WatchX Hardware
  Need only WatchX Libraries

  Checking
  -Oled, if you don't see anything there is something wrong with the OLED
  -Bluetooth Module, you should see two OK's. The first is for the Init, the second for an Factory Reset
  -i2c device scanner for NAG/RTC/MPU/BMP
  -LEDs
  -Buzzer
  -Buttons

  The Buzzer beeps at the end of the i2c scan and shows the result.
  If a Button is pressed the Oled show a short Text and the Buzzer do one beep.
  The LEDs blinking all the time.

  i2c Devices on WatchX
  0x0E MAG3110 Magnetometer
  0x68 DS3231  Real Time Clock
  0x69 MPU6050 Gyroscope/Accelerometer
  0x76 BMP280  Temperature/Pressure

  RAM Module on my RTC Breakout of my Test System
  0x57 AT24C0x 
*/

// Include Libs
#include <Wire.h>
#include "SSD1306Ascii.h"          // OLED Lib Main Part
#include "SSD1306AsciiSpi.h"       // OLED Lib SPI Part
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"

// Some Defines
#define OLED_DC      A3
#define OLED_CS      A5
#define OLED_RST     A4
#define BLE_CS       A2
#define BLE_IRQ      0
#define BLE_RST      A1 // Optional but recommended, set to -1 if unused
#define LEDL         13
#define LEDR         6
#define BUZZER       9
#define BUTTON1      8       // PullUp
#define BUTTON2      11      // PullUp
#define BUTTON3      10      // PullUp

// Create Objects
SSD1306AsciiSpi oled;
Adafruit_BluefruitLE_SPI ble(BLE_CS, BLE_IRQ, BLE_RST);

//500ms Blinker Vars
long int blink500millis = 0;
const int blink500interval = 500;
bool blink500 = false;

// Some Vars
bool i2c_checked = false;
bool ble_status = false;


void setup()
{
  // Init Display and Clear up
  oled.begin(&Adafruit128x64, OLED_CS, OLED_DC, OLED_RST);
  oled.setFont(System5x7);
  oled.clear();
  oled.println("WatchX Diag v0.1");

  oled.print("BLE ");
  if (ble.begin(false)) {
    oled.print("OK");
  }
  else {
    oled.print("ERROR");
  }
  oled.print(",");
  if (ble.factoryReset()) {
    oled.println("OK");
  }
  else {
    oled.println("ERROR");
  }

  // Start i2c
  Wire.begin();
 
  // Setup In- and Outputs
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);

  pinMode(LEDL, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  
  digitalWrite(LEDL, LOW);
  digitalWrite(LEDR, LOW);
  digitalWrite(BUZZER, LOW);
  
}
 
 
void loop() {
  // Needed for i2c scan
  byte error, address;
  // Needed for the blinker
  long int currentmillis = millis();
  
  // 500ms Blinker
  if (currentmillis - blink500millis > blink500interval) {
    blink500millis = currentmillis;
    blink500 = !blink500;
  }
  
  // LEDs blinking
  digitalWrite(LEDL, blink500);
  digitalWrite(LEDR, !blink500);
  
  // Checking Buttons 
  if (!digitalRead(BUTTON1)) {
    oled.setCursor(0,7);
    oled.print("B1:OK");
    tone(BUZZER, 500, 200);
  }
  if (!digitalRead(BUTTON2)) {
    oled.setCursor(40,7);
    oled.print("B2:OK");
    tone(BUZZER, 1000, 200);
  }
  if (!digitalRead(BUTTON3)) {
    oled.setCursor(80,7);
    oled.print("B3:OK");
    tone(BUZZER, 1500, 200);
  }
  
  // Checking i2x Devices, Code taken from the I2C Scanner sketch
  if (!i2c_checked) {
    for(address = 1; address < 127; address++ ) {
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
      if (error == 0)
      {
        if (address == 0x0E) oled.print("MAG");
        if (address == 0x57) oled.print("RTC-RAM");
        if (address == 0x68) oled.print("RTC");
        if (address == 0x69) oled.print("MPU");
        if (address == 0x76) oled.print("BMP");
        oled.print(" i2c 0x");
        if (address<16) 
        oled.print("0");
        oled.print(address,HEX);
        oled.println(" found");
      }
      else if (error==4) 
      {
        oled.print("Error at 0x");
        if (address<16) 
          oled.print("0");
        oled.println(address,HEX);
      }    
    }
    // i2c scanning finished
    i2c_checked = true;
    tone(BUZZER, 100, 200);
  }
}
