WatchX Diag v0.3
Added i2c Detection of MLX Magnetometer on WatchX 1.3

WatchX Diag v0.2
Added Battery Voltage, USB Power and Charging Detection  
  
WatchX Diag v0.1
Simple Sketch for checking WatchX Hardware
Need only WatchX Libraries

Checking
* Oled, if you don't see anything there is something wrong with the OLED
* Bluetooth Module, you should see two OK's. The first is for the Init, the second for an Factory Reset
* i2c device scanner for MAG/RTC/MPU/BMP
* LEDs
* Buzzer
* Buttons
  
The Bluetooth Module is checked in "setup()".
The Buzzer beeps at the end of the i2c scan and shows the result.
If a Button is pressed the Oled shows a short Text and the Buzzer do one beep.
The LEDs blinking all the time.

i2c Devices on WatchX
0x0E MAG3110  Magnetometer on WatchX 1.2
0x19 MLX90393 Magnetometer on WatchX 1.3
0x68 DS3231   Real Time Clock
0x69 MPU6050  Gyroscope/Accelerometer
0x76 BMP280   Temperature/Pressure
