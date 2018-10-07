# TapClock
A WatchX Clock based on 
* WatchX libs provided by ArgeX
  * Especially the MPU6050 lib and Demo Code from Korneliusz Jarzebski
* BasicWatch v1 from kghunt
* OLED Library SSD1306Ascii by Greiman  
* Using Arduino 1.8.5 IDE

The Display is shown after tapping or shaking the clock.<br>
You can adjust the sensity by modify **mpu.setMotionDetectionThreshold** and/or **mpu.setMotionDetectionDuration** values<br>
<br>
The Display shows Date & Time, Uptime (right-top) and Battery level.  
  
v0.4.1
* Power saving using idle mode and disabling/enabling ADC  
  
v0.5.1
* Changed MPU6050 library to I2CDEVLIB https://github.com/jrowberg/i2cdevlib 
  * You need **I2Cdev** and **MPU6050** from https://github.com/jrowberg/i2cdevlib/tree/master/Arduino 
  * You need to remove the original MPU library
* Power saving using now 32u4's "SLEEP_MODE_PWR_DOWN". From my knowledge the highest power saving mode.
  * Modified interrupt handling needed for 32u4 and MPU6050 (latched interrupt)  
* Around 20-21hrs runtime
  * As of power saving "Uptime" shows the real 32u4 runtime instead of timed uptime.  
  * After 20hrs running my "Uptime" shows a value of 3 minutes :-)
<br>
<br>

Work in progress
