# TapClock
Credits see end of Readme

The Display shows Date & Time, Battery level. Additional Values see releases.  

The Display is shown after tapping or shaking the clock.<br>
You can adjust the sensity by modify **mpu.setMotionDetectionThreshold** and/or **mpu.setMotionDetectionDuration** values<br>
Starting with v0.5.5 you have to define two watch positions (in the sketch) to show the clock. Therefore you need<br>
to calibrate your MPU with the WatchX_IMU_Zero_0x69.ino sketch from i2cdevlib/mpu6050 library or the uploaded version.<br>
<br>
  
Definitions: ULB = Upper Left Button (USB Side), URB/LRB = Upper/Lower Right Button<br>
  
v0.5.7  
Sketch needs MPU calibating values which you can create with the WatchX_IMU_Zero_0x69.ino sketch.  
  
Menu/Stats
* Time setting Menu
  * If clock is shown ULB opens the Menu for setting the time.
   * URB and LRB Button move the ">" Cursor Up and Down (Rotating Cursor)
   * Pick a value to modify with ULB, the Menu Entry will be inverted and values can be changed with ULB/LRB, confirm with ULB
   * Select Exit or Save and confirm with ULB to Save the time values to the RTC or Exit without saving
  
* Stats Site
  * If clock is shown URB opens the Stats Window showing
   * Uptime after Reset/Restart in hrs:min (if RTC time is set correctly)
   * Wakes and Shows to show how often the clock is woken up with/without showing the clock
   * Angle values calculated from Accelerometer (see sketch for details), calibration needed for correct values
   * Exit with ULB
  
v0.5.5
* Sketch needs MPU calibating values which you can create with the WatchX_IMU_Zero_0x69.ino from i2cdevlib/mpu6050 library
* After wakeing up you need to move the clock to two positions defined by min/max values inside the sketch before the clock is shown.
* Angle values can be shown with **#define SHOW_ANGLE  false/true**

v0.5.2
* Added USB Power detection  
* Charging is shown as a glowing right LED if the watch is "shaked or tapped on"  
* The Clock stays active if Usb Power if connected and don't go to sleep
* As long as the RTC has power the Clock is no longer set back to compile time on a device reset or upload  

v0.5.1
* Changed MPU6050 library to I2CDEVLIB https://github.com/jrowberg/i2cdevlib 
  * You need **I2Cdev** and **MPU6050** from https://github.com/jrowberg/i2cdevlib/tree/master/Arduino 
  * You need to remove the original MPU library
* Power saving using now 32u4's "SLEEP_MODE_PWR_DOWN". From my knowledge the highest power saving mode.
  * Modified interrupt handling needed for 32u4 and MPU6050 (latched interrupt)  
* Around 20-21hrs runtime
  * As of power saving "Uptime" shows now the **real 32u4 runtime** instead of timed uptime.  
  * After 20hrs running my "Uptime" shows a value of 3 minutes :-)  
  
v0.4.1  
* Power saving using idle mode and disabling/enabling ADC  
  
<br>
Credits  
A WatchX Clock based on WatchX Hardware and  
* WatchX libs provided by ArgeX, especially the MPU6050 lib and Demo Code from Korneliusz Jarzebski  
* OLED Library SSD1306Ascii by Greiman  
* i2cdevlib/mpu6050 by jrowberg  
* BasicWatch v1 from kghunt  
* WatchX by Hackeitos  
* Interrupt and Power Save Information by Nick Gammon  
* Arduino 1.8.5  
