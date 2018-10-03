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
The Display shows Date & Time, Uptime (right-top) and Battery level.<br>
<br>

Work in progress

v0.4.1  
* Power saving using idle mode and disabling/enabling ADC

