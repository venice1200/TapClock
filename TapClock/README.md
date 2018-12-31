**Readme for TapClock v0.6.4** (Happy new Year Edition)
All Infos for v0.6.3 are valid for v0.6.4 and:  
* Include "Clock5x7.h" Font file through the sketch or library System instead of modifying the SSD1306ASCii library.  
See HowTo here https://community.watchx.io/t/font-for-greiman-library-with-battery-symbols/150/4  
* EEPROM reading and writing of Setup Parameter  
If the EEPROM System was never used it will preset'ed during Setup()  
EEPROM stores Clocktime (40), Watchface (1), Wakeup Mode (1) and Contrast (20), defaults in brackets  
Only changed Setup Values are stored to EEPROM  
* Faster changing of Setup Values by holding the buttons pressed for more than 1500ms  
* Reset Uptime and Counter in Stats with URB  
* Re-Write Setup Menu Rotation  
* Some cosmetics
  
And see the release notes inside the sketch


**Readme for TapClock v0.6.3 / v0.6.2 (testing)**  
All Infos for v0.6.1 are valid for v0.6.3 and:  
Using the new Font "Clock5x7.h" (see fonts readme)  
Watchface 3 BlockClock

**Readme for TapClock v0.6.1**  
All Infos for v0.6.0 are valid for v0.6.1 and:  
Wakeup Mode added to Setup.  
Wakeup Mode 1: MPU based Wakeup (Standard).  
Wakeup Mode 2: LRB Button based Wakeup, like WordClock.  
If Wakeup Mode 2 is chosen the MPU is send to sleep, good for the battery. 

**Readme for TapClock v0.6.0**  
All Libraries v0.5.7 are also needed for v0.6.0 and  
* PinChange Interrupt library, search for "NicoHood" at GitHub  
  
All Infos for v0.5.7 are valid for v0.6.0 and:  
Usage  
The Menu contains two new values
* Watchface: 1=TapClock Watchface (Standard), 2=WordClock Watchface
* Wakeup: will come with v0.6.1
  
Gestures  
If your MPU is properly calibrated you can move the Setup Menu Cursor up and down  
and change values +/- by rotating your Arm/WatchX forwards and backwards.  
I save the current x/y/z values of the WatchX Position when the menu is opened  
and use these values with offsets to modify the cursor or the menu values.
  
**Readme for TapClock v0.5.7**

Definitions: 
* ULB = Upper Left Button (USB Side)
* URB = Upper Right Button
* LRB = Lower Right Button

Libraries
* I2CDEVLIB including an MPU6050 library, search for “jrowberg/i2cdevlib” at Github
  * You need I2Cdev and MPU6050 from the Arduino folder
  * You need to remove the original MPU library as they share the same filename(s)
* Bounce 2, debouncing Buttons, search for “thomasfredericks/Bounce2” at Github
* SSD1306Ascii, OLED library, search for “greiman/SSD1306Ascii” at Github

Upload
* If TapClock runs you can upload code to the MCU only if the clock is awaken, so shake it :-)

Hints
* The Sketch needs MPU calibrating values which you can create with the WatchX_IMU_Zero_0x69.ino sketch avalible at my Github Repo.

Usage  
If WatchX is connected to USB Power the System will never go to sleep if it’s "on".  
You can wait until the display is powerred off and then connect USB Power for charging without “display on”.  
Charging is shown with an glowing Right Led.  
If WatchX is on Battery, the system waits for 5 secs (const long clocktime = 5000;) then send the MCU into “SLEEP_MODE_PWR_DOWN”.  
The Interrupt from the MPU is configurred for waking the MCU on shaking or tapping.  
The motion values for shaking or tapping are actually fixed in sketch.  
  
Search Setup for:

    //Modify the following for your needs
    mpu.setMotionDetectionThreshold(80); //80 
    mpu.setMotionDetectionDuration(4); //4

To get proper Angle values from the MPU you need to calibrate your MPU.  
See "Calibration" below for more details.  

To show the time the system needs to be woken up.
After this the clock must be moved to two positions configured in the sketch to show the time (maybe i have to change this).
Search for:

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

How to get the time shown:
* Wake up the system by shaking the WatchX
* Rotate Arm/WatcX forward to the first position (clockface can’t be seen)
* Rotate Arm/WatcX backward to the second position (clockface can be seen)

Now the time should be shown.
If you don’t do this within the "Clocktime" the system goes back to sleep.

The Stats Menu can be used to show the needed angle values.

Setup Menu (Time Menu)
* If Clock is shown ULB opens the Menu for setting the time.
* URB and LRB Button moving the “>” Cursor Up and Down (Rotating Cursor)
* Pick a value to modify with ULB, the Menu Entry will be inverted and values can be changed with URB/LRB, confirm change with ULB
* Select Exit or Save and confirm with ULB to Save the changed values to the WatchX or Exit without saving

Stats Site
* If Clock is shown URB opens the Stats Window showing…
  * Uptime after Reset/Restart in hrs:min (if RTC time is set correctly)
  * Wakes and Shows to show how often the clock is woken up with/without showing time
  * Angle values (+/-90°) calculated from Accelerometer (see sketch for details), calibration (!!) needed for correct values
  * Exit with ULB

Calibration  
Run the calibration sketch available in the Repo.  
Copy the calibration values out of the serial window of the calibration sketch into the TapClock sketch  
and replace my values with yours as each MPU is different.

See the screeshot here which values are needed: https://i.imgur.com/FOmkXfg.png  
You have to choose one of the calibration values from each section.  
  
Replace the values for:

    // Offsets got from IMU_Zero Sketch
      mpu.setXAccelOffset(106);
      mpu.setYAccelOffset(-397);
      mpu.setZAccelOffset(1255);
      mpu.setXGyroOffset(68);
      mpu.setYGyroOffset(-9);
      mpu.setZGyroOffset(29);
  
After you have replaced my calibration values in the sketch and uploaded it the Stats Screen  
show nearly 0 at each axe if the WatchX lies on a flat and horizontal surface.  
See https://i.imgur.com/mNhEpCd.jpg
  
See the calibration sketch header as there are some more infos  
like heating up the MPU for 10 Minutes before you run the calibrating sketch  
and use a flat and horizontal surface for calibration.  
  
Compiling
* Sketch: v0.5.7: 73%, v0.6.1: 87%
* Global Variables: v0.5.7: 30%, v0.6.1: 42%
  
Cover  
The cover is Transparent PLA sprayed with Tamiya Color Spray.  
Inspired by the Odroid-Go community on Reddit.  
  
Credits  
A WatchX Clock Sketch based on WatchX Hardware and:
* WatchX libs provided by ArgeX, especially the MPU6050 lib and Demo Code from Korneliusz Jarzebski
* OLED Library SSD1306Ascii by Greiman
* i2cdevlib/mpu6050 by jrowberg
* BasicWatch v1 from kghunt
* WatchX by Hackeitos
* Interrupt and Power Save Mode Information by Nick Gammon
* PinChange Interrupt library by Nico Hood
* Arduino 1.8.5
* The WatchX Reddit Community
  
Cheers  
  
P.S. Still working on Dokumentation
