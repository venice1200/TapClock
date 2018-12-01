Readme for TapClock v0.5.7

Definitions: ULB = Upper Left Button (USB Side), URB/LRB = Upper/Lower Right Button

Libraries

    I2CDEVLIB including an MPU6050 library, search for “jrowberg/i2cdevlib” at Github
        You need I2Cdev and MPU6050 from the Arduino folder
        You need to remove the original MPU library as they share the same filename(s)
    Bounce 2, debouncing Buttons, search for “thomasfredericks/Bounce2” at Github
    SSD1306Ascii, OLED library, search for “greiman/SSD1306Ascii” at Github

Upload

    If TapClock runs you can upload code to the MCU only if the clock is awaken, so shake it :slight_smile:

Hints

    The Sketch needs MPU calibrating values which you can create with the WatchX_IMU_Zero_0x69.ino sketch avalible at my Github Repo.

Usage
If WatchX is connected to USB Power the System will never go to sleep if it’s awaken.
You can wait until the display is powerred off and then connect USB Power for charging without “display on”.
Charging is shown through an glowing right side Led.
If WatchX is on Battery, the system waits for 5 secs (const long clocktime = 5000;) then send the MCU into “SLEEP_MODE_PWR_DOWN”.
The Interrupt from the MPU is configurred for waking the MCU on shaking or tapping.
The motion values for shaking or tapping are actually fixed in sketch.
Search Setup for:

    //Modify the following for your needs
    mpu.setMotionDetectionThreshold(80); //80 
    mpu.setMotionDetectionDuration(4); //4

To get proper Angle values from the MPU you need to calibrate your MPU.
Search Setup for the below section and replace my values with your own calibration values:

    // Offsets got from IMU_Zero Sketch
    mpu.setXAccelOffset(106);
    mpu.setYAccelOffset(-397);
    mpu.setZAccelOffset(1255);
    mpu.setXGyroOffset(68);
    mpu.setYGyroOffset(-9);
    mpu.setZGyroOffset(29);

To show the time the system needs to be woken up.
After this the clock must be moved to two positions configured in the sketch to show the time (maybe i have to changed this).
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

How to get the tiem shown:

    Wake up the system by shaking watch
    Rotate watch in that way that clockface can’t be seen
    Rotate watch in that way that clockface can be seen

Now the time should be shown.
If you don’t do this within the clocktime the system goes back to sleep.

The Stats Menu can be used to show the needed angle values.

Time Menu

    If Clock is shown ULB opens the Menu for setting the time.
    URB and LRB Button move the “>” Cursor Up and Down (Rotating Cursor)
    Pick a value to modify with ULB, the Menu Entry will be inverted and values can be changed with URB/LRB, confirm with ULB
    Select Exit or Save and confirm with ULB to Save the time values to the RTC or Exit without saving

Stats Site

    If Clock is shown URB opens the Stats Window showing…
        Uptime after Reset/Restart in hrs:min (if RTC time is set correctly)
        Wakes and Shows to show how often the clock is woken up with/without showing time
        Angle values (+/-90°) calculated from Accelerometer (see sketch for details), calibration (!!) needed for correct values
        Exit with ULB

Compiling

    Sketch 21194 Bytes (73%)
    Global Variables 775 Bytes (30%)

See also the Readme on Github.

Cover
The cover is Transparent PLA Painted with Tamiya Color Spray.
Inspired by the Odroid-Go community on Reddit.

Credits
A WatchX Clock Sketch based on WatchX Hardware and:

    WatchX libs provided by ArgeX, especially the MPU6050 lib and Demo Code from Korneliusz Jarzebski
    OLED Library SSD1306Ascii by Greiman
    i2cdevlib/mpu6050 by jrowberg
    BasicWatch v1 from kghunt
    WatchX by Hackeitos
    Interrupt and Power Save Mode Information by Nick Gammon
    Arduino 1.8.5
    The WatchX Reddit Community

Cheers

P.S.Still working on Dokumentation
