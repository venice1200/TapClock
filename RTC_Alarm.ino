// Arduino DS3232RTC Library
// https://github.com/JChristensen/DS3232RTC
// Copyright (C) 2018 by Jack Christensen and licensed under
// GNU GPL v3.0, https://www.gnu.org/licenses/gpl.html
//
// DS3231/DS3232 Alarm Example Sketch #1
//
// Set Alarm 1 to occur once a minute at 5 seconds after the minute.
// Detect the alarm by polling the RTC alarm flag.
//
// Hardware:
// Arduino Uno, DS3231 RTC.
// Connect RTC SDA to Arduino pin A4.
// Connect RTC SCL to Arduino pin A5.
//
// Jack Christensen 16Sep2017

#include <DS3232RTC.h>      // https://github.com/JChristensen/DS3232RTC
#include <Streaming.h>      // http://arduiniana.org/libraries/streaming/
#include <avr/sleep.h>

//#define INT_PIN PD3
#define INT_PIN 1       // Pin 1/ INT3/TX1
//#define INT_PIN 7     // Pin 7/ INT6
#define LEDL 13
#define LEDR 6


volatile int count = 0;

void fint3(void) {
  count++;
}

void setup()
{
    Serial.begin(115200);
    //while (!Serial);
    //Serial1.end();
    
    // initialize the alarms to known values, clear the alarm flags, clear the alarm interrupt flags
    RTC.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);
    RTC.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);
    RTC.alarm(ALARM_1);
    RTC.alarm(ALARM_2);
    RTC.alarmInterrupt(ALARM_1, false);
    RTC.alarmInterrupt(ALARM_2, false);
    RTC.squareWave(SQWAVE_NONE);

    // set the RTC time and date to the compile time
    RTC.set(compileTime());

    // set Alarm 1 to occur at 5 seconds after every minute
    RTC.setAlarm(ALM1_MATCH_SECONDS, 30, 0, 0, 1);
    RTC.alarm(ALARM_1);                             // clear the alarm flag

    RTC.setAlarm(ALM2_EVERY_MINUTE, 0, 0, 1);
    RTC.alarm(ALARM_2);                             // clear the alarm flag
    //RTC.alarmInterrupt(ALARM_2, true);
    

    Serial << millis() << " Start ";
    printDateTime(RTC.get());
    Serial << endl;
    
    pinMode(LEDL, OUTPUT);
    pinMode(LEDR, OUTPUT);
    
    pinMode(INT_PIN, INPUT);
    //pinMode(INT_PIN, INPUT_PULLUP);
    //attachInterrupt(digitalPinToInterrupt(INT_PIN), fint3, LOW);
    //attachInterrupt(digitalPinToInterrupt(INT_PIN), fint3, FALLING);
    //attachInterrupt(digitalPinToInterrupt(INT_PIN), fint3, RISING);
    //attachInterrupt(digitalPinToInterrupt(INT_PIN), fint3, CHANGE);

}

void loop()
{
    delay(2000);
    for (int i=0; i<5; i++) {
      digitalWrite(LEDL, true);
      delay (500);
      digitalWrite(LEDL, false);
      delay (500);
    }
    
    set_sleep_mode (SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    RTC.alarm(ALARM_1);                    // Read and Clear RTC IRQ
    RTC.alarmInterrupt(ALARM_1, true);     // Enable IRQ
    attachInterrupt(digitalPinToInterrupt(INT_PIN), fint3, LOW);
    sleep_cpu();

    // ***************************************************
    // ******* The Code starts here after wakeup *********
    // ***************************************************

    // Disable sleep
    sleep_disable();
    // Disable 32u4 MPU interrupt
    detachInterrupt(digitalPinToInterrupt(INT_PIN));
    RTC.alarmInterrupt(ALARM_1, false);     // Enable IRQ
    RTC.alarm(ALARM_1);                     // Read and Clear RTC IRQ
      
/*    
    if ( RTC.alarm(ALARM_1) )    // check alarm flag, clear it if set
    {
        Serial << millis() << " ALARM_1 ";
        printDateTime(RTC.get());
        Serial << endl;
    }
 
    if ( RTC.alarm(ALARM_2) )    // check alarm flag, clear it if set
    {
        Serial << millis() << " ALARM_2 ";
        printDateTime(RTC.get());
        Serial << endl;
    }
    //delay(100);                  // no need to bombard the RTC continuously
*/

    for (int i=0; i<20; i++) {
      digitalWrite(LEDL, true);
      delay (100);
      digitalWrite(LEDL, false);
      delay (100);
      digitalWrite(LEDR, true);
      delay (100);
      digitalWrite(LEDR, false);
      delay (100);
    }

}

void printDateTime(time_t t)
{
    Serial << ((day(t)<10) ? "0" : "") << _DEC(day(t));
    Serial << monthShortStr(month(t)) << _DEC(year(t)) << ' ';
    Serial << ((hour(t)<10) ? "0" : "") << _DEC(hour(t)) << ':';
    Serial << ((minute(t)<10) ? "0" : "") << _DEC(minute(t)) << ':';
    Serial << ((second(t)<10) ? "0" : "") << _DEC(second(t));
    Serial << " Count: " << count;
    
}

// function to return the compile date and time as a time_t value
time_t compileTime()
{
    const time_t FUDGE(10);    //fudge factor to allow for upload time, etc. (seconds, YMMV)
    const char *compDate = __DATE__, *compTime = __TIME__, *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
    char compMon[3], *m;

    strncpy(compMon, compDate, 3);
    compMon[3] = '\0';
    m = strstr(months, compMon);

    tmElements_t tm;
    tm.Month = ((m - months) / 3 + 1);
    tm.Day = atoi(compDate + 4);
    tm.Year = atoi(compDate + 7) - 1970;
    tm.Hour = atoi(compTime);
    tm.Minute = atoi(compTime + 3);
    tm.Second = atoi(compTime + 6);

    time_t t = makeTime(tm);
    return t + FUDGE;        //add fudge factor to allow for compile time
}
