// Current Version
#define VERSION "SVTrackR v1.63 "

/*
 SVTrackR ( Arduino APRS Tracker )
 Copyright (C) 2014 Stanley Seow <stanleyseow@gmail.com>
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.

 github URL :-
 https://github.com/stanleyseow/ArduinoTracker-MicroAPRS

 This sketch configure the MicroAPRS for the proper callsign and ssid and
 read/write data coming in from the MicroAPRS via debug port

 Pin 0/1 (rx,tx) connects to Arduino with MicroAPRS firmware
 Pin 8,9 ( rx,tx ) connects to GPS module
 Pin 2,3 connect to debug serial port

 Pin 4 - Buzzer during Radio Tx

 Date : 03 July 2014
 Written by Stanley Seow
 e-mail : stanleyseow@gmail.com

 History :-
 03 July 2014 :-
 - Initial released

 06 July 2014 :-
 - added checks for speed and idle speed, modify the Txinternal
 - remove all debug Monitor output

 12 July 2014 :-
 - Added SmartBeaconing algorithm, Tx when turn is more than 25 deg
 - Reduce the Tx interval

 14 July 2014 :-
 - Fixed coordinates conversion from decimal to Deg Min formula
 - Formula to calculate distance from last Tx point so that it will Tx once the max direct distance
  of 500m is reached

 18 July 2014 :-
 - Fixed lastTx checking routine and ensure lastTx is 5 or more secs
 - Check for analog0 button at least 10 secs per Tx
 - Rewrote the DecodeAPRS functions for display to LCD only

 1 Aug 2014 :-
 - Ported codes to Arduino Mini Pro 3.3V
 - Due to checksum errors and Mini Pro 3.3V 8Mhz on SoftwareSerial, I swapped GPS ports to AltSoftwareSerial

 3 Aug 2014 :-
 - Added #define codes to turn on/off LCD, DEBUG and TFT
 - Added TFT codes for 2.2" SPI TFT ( runs on 3.3V )
 - Added the F() macros to reduce memory usages on static texts

 6 Aug 2014
 - Added Course/Speed/Alt into comment field readable by aprs.fi

 10 Aug 2014
 - Added support for teensy 3.1 (mcu ARM Cortex M4)

 19 Aug 2014
 - Added GPS simulator codes
 - Added Tx status every 10 Tx
 - Added counter for Headins, Time, Distance and Button

 19 Sep 2014
 - Modify button pressed to send STATUS & position
 - If GPS not locked, only sent out STATUS

 24 Sep 2014
 - Added support for BD GPS by modifying the tinyGPSPlus

 15 Oct 2014
 - Added setting PATH1 & PATH2 to the modem

 24 Oct 2014
 - Added blinking LED 13 during configuring the modem and a beep when completed
 - Minor changes in Status codes

 18 Nov 2014 ( v0.9 )
 - Adding OLED text only display to the Tracker
 - Moving LED to pin 6 as OLED used up SPI pins of 13,11,10,7
 - Switch top 2 lines from gps info to tracker info ( packet rx,tx)

 1 Dec 2014 ( v1.0 )
 - Added mic-e/compressed packets decoding from MicroAPRS libs
 - Ability to display received messages
 24 Dec 2014 ( v1.1 )
 - Added comment parsing codes from latest MicroAPRS libs
 - Added status ">" type parsing

 20 Jan 2015 ( v1.2 )
 - Modify Tx position before Tx status with 3 secs delay

 25 Jan 2015 ( V1.3 )
 - Added smartDelay for Tx status to display any received packets during the smartDelay period
 - Added 2 short beeps if Rx station is within 500m away

  15 Mar 2015 ( v1.4 - lamaral )
 - Fixed the hardcoding of the North and East on the GPS position
 - Added custom comment on APRS packet
 - Appended "0" if Lat/Lng is less than 10 deg

  23 Mar 2015 ( v1.5 - lamaral )
 - Fixed the problem of appending a zero to negative coordinates
 - Created the beep(int) function to save some memory

  07 Apr 2015 ( v1.5a - stanleyseow )
  - fixed the Lontitide with hard coded 0, will give coordinates errors if Lontitide is above 100 deg
  - fixed OLED display for above errors

  24 Apr 2015 ( v1.6 - stanleyseow )
  - append recevied callsign and distance to the end of comment
  - fixes some txTimer and lastTx codes
  - changed TxtoRadio return value to boolean and reason for Tx trigger
  - removed unused variables

  4 May 2015 ( v1.61 stanleyseow )
  - Change OLED callsign textsize 2
  - single page packet decode
  - minor changes to comment strings

  31 Aug 2015 ( V1.62 stanleyseow )
  - Adding support for I2C 16x2 for Dashboard LCD
  - Added 3 screens for speed/sats/headings, date/pos and other stats
  - Added N/E/S/W into main screen

    11 Apr 2016 ( V1.63 stanleyseow )
  - Changed to 500km away station calculations
  - Line 657
  - rev13 PCB will change to pullup circuit and read using digitalRead() instead of analogRead
*/

#include "config.h"

// GPS libraries
#include <TinyGPS++.h>
TinyGPSPlus gps;

// The packet decoding libs
#include <MicroAPRS.h>

#if defined(ARDUINO_AVR_LEONARDO) || defined(ARDUINO_AVR_A_STAR_32U4)
#define MODEM_SERIAL Serial1
#else
#define MODEM_SERIAL Serial
#endif

MicroAPRS microaprs = MicroAPRS(&MODEM_SERIAL);
// APRS Buffers
#define BUFLEN 260
char packet[BUFLEN];
int buflen = 0;
bool showmsg, showstation;

float latitude = 0.0;
float longitude = 0.0;
float wayPointLatitude, wayPointLongitude;
float latitudeRadians, wayPointLatitudeRadians, longitudeRadians, wayPointLongitudeRadians;
float distanceToWaypoint, bearing, deltaLatitudeRadians, deltaLongitudeRadians;
const float pi = 3.14159265;
const int radiusOfEarth = 6371; // in km

// AltSoftSerial default on UNO is 8,9 (Rx,Tx)
#if defined (__AVR_ATmega328P__)
#include <AltSoftSerial.h>
AltSoftSerial ss(8, 9);
#elif defined(ARDUINO_AVR_LEONARDO) || defined(ARDUINO_AVR_A_STAR_32U4)
#include <AltSoftSerial.h>
AltSoftSerial ss(13, 5);
#else
// Map hw Serial2 to ss for gps port for other platform with hw serial
#define ss Serial2
#endif

#if defined (__AVR_ATmega328P__)
#include <SoftwareSerial.h>
SoftwareSerial radio_serial(2, 3);     // RX, TX
#elif defined(ARDUINO_AVR_LEONARDO) || defined(ARDUINO_AVR_A_STAR_32U4)
#define radio_serial Serial
#elif defined(__arm__) && defined(TEENSYDUINO)
#define radio_serial Serial3
#endif

#ifdef DEBUG
#define debug radio_serial
#endif

#define BUZZER_ON() if (buzzerPin >= 0) { digitalWrite(buzzerPin, HIGH); }
#define BUZZER_OFF() if (buzzerPin >= 0) { digitalWrite(buzzerPin, LOW); }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Put All global defines here
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

const byte buzzerPin = -1;
const byte ledPin = 13;

#include "i2c_lcd.h"

// Defines for OLED 128x64
#ifdef OLED
#include <SPI.h>
#include <SSD1306_text.h>
#include <stdlib.h>
#ifdef OLED_I2C
SSD1306_text oled(0x3c, -1);
#else
// Hardware SPI pins include D11=Data and D13=Clk
// DO = CLK, 13
// DI = MOSI, 11
#define OLED_DC 7 // OLED Label DC
#define OLED_CS 0 // OLED Not used
#define OLED_RST 10 // OLED Label RST
SSD1306_text oled(OLED_DC, OLED_RST, OLED_CS);
#endif
#endif

// Varables for Packet Decode
const unsigned int MAX_INPUT = 103;
static unsigned int packetDecoded = 0;

char *lastCall = "";
String rxCallsign = "";
unsigned int rxStation;

unsigned int mCounter = 0;
unsigned int txCounter = 0;
unsigned long txTimer = 0;
#ifdef I2C16X2
bool packetDisplay = 0;
unsigned long displayTime = 0;
#endif
unsigned long lastTx = 0;
unsigned long lastRx = 0;
unsigned long txInterval = 80000L;  // Initial 80 secs internal

int lastCourse = 0;
byte lastSpeed = 0;
byte buttonPressed = 0;

int previousHeading, currentHeading = 0;
// Initial lat/lng pos, change to your base station coordnates
float lastTxLat = HOME_LAT;
float lastTxLng = HOME_LON;
float lastTxdistance, homeDistance, base = 0.0;

// Used in the future for sending messages, commands to the tracker
const unsigned int MAX_DEBUG_INPUT = 30;

//////////////////////////////////////////////////////////////////////////////
// setup()
//////////////////////////////////////////////////////////////////////////////
void setup() {
#ifdef I2C16X2
    lcd_setup(VERSION);
    delay(1000);
#endif

#ifdef OLED
    oled.init();
    oled.clear();                 // clear screen
    oled.setTextSize(1, 1);       // 5x7 characters, pixel spacing = 1
    oled.setCursor(0, 0);         // move cursor to row 1, pixel column 100
    oled.write(VERSION);
    oled.setCursor(1, 0);         // move cursor to row 1, pixel column 100
    oled.write("by APRS Studio");
    delay(2000);
#endif

    delay(4000);
    radio_serial.begin(9600);
    delay(30);
    radio_serial.println("AT+DMOCONNECT");
    delay(20);
    oled.setCursor(2, 0);
    while (radio_serial.available() > 0) {
        oled.write(radio_serial.read());
    }
    delay(20);
    
    // AT+DMOSETGROUP=
    //  1,          - bandwith in KHz ( 0 = 12.5KHz or 1 = 25KHz )
    //  144.3900,   - tx frequency in MHz (134.0000 - 174.0000)
    //  144.3900,   - rx frequency in MHz (134.0000 - 174.0000)
    //  0000,       - tx_ctcss frequency ( 0000 - 0038 ); 0000 = "no CTCSS"
    //  3,          - squelch level  ( 0 - 8 ); 0 = "open"
    //  0000        - rx_ctcss frequency ( 0000 - 0038 ); 0000 = "no CTCSS"
    // radio_serial.println("AT+DMOSETGROUP=0,144.3875,144.3875,0000,2,0000");
    radio_serial.println("AT+DMOSETGROUP=0,144.3900,144.3900,0000,1,0000");
    delay(20);
    oled.setCursor(3, 0);
    while (radio_serial.available() > 0) {
        oled.write(radio_serial.read());
    }
    delay(20);

    radio_serial.println("AT+DMOSETVOLUME=6");
    delay(20);
    oled.setCursor(4, 0);
    while (radio_serial.available() > 0) {
        oled.write(radio_serial.read());
    }
    delay(20);

    radio_serial.println("AT+SETFILTER=0,0,0");
    delay(20);
    oled.setCursor(5, 0);
    while (radio_serial.available() > 0) {
        oled.write(radio_serial.read());
    }
    delay(20);
    
    radio_serial.end();
    delay(20000);

#if defined(__arm__) && defined(TEENSYDUINO)
    // This is for reading the internal reference voltage
    analogReference(EXTERNAL);
    analogReadResolution(12);
    analogReadAveraging(32);
#endif

    if (buzzerPin >= 0) {
        pinMode(buzzerPin, OUTPUT);
    }

    pinMode(ledPin, OUTPUT);

    // Main serial talks to the MicroModem directly
    MODEM_SERIAL.begin(9600);

    // ss talks to the GPS receiver at 9600
    ss.begin(9600);

    
#ifdef DEBUG
    debug.begin(9600);

    debug.flush();
    debug.println();
    debug.print(F("DEBUG:- "));
    debug.println(F(VERSION));
    debug.println();
#endif

    // Set a delay for the MicroAPRS to boot up before configuring it
    delay(1000);
    configModem();

    MODEM_SERIAL.flush();

    txTimer = millis();

#ifdef OLED
    oled.clear();
    oledLine1();
#endif
} // end setup()

//////////////////////////////////////////////////////////////////////////////
// loop()
//////////////////////////////////////////////////////////////////////////////

void loop() {
    // Speed in km/h
    const byte highSpeed = 80;       // High speed
    const byte lowSpeed = 30;        // Low speed
    char c;
    boolean inputComplete = false;
    int headingDelta = 0;

#ifdef DEBUG
    // Send commands from debug serial into hw Serial char by char

#if defined (__AVR_ATmega328P__)
    debug.listen();
#endif
#endif

    // Turn on listen() on GPS
#if defined (__AVR_ATmega328P__)
    ss.listen();
#endif

    while (ss.available() > 0) {
        gps.encode(ss.read());
    }

    smartDelay(-1);

    ///////////////// Triggered by location updates ///////////////////////
    if (gps.location.isUpdated()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        
        homeDistance = TinyGPSPlus::distanceBetween(
            latitude, longitude,
            HOME_LAT, HOME_LON);

        lastTxdistance = TinyGPSPlus::distanceBetween(
            latitude, longitude,
            lastTxLat, lastTxLng);

        base = TinyGPSPlus::distanceBetween(
            latitude, longitude,
            HOME_LAT, HOME_LON) / 1000;

         // Get headings and heading delta
         currentHeading = (int)gps.course.deg();

         if (currentHeading >= 180) {
             currentHeading = currentHeading - 180;
         }

         headingDelta = (int)(previousHeading - currentHeading) % 360;
    } // endof gps.location.isUpdated()

    ///////////////// Triggered by time updates ///////////////////////
    // Update LCD every second

    if (gps.time.isUpdated()) {
        double gps_speed = gps.speed.kmph();
        uint16_t seconds = gps.time.second();
        
        if (gps.satellites.value() > 3) {
            digitalWrite(ledPin, HIGH);
        }
        else {
            digitalWrite(ledPin, LOW);
        }

#ifdef OLED
        if (seconds % 10 == 0) {
            oledLine2();
        }
        else if (seconds % 5 == 0) {
            oledLine1();
        }
#endif

#ifdef I2C16X2
        // I2C LCD addons
        if (packetDisplay) {
            // Do not overwrite the decoded packet
            // display packetDecoded for 10 secs
            if (millis() - displayTime > 10000) {
                packetDisplay = 0;
            }
        }
        else if (seconds > 30 && seconds < 36) {
            lcd_screen3(gps); // Screen1 is speed / sats
        }
        else if (seconds > 50 && seconds < 56) {
            lcd_screen2(gps, base, txCounter, packetDecoded); // Screen2 is Altitide / bearing / distBase / Uptime / Rx / Tx / Msgs
        }
        else {
            lcd_screen1(gps); // Screen1 is speed / sats
        }
#endif

        // Change the Tx internal based on the current speed
        // This change will not affect the countdown timer
        // Based on HamHUB Smart Beaconing(tm) algorithm

        if (gps_speed < 5) {
            txInterval = 300000;         // Change Tx internal to 5 mins
        }
        else if (gps_speed < lowSpeed) {
            txInterval = 70000;          // Change Tx interval to 60
        }
        else if (gps_speed > highSpeed) {
            txInterval = 30000;          // Change Tx interval to 30 secs
        }
        else {
            // Interval inbetween low and high speed
            txInterval = (highSpeed / gps_speed) * 30000;
        } // endif
    }  // endof gps.time.isUpdated()

    ////////////////////////////////////////////////////////////////////////////////////
    // Check for when to Tx packet
    ////////////////////////////////////////////////////////////////////////////////////
    lastTx = millis() - txTimer;

    // Only check the below if locked satellites < 3

    if (gps.satellites.value() > 3) {
        if (lastTx > 5000) {
            // Check for heading more than 25 degrees
            if ((headingDelta < -25 || headingDelta >  25) && lastTxdistance > 5) {
                if (TxtoRadio(1)) {
                    lastTxdistance = 0;   // Ensure this value is zero before the next Tx
                    previousHeading = currentHeading;
                }
            } // endif headingDelta
        } // endif lastTx > 5000

        if (lastTx > 10000) {
            // check of the last Tx distance is more than 600m
            if (lastTxdistance > 600) {
                if (TxtoRadio(2)) {
                    lastTxdistance = 0;   // Ensure this value is zero before the next Tx
                }
            } // endif lastTxdistance
        } // endif lastTx > 10000

        if (lastTx >= txInterval) {
            // Trigger Tx Tracker when Tx interval is reach
            // Will not Tx if stationary bcos speed < 5 and lastTxDistance < 20
            if (lastTxdistance > 20) {
                TxtoRadio(3);
            } // endif lastTxdistance > 20
        } // endif of check for lastTx > txInterval

        // Force a tx when 3 unique stations was received
        // Or 60 secs after last received station to prevent stale info on the distance
        if (rxStation > 2 || rxStation > 0 && ((millis() - lastRx) / 1000) > 60) {
            TxtoRadio(4);
        }
    } // Endif check for satellites

    // Check if the analog0 is plugged into 5V and more than 10 secs
    // rev13 PCB will change to pullup circuit and read using digitalRead() instead of analogRead
    if (analogRead(0) > 700 && lastTx > 10000) {
        buttonPressed = 1;
        TxtoRadio(5); 
    } // endif check analog0
} // end loop()

// New MicroAPRS function
void show_packet() {
    char *posit, *pmsgTo, *call, *pcomment, *pmsg;
    char type, pmsgID;
    long lat, lon;

    // Only displasy if decode is true
    if (microaprs.decode_posit(packet, &call, &type, &posit, &lon, &lat, &pcomment, &pmsgTo, &pmsg, &pmsgID)) {
        if (type == 58) {
            // 58 = "!" = Message
            if (startsWith(MYCALL, pmsgTo)) {
                mCounter++;
#ifdef OLED
                oled.setCursor(2,0);
                clear3Line();
                oled.setCursor(2,0);
                //oled.print("F:");
                oled.setTextSize(2,1);
                oled.print(call);
                oled.setTextSize(1,1);
                oled.setCursor(4,0);
                oled.print("M:");
                oled.print(pmsg);
#endif
                // Beep 3 times
                beep(3);
            }
        }
	else {
            // Not message, decode , calculate and display packets
            wayPointLatitude = lat / 1000000;
            wayPointLongitude = lon / 1000000;

            distanceToWaypoint = calculateDistance();
            bearing = calculateBearing();

            // Beep twice is station is less than 500m
            if (distanceToWaypoint < 0.5) {
                beep(2);
            }

            // Check for valid decoded packets
            if (strlen(call) < 12) {
                lastRx = millis();
                packetDecoded++;

                // Append rx callsign into rxCallsign strings
                // Do not add own callsign
                if (!startsWith(MYCALL, call)) {
                    // Do not add duplicated callsign from digipeater packets
                    if (!rxCallsign.startsWith(call)) {
                        rxCallsign.concat(" ");
                        rxCallsign.concat(call);
                        lastCall = call;
                        // Only send distance if GPS is locked AND less than 500km away
                        if (gps.satellites.value() > 3 && distanceToWaypoint < 500) {
                            rxCallsign.concat("/");
                            rxCallsign.concat(distanceToWaypoint);
                            rxCallsign.concat("km");
                        }
                        rxStation++;
                     }
                }

#ifdef OLED
                oled.setCursor(2, 0);
                clear6Line();
                oled.setCursor(2, 0);
                oled.setTextSize(2, 1);
                oled.print(call);
                oled.setTextSize(1, 1);

                if (distanceToWaypoint < 1000 ) {
                    oled.setCursor(4, 0);
                    oled.print("B:");
                    oled.print(bearing, 0);
                    oled.print("deg D:");
                    oled.print(distanceToWaypoint, 1);
                    oled.print("km");
                    oled.setCursor(5, 0);
                    oled.print(pcomment);
                }
                else {
                    oled.setCursor(4, 0);
                    oled.print(pcomment);
                }
#endif
#ifdef I2C16X2
                packetDisplay = 1;
                displayTime = millis();
                lcd_screen4(call, pcomment, distanceToWaypoint);
#endif
            }
	}
    } // endif microaprs.decode_posit()
}

// Functions for OLED
#ifdef OLED
void oledLine1() {
    double lat = gps.location.lat();
    double lng = gps.location.lng();
    
    oled.setCursor(0, 0);
    clearLine();
    oled.setCursor(1, 0);
    clearLine();
    oled.setCursor(0, 0);

    if (fabs(lat) < 10) {
        oled.print('0');
    }

    oled.print(convertDegMin(fabs(lat)), 2);

    // Determime to print N or S
    if (convertDegMin(lat) >= 0) {
        oled.print('N');
    }
    else {
        oled.print('S');
    }

    oled.print('/');

    // Append 0 to OLED
    if (fabs(lng) < 100) {
        oled.print('0');
    }

    // Append 0 to OLED
    if (fabs(lng) < 10) {
        oled.print('0');
    }

    oled.print(convertDegMin(fabs(lng)), 2);

    // Determime to print E or W
    if (convertDegMin(lng) >= 0) {
        oled.print('E');
    }
    else {
        oled.print('W');
    }

    oled.setCursor(0, 115);
    oled.print(gps.satellites.value());

    oled.setCursor(1, 0);
    oled.print("S:");
    oled.print((unsigned int)gps.speed.kmph());
    oled.print(" C:");
    oled.print((unsigned int)gps.course.deg());
    oled.print(" A:");
    oled.print((unsigned int)gps.altitude.meters());
}

void oledLine2() {
    oled.setCursor(0, 0);
    clearLine();
    oled.setCursor(1, 0);
    clearLine();
    oled.setCursor(0, 0);
    oled.print("Rx:");
    oled.print(packetDecoded);
    oled.print(" Tx:");
    oled.print(txCounter);
    oled.print(" L:");
    oled.print((unsigned int)(millis() - lastRx) / 1000);  // Print the seconds since last Rx

    oled.setCursor(1, 0);

    oled.print("M:");
    oled.print(mCounter);    // Messages received
    oled.print(" B:");
    oled.print(base, 0);     // Only shows km
    oled.print(" U:");
    oled.print((float)millis() / 60000, 0); // Only shows in minutes
}

void clearLine() {
    for (int i = 0; i < 21; i++) {
        oled.write(' ');
    }
}

void clear3Line() {
    for (int i = 0; i < 64; i++) {
        oled.write(' ');
    }
}

void clear6Line() {
    for (int i = 0; i < 128; i++) {
        oled.write(' ');
    }
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////
boolean TxtoRadio(int type) {
    char tmp[10];
    float latDegMin, lngDegMin = 0.0;
    String latOut, lngOut, cmtOut = "";
    unsigned int Mem = freeRam();
    float Volt = (float)readVcc() / 1000.0;

    lastTxLat = gps.location.lat();
    lastTxLng = gps.location.lng();

    if (lastTx > 6000) {
        // This prevent ANY condition to Tx below 6 secs
        latDegMin = convertDegMin(lastTxLat);
        lngDegMin = convertDegMin(lastTxLng);

        // Convert Lat float to string
        dtostrf(fabs(latDegMin), 2, 2, tmp);
        latOut.concat("lla");        // set latitute command

        // Append 0 if Lat less than 10
        if (fabs(lastTxLat) < 10) {
            latOut.concat("0");
        }

        latOut.concat(tmp);          // Actual Lat in DDMM.MM

        // Determine N or S
        if (latDegMin >= 0) {
            latOut.concat("N");
        }
        else {
            latOut.concat("S");
        }

        // Convert Lng float to string
        dtostrf(fabs(lngDegMin), 2, 2, tmp);
        lngOut.concat("llo");        // set longtitute command

        // Append 0 if Lng less than 100
        if (fabs(lastTxLng) < 100) {
            lngOut.concat("0");
        }

        // Append 0 if Lng less than 10
        if (fabs(lastTxLng) < 10) {
            latOut.concat("0");      // Append 0 if Lng less than 10
        }

        lngOut.concat(tmp);          // Actual Lng in DDDMM.MM

        // Determine E or W
        if (lngDegMin >= 0) {
            lngOut.concat("E");
        }
        else {
            lngOut.concat("W");
        }

        cmtOut.concat("@");
        cmtOut.concat(padding((int)gps.course.deg(), 3));
        cmtOut.concat("/");
        cmtOut.concat(padding((int)gps.speed.mph(), 3));
        cmtOut.concat("/A=");
        cmtOut.concat(padding((int)gps.altitude.feet(), 6));
        cmtOut.concat(" Seq:");
        cmtOut.concat(txCounter);

        if (rxCallsign.length() > 0) {
            cmtOut.concat(rxCallsign);     // Send out all the Rx callsign & dist
        }

        // Send out the type of Tx trigger
        switch (type) {
        case 1:
            cmtOut.concat(" H");
            break;
        case 2:
            cmtOut.concat(" D");
            break;
        case 3:
            cmtOut.concat(" T");
            break;
        case 4:
            cmtOut.concat(" R");
            break;
        case 5:
            cmtOut.concat(" B");
            break;
        }

        cmtOut.concat(" ");
        cmtOut.concat(COMMENT);

        // This condition is ONLY for button pressed ( do not sent out position if not locked )
        if (gps.satellites.value() > 3) {
            MODEM_SERIAL.println(latOut);
            delay(200);
            BUZZER_ON();
            MODEM_SERIAL.println(lngOut);
            delay(200);
            MODEM_SERIAL.println(cmtOut);
            BUZZER_OFF();
            delay(50);

            // Clear the rxCallsign contents & counters
            rxCallsign = "";
            rxStation = 0;
            lastRx = 0;
        }

        // Only send status/version every 10 packets to save packet size
        // Sample status display
        // >SVTrackR v1.5a 5.14V S:10 B:0 U:135m Seq:20
        if (txCounter % 10 == 0 || buttonPressed) {
            BUZZER_ON();
            cmtOut = "!>";
            cmtOut.concat(VERSION);
            cmtOut.concat(Volt);
            cmtOut.concat("V S:");
            cmtOut.concat(gps.satellites.value());
            cmtOut.concat(" B:");

            if (gps.satellites.value() > 3) {
                cmtOut.concat((unsigned int)base);
            }
            else {
                cmtOut.concat("0");
            }

            // R - number of packets decoded
            cmtOut.concat(" R:");
            cmtOut.concat(packetDecoded);

            cmtOut.concat(" U:");
            cmtOut.concat(millis()/60000);
            cmtOut.concat("m Seq:");
            cmtOut.concat(txCounter);

            delay(200);
            BUZZER_OFF();

            smartDelay(5000);
            MODEM_SERIAL.println(cmtOut);
        } //endof txCounter / buttonPressed

        // Reset all tx timer & Tx variables
        txInterval = 80000;
        buttonPressed = 0;
        txCounter++;
        txTimer = millis();
        lastTx = 0;

        // Tx success, return TRUE
        return 1;
    }
    else {
        return 0;
    }// endif lastTX > 6000
} // endof TxtoRadio()

///////////////////////////////////////////////////////////////////////////////////////////////////////
void configModem() {
    // Functions to configure the callsign, ssid, path and other settings
#ifdef I2C16X2
    lcd_config_start(MYCALL, CALL_SSID);
#endif
#ifdef OLED
    oled.setCursor(1, 0);      // move cursor to row 1, pixel column 100
    clearLine();
    oled.setCursor(1, 0);
    oled.print("Config ");
    oled.print(MYCALL);
    oled.print("-");
    oled.print(CALL_SSID);
#endif

    digitalWrite(ledPin, HIGH);
    MODEM_SERIAL.println("1WIDE1");  // Set PATH1 callsign
    delay(100);

    digitalWrite(ledPin, LOW);
    MODEM_SERIAL.println("2WIDE2");  // Set PATH2 callsign
    delay(100);

    digitalWrite(ledPin, HIGH);
    MODEM_SERIAL.println("dAPZSVT");  // Set DST Callsign to APRSVTH
    delay(100);

    digitalWrite(ledPin, LOW);
    MODEM_SERIAL.print("c");          // Set SRC Callsign
    MODEM_SERIAL.println(MYCALL);     // Set SRC Callsign
    delay(100);

    digitalWrite(ledPin, HIGH);
    MODEM_SERIAL.print("sc");         // Set SRC SSID
    MODEM_SERIAL.println(CALL_SSID);  // Set SRC SSID
    delay(100);

    digitalWrite(ledPin, LOW);
    MODEM_SERIAL.println("pd1");      // Ensable printing DST
    delay(100);

    digitalWrite(ledPin, HIGH);
    MODEM_SERIAL.println("pp0");      // Disable printing PATH
    delay(100);

    digitalWrite(ledPin, LOW);
    MODEM_SERIAL.print("ls");         // Set symbol n / Bluedot
    MODEM_SERIAL.println(SYMBOL_CHAR);      // Set symbol n / Bluedot
    delay(100);
    BUZZER_ON();
    digitalWrite(ledPin, HIGH);
    MODEM_SERIAL.print("lt");          // Standard symbol
    MODEM_SERIAL.println(SYMBOL_TABLE); // Standard symbol
    delay(100);
    BUZZER_OFF();
    digitalWrite(ledPin, LOW);
    MODEM_SERIAL.println("V1");
    delay(100);

#ifdef I2C16X2
    lcd_config_done();
    delay(1000);
#endif
#ifdef OLED
    oled.setCursor(1, 0);        // move cursor to row 1, pixel column 100
    clearLine();
    oled.setCursor(1, 0);        // move cursor to row 1, pixel column 100
    delay(1000);
    oled.print("Done..........");
    delay(500);
    oled.clear();
#endif
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
float convertDegMin(float decDeg) {
    int intDeg = decDeg;

    decDeg -= intDeg;
    decDeg *= 60;

    return 100.0 * intDeg + decDeg;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
long readVcc() {
    long result;
#if defined(__arm__) && defined(TEENSYDUINO)
    extern "C" char* sbrk(int incr);
    result = 1195 * 4096 / analogRead(39);
#else
    // Read 1.1V reference against AVcc
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    delay(2);                     // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Convert
    while (bit_is_set(ADCSRA, ADSC));
    result = ADCL;
    result |= ADCH << 8;
    result = 1126400L / result; // Back-calculate AVcc in mV
#endif

    return result;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
String padding(int number, byte width) {
    String result;
    int temp = number;

    // Prevent a log10(0) = infinity
    if (temp == 0) {
        temp++;
    }

    temp = width - log10(temp) - 1;
    for (int i = 0; i < temp; i++) {
        result.concat('0');
    }

    result.concat(number);

    return result;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
int freeRam() {
#if defined(__arm__) && defined(TEENSYDUINO)
    char top;

    return &top - reinterpret_cast<char*>(sbrk(0));
#else  // non ARM, this is AVR
    extern int __heap_start, *__brkval;
    int v;

    return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
#endif
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// convert degrees to radians
void radianConversion() {
    deltaLatitudeRadians = (wayPointLatitude - latitude) * pi / 180;
    deltaLongitudeRadians = (wayPointLongitude - longitude) * pi / 180;
    latitudeRadians = latitude * pi / 180;
    wayPointLatitudeRadians = wayPointLatitude * pi / 180;
    longitudeRadians = longitude * pi / 180;
    wayPointLongitudeRadians = wayPointLongitude * pi / 180;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// calculate distance from present location to next way point
float calculateDistance() {
    radianConversion();
    float a = sin(deltaLatitudeRadians / 2) * sin(deltaLatitudeRadians / 2) +
              sin(deltaLongitudeRadians / 2) * sin(deltaLongitudeRadians / 2) *
              cos(latitudeRadians) * cos(wayPointLatitudeRadians);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    float d = radiusOfEarth * c;

    return d;                  // distance in kilometers
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// calculate bearing from present location to next way point
float calculateBearing() {
    radianConversion();
    float y = sin(deltaLongitudeRadians) * cos(wayPointLatitudeRadians);
    float x = cos(latitudeRadians) * sin(wayPointLatitudeRadians) -
              sin(latitudeRadians) * cos(wayPointLatitudeRadians) * cos(deltaLongitudeRadians);
    bearing = atan2(y, x) / pi * 180;

    if(bearing < 0) {
        bearing = 360 + bearing;
    }

    return bearing;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
bool startsWith(const char *pre, const char *str) {
    size_t lenpre = strlen(pre);
    size_t lenstr = strlen(str);

    return lenstr < lenpre ? false : strncmp(pre, str, lenpre) == 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
static void smartDelay(int16_t ms) {
    unsigned long start = millis();

    do {
        while (MODEM_SERIAL.available() > 0) {
            char ch = microaprs.read();

            if (ch == '\n') {
                packet[buflen] = 0;
                show_packet();
                buflen = 0;
            }
            else if ((ch > 31 || ch == 0x1c || ch == 0x1d || ch == 0x27) && buflen < BUFLEN) {
                // Mic-E uses some non-printing characters
                packet[buflen++] = ch;
            }
        } // end of Serial.available while loop
    } while ((int16_t)(millis() - start) < ms);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
void beep(int i) {
    while (i > 0) {
        BUZZER_ON();
        delay(100);
        BUZZER_OFF();
        delay(100);
        i--;
    }
}
