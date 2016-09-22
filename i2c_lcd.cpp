#include <TinyGPS++.h>

#include "config.h"

#ifdef I2C16X2
#include <LiquidCrystal_I2C.h>
// LiquidCrystal_I2C lcd(0x27,16,2); // 0x20 is adresss for LCC 16x2
LiquidCrystal_I2C lcd(0x3F,20,4); // 0x20 is adresss for LCC 16x2

void lcd_setup(char *v) {
    lcd.backlight();
    lcd.begin();
    lcd.print(v);
}

void lcd_screen1(TinyGPSPlus &gps) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print((uint16_t)gps.speed.kmph());

    uint16_t s = gps.satellites.value();
    lcd.setCursor(13, 0);
    if (s < 10) {
        lcd.print(" ");
    }
    lcd.print(s);
    lcd.print("s");

    // Print degree or N/S/E/W headings
    uint16_t d = (uint16_t)gps.course.deg();
    lcd.setCursor(12,1);
    if (d < 100) {
        lcd.print("0");
    }
    if (d < 10) {
        lcd.print("0");
    }
    lcd.print(d);
    lcd.print("d");
}

void lcd_screen2(TinyGPSPlus &gps, float base, uint16_t txCounter, uint16_t packetDecoded) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("B:");
    lcd.print((uint16_t)base);     // Max 999

    lcd.setCursor(6, 0);
    lcd.print("T:");
    lcd.print(txCounter);  // Max 999

    // Max 2 digits10w

    lcd.setCursor(12, 0);
    lcd.print("R:");
    lcd.print(packetDecoded);  // Max 99

    lcd.setCursor(0, 1);
    lcd.print("U:");
    lcd.print(millis() / 60000); // Max 999

    lcd.setCursor(6, 1);
    lcd.print("A:");
    lcd.print((uint16_t)gps.altitude.meters()); // Max 9999
}

void lcd_screen3(TinyGPSPlus &gps) {
    lcd.clear();
    lcd.setCursor(0, 0);

    char *timeString = "00:00:00";
    uint8_t x = gps.time.hour();
    timeString[0] = x / 10 + '0';
    timeString[1] = x % 10 + '0';

    x = gps.time.minute();
    timeString[3] = x / 10 + '0';
    timeString[4] = x % 10 + '0';

    x = gps.time.second();
    timeString[6] = x / 10 + '0';
    timeString[7] = x % 10 + '0';

    lcd.print(timeString);

    lcd.print(" H:");
    lcd.print(gps.hdop.value(), 1);

    lcd.setCursor(0, 1);
    lcd.print(gps.location.lat(), 4);
    lcd.setCursor(8, 1);
    lcd.print(gps.location.lng(), 4);
}

void lcd_screen4(char *callsign, char *comment, float distanceToWaypoint) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(callsign);

    if (distanceToWaypoint < 500) {
        lcd.print(" ");
        lcd.print(distanceToWaypoint, 1);
        lcd.print("km");
    }

    lcd.setCursor(0, 1);
    comment[16] = 0;
    lcd.print(comment);
}

void lcd_config_start(char *mycall, int ssid) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Config:");
    lcd.setCursor(0, 1);
    lcd.print(mycall);
    lcd.print("-");
    lcd.print(ssid);
}

void lcd_config_done() {
    lcd.setCursor(0, 0);
    lcd.print("Done...");
}

#endif
