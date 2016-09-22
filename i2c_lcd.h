#ifndef _I2C_LCD_H
#define _I2C_LCD_H

#ifdef I2C16X2
extern void lcd_setup(char *v);

extern void lcd_screen1(TinyGPSPlus &gps);
extern void lcd_screen2(TinyGPSPlus &gps, float base, unsigned int txCounter, unsigned int packetDecoded);
extern void lcd_screen3(TinyGPSPlus &gps);
extern void lcd_screen4(char *callsign, char *comment, float distanceToWaypoint);

extern void lcd_config_start(char *mycall, int ssid);
extern void lcd_config_done();
#endif

#endif // _I2C_LCD_H

