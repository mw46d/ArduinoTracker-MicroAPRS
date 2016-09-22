#ifndef _SVTRACKR_CONFIG_H
#define _SVTRACKR_CONFIG_H

// config file to defines callsign, ssid, symbols and home location
#define MYCALL "KK6JNN"
#define CALL_SSID 9
// Refer to http://wa8lmf.net/aprs/APRS_symbols.htm
#define SYMBOL_CHAR '('         // Blue Dot
#define SYMBOL_TABLE 's'        // Primary Table (s) or Alternate Table (a)
// Define your home lat & lon below
// http://andrew.hedges.name/experiments/convert_lat_long/
// Goto aprs.fi and find your deg mm.mm ( leave the secs blank )
// home
const float HOME_LAT = 37.70683;
const float HOME_LON = -121.42267;
//
//
// Custom Comments here
#define COMMENT ""

// Turn on/off debug, on by default on pin 2,3
#undef DEBUG

// Turn on/off 16x2 I2C LCD
#undef I2C16X2

// Turn on/off 0.96" OLED
#define OLED
#define OLED_I2C

#endif /* _SVTRACKR_CONFIG_H */
