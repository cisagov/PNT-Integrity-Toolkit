//============================================================================//
//------------------------- cf_display/typdefs.cpp -------------*- C++ -*-----//
//============================================================================//
// This is free and unencumbered software released into the public domain.
//
// Anyone is free to copy, modify, publish, use, compile, sell, or
// distribute this software, either in source code form or as a compiled
// binary, for any purpose, commercial or non-commercial, and by any
// means.
//
// In jurisdictions that recognize copyright laws, the author or authors
// of this software dedicate any and all copyright interest in the
// software to the public domain. We make this dedication for the benefit
// of the public at large and to the detriment of our heirs and
// successors. We intend this dedication to be an overt act of
// relinquishment in perpetuity of all present and future rights to this
// software under copyright law.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
// OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.

// For more information, please refer to <http://unlicense.org>
//----------------------------------------------------------------------------//
//
// Typedefs for CF display (modified from source obtained from 
//           https://www.crystalfontz.com/product/linuxexamplecode)
// David Hodo <david.hodo@is4s.com>
// August, 2013
//============================================================================//
#include "cf_display/typedefs.h"

const char* cf_display::command_names[36] = {
  " 0 = Ping",
  " 1 = Read Version",
  " 2 = Write Flash",
  " 3 = Read Flash",
  " 4 = Store Boot State",
  " 5 = Reboot",
  " 6 = Clear LCD",
  " 7 = LCD Line 1",  // not a valid command for the 635 module
  " 8 = LCD Line 2",  // not a valid command for the 635 module
  " 9 = LCD CGRAM",
  "10 = Read LCD Memory",
  "11 = Place Cursor",
  "12 = Set Cursor Style",
  "13 = Contrast",
  "14 = Backlight",
  "15 = Read Fans",
  "16 = Set Fan Rpt.",
  "17 = Set Fan Power",
  "18 = Read DOW ID",
  "19 = Set Temp. Rpt",
  "20 = DOW Transaction",
  "21 = Set Live Display",
  "22 = Direct LCD Command",
  "23 = Set Key Event Reporting",
  "24 = Read Keypad, Polled Mode",
  "25 = Set Fan Fail-Safe",
  "26 = Set Fan RPM Glitch Filter",
  "27 = Read Fan Pwr & Fail-Safe",
  "28 = Set ATX switch functionality",
  "29 = Watchdog Host Reset",
  "30 = Rd Rpt",
  "31 = Send Data to LCD (row,col)",
  "32 = Key Legends",
  "33 = Set Baud Rate",
  "34 = Set/Configure GPIO",
  "35 = Read GPIO & Configuration"};

const char* cf_display::error_names[36] = {
  "Error: Ping",
  "Error: Version",
  "Error: Write Flash",
  "Error: Read Flash",
  "Error: Store Boot State",
  "Error: Reboot",
  "Error: Clear LCD",
  "Error: Set LCD 1",
  "Error: Set LCD 2",
  "Error: Set LCD CGRAM",
  "Error: Read LCD Memory",
  "Error: Place LCD Cursor",
  "Error: LCD Cursor Style",
  "Error: Contrast",
  "Error: Backlight",
  "Error: Query Fan",
  "Error: Set Fan Rept.",
  "Error: Set Fan Power",
  "Error: Read DOW ID",
  "Error: Set Temp. Rept.",
  "Error: DOW Transaction",
  "Error: Setup Live Disp",
  "Error: Direct LCD Command",
  "Error: Set Key Event Reporting",
  "Error: Read Keypad, Polled Mode",
  "Error: Set Fan Fail-Safe",
  "Error: Set Fan RPM Glitch Filter",
  "Error: Read Fan Pwr & Fail-Safe",
  "Error: Set ATX switch functionality",
  "Error: Watchdog Host Reset",
  "Error: Read  Reporting/ATX/Watchdog",
  "Error: Send Data to LCD (row,col)",
  "Error: Key Legends",
  "Error: Set Baud Rate",
  "Error: Set/Configure GPIO",
  "Error: Read GPIO & Configuration"};

const char* cf_display::key_names[21] = {
  "KEY_NONE",         "KEY_UP_PRESS",      "KEY_DOWN_PRESS",
  "KEY_LEFT_PRESS",   "KEY_RIGHT_PRESS",   "KEY_ENTER_PRESS",
  "KEY_EXIT_PRESS",   "KEY_UP_RELEASE",    "KEY_DOWN_RELEASE",
  "KEY_LEFT_RELEASE", "KEY_RIGHT_RELEASE", "KEY_ENTER_RELEASE",
  "KEY_EXIT_RELEASE", "KEY_UL_PRESS",      "KEY_UR_PRESS",
  "KEY_LL_PRESS",     "KEY_LR_PRESS",      "KEY_UL_RELEASE",
  "KEY_UR_RELEASE",   "KEY_LL_RELEASE",    "KEY_LR_RELEASE"};