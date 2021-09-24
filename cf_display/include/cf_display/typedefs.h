//============================================================================//
//------------------------ cf_display/typedefs.h -------------*- C++ -*-----//
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
/// \file
/// \brief    Typedefs for CF display (modified from source obtained from 
///           https://www.crystalfontz.com/product/linuxexamplecode)
/// \author   David Hodo <david.hodo@is4s.com>
/// \date     August, 2013
//============================================================================//
#ifndef CF_DISPLAY_TYPEDEFS_H_
#define CF_DISPLAY_TYPEDEFS_H_

#include <cstddef>
#include <cstdint>

namespace cf_display
{
//<type><data_length><data><CRC>
// type is one byte, and identifies the type and function of the packet:
//   TTcc cccc
//   |||| ||||--Command, response, error or report code 0-63
//   ||---------Type:
//                00 = normal command from host to CFA633
//                01 = normal response from CFA633 to host
//                10 = normal report from CFA633 to host (not in
//                     direct response to a command from the host)
//                11 = error response from CFA633 to host (a packet
//                     with valid structure but illegal content
//                     was received by the CFA633)

/*****************************************************************************
 ** Constants
 *****************************************************************************/

const size_t  MAX_DATA_LENGTH = 22;
const uint8_t MAX_COMMAND     = 35;

typedef union
{
  uint8_t  as_bytes[2];
  uint16_t as_word;
} WordUnion;

typedef struct
{
  uint8_t   command;
  uint8_t   data_length;
  uint8_t   data[MAX_DATA_LENGTH];
  WordUnion CRC;
} CommandPacket;

// Define commands
const uint8_t CMD_PING             = 0;
const uint8_t CMD_VERSION          = 1;
const uint8_t CMD_REBOOT           = 5;
const uint8_t CMD_CLEAR            = 6;
const uint8_t CMD_PLACE_CURSOR     = 11;
const uint8_t CMD_SET_CURSOR_STYLE = 12;
const uint8_t CMD_CONTRAST         = 13;
const uint8_t CMD_BACKLIGHT        = 14;
const uint8_t CMD_SEND_DATA        = 0x1F;
const uint8_t CMD_SET_LED          = 34;

const uint8_t CURSOR_STYLE_NO_CURSOR           = 0;
const uint8_t CURSOR_STYLE_BLINKING_BLOCK      = 1;
const uint8_t CURSOR_STYLE_STATIC_UNDERSCORE   = 2;
const uint8_t CURSOR_STYLE_BLINKING_UNDERSCORE = 3;

const uint8_t KEY_UP_PRESS      = 1;
const uint8_t KEY_DOWN_PRESS    = 2;
const uint8_t KEY_LEFT_PRESS    = 3;
const uint8_t KEY_RIGHT_PRESS   = 4;
const uint8_t KEY_ENTER_PRESS   = 5;
const uint8_t KEY_EXIT_PRESS    = 6;
const uint8_t KEY_UP_RELEASE    = 7;
const uint8_t KEY_DOWN_RELEASE  = 8;
const uint8_t KEY_LEFT_RELEASE  = 9;
const uint8_t KEY_RIGHT_RELEASE = 10;
const uint8_t KEY_ENTER_RELEASE = 11;
const uint8_t KEY_EXIT_RELEASE  = 12;

extern const char* command_names[36];
extern const char* error_names[36];
extern const char* key_names[21];
//============================================================================

}  // namespace cf_display

#endif