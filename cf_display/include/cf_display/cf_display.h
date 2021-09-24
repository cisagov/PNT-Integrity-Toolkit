//============================================================================//
//------------------------ cf_display/cf_display.h -------------*- C++ -*-----//
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
/// \brief    Class definition for CrystalFontz LCD display
/// \author   David Hodo <david.hodo@is4s.com>
/// \date     August, 2013
//============================================================================//
#ifndef CF_DISPLAY__CF_DISPLAY_H
#define CF_DISPLAY__CF_DISPLAY_H

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#include <serial/serial.h>
#include <stdint.h>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include "cf_display/typedefs.h"

/*****************************************************************************
 ** Type Definitions
 *****************************************************************************/

using KeypressCallback = std::function<void(uint8_t key_code)>;
using LogMsgCallback   = std::function<void(const std::string&)>;

/*****************************************************************************
 ** Namespace
 *****************************************************************************/
namespace cf_display
{
enum class LedColor
{
  LED_COLOR_GREEN = 0,
  LED_COLOR_RED,
  LED_COLOR_YELLOW,
};

/*
 * Class to interface a CrystalFontz LCD/Keypad unit.
 */
class CfDisplay
{
public:
  // Default Constructor
  CfDisplay();
  // Default Desctructor
  ~CfDisplay();

  bool Initialize(std::string device, int baud_rate, size_t num_attempts = 5);
  void Shutdown();
  bool Ping(size_t num_attempts);
  bool IsConnected();

  void SetGPIO(uint8_t gpio, uint8_t level);

  void SetData(std::string data, uint8_t row = 0, uint8_t col = 0);
  void ClearCheckName();
  void replaceUnderscore(std::string);
  void ClearDisplay();

  void SetLED(uint8_t ledNum, uint8_t gpio, uint8_t level);
  void SetLEDColor(uint8_t ledNum, const LedColor& ledColor);
  void ClearLED(uint8_t ledNum, uint8_t gpio);
  void ClearAllLeds();

  void SetCursorPosition(uint8_t row, uint8_t col);
  void SetCursorStyle(uint8_t style);

  void SetKeypressCallback(KeypressCallback callback)
  {
    keypress_callback_ = callback;
  };

  void setLogDebugCallback(LogMsgCallback debug_callback)
  {
    log_debug_ = debug_callback;
  };
  void setLogInfoCallback(LogMsgCallback info_callback)
  {
    log_info_ = info_callback;
  };
  void setLogWarningCallback(LogMsgCallback warning_callback)
  {
    log_warning_ = warning_callback;
  };
  void setLogErrorCallback(LogMsgCallback error_callback)
  {
    log_error_ = error_callback;
  };

private:
  bool     SendPacket(CommandPacket packet);
  uint16_t GetCrc(uint8_t* bufptr, uint16_t len, uint16_t seed);

  void ReadSerialPort();
  void BufferIncomingData(uint8_t* message, size_t length);

  //! Callback function to call when a key is pressed
  KeypressCallback keypress_callback_;

  LogMsgCallback log_debug_;
  LogMsgCallback log_info_;
  LogMsgCallback log_warning_;
  LogMsgCallback log_error_;

  //! Object to communicate with the serial port
  serial::Serial* serial_port_;

  //! Pointer to thread to read from serial port
  std::shared_ptr<std::thread> read_thread_ptr_;
  bool
    reading_status_;  //!< True if the read thread is running, false otherwise.

  uint8_t
    data_buffer_[MAX_DATA_LENGTH];  //!< data currently being buffered to read
  size_t buffer_index_;             //!< index into data_buffer_
};

}  // namespace cf_display
#endif /* CF_DISPLAY_H_ */
