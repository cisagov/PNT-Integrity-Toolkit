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
/// \brief    his example shows to how to use the cf_display class.
/// \author   David Hodo <david.hodo@is4s.com>
/// \date     August, 2013
//============================================================================//
#include <iostream>
#include <string>
#include "cf_display/cf_display.h"
#include "cf_display/typedefs.h"
using namespace cf_display;

inline void DefaultDebugMsgCallback(const std::string& msg)
{
  std::cout << "[Debug] " << msg << std::endl;
}

inline void DefaultInfoMsgCallback(const std::string& msg)
{
  std::cout << "[Info] " << msg << std::endl;
}

inline void DefaultWarningMsgCallback(const std::string& msg)
{
  std::cout << "[Warning] " << msg << std::endl;
}

inline void DefaultErrorMsgCallback(const std::string& msg)
{
  std::cout << "[Error] " << msg << std::endl;
}

inline void HandleKeypress(uint8_t key_id)
{
  std::cout << "Keypress: " << cf_display::key_names[key_id] << std::endl;
}

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    std::cerr << "Usage: cf_633_example <serial port address> ";
    std::cerr << "<baudrate> " << std::endl;
    return 0;
  }
  // Argument 1 is the serial port
  std::string port(argv[1]);

  // Argument 2 is the baudrate
  unsigned long baud = 0;
#ifdef WIN32
  sscanf_s(argv[2], "%lu", &baud);
#else
  sscanf(argv[2], "%lu", &baud);
#endif

  cf_display::CfDisplay display;
  display.setLogDebugCallback(DefaultDebugMsgCallback);
  display.setLogInfoCallback(DefaultInfoMsgCallback);
  display.setLogWarningCallback(DefaultWarningMsgCallback);
  display.setLogErrorCallback(DefaultErrorMsgCallback);
  display.SetKeypressCallback(HandleKeypress);

  if (!display.Initialize(port, baud))
  {
    std::cerr << "Error initializing CF Display." << std::endl;
    return 1;
  }

  display.ClearDisplay();
  display.SetData("Test", 1, 0);
  display.SetLEDColor(0, LedColor::LED_COLOR_RED);

  while (1)
  {
    std::this_thread::sleep_for(std::chrono::seconds(10));
  }
}