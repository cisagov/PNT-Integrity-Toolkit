//============================================================================//
//---------------------- cf_display/cf_display.cpp -------------*- C++-* -----//
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
//
// For more information, please refer to <http://unlicense.org>
//----------------------------------------------------------------------------//
//
// Class definition for main toolkit application
// David Hodo <david.hodo@is4s.com>
// August, 2013
//============================================================================//
#include "cf_display/cf_display.h"
#include <algorithm>

using namespace cf_display;

inline void DefaultDebugMsgCallback(const std::string& /*msg*/)
{
  // std::cout << "[DEBUG]: " << msg << std::endl;
}

inline void DefaultInfoMsgCallback(const std::string& msg)
{
  std::cout << "[INFO]: " << msg << std::endl;
}

inline void DefaultWarningMsgCallback(const std::string& msg)
{
  std::cout << "[WARN]: " << msg << std::endl;
}

inline void DefaultErrorMsgCallback(const std::string& msg)
{
  std::cout << "[ERROR]: " << msg << std::endl;
}

CfDisplay::CfDisplay()
  : log_debug_(DefaultDebugMsgCallback)
  , log_info_(DefaultInfoMsgCallback)
  , log_warning_(DefaultWarningMsgCallback)
  , log_error_(DefaultErrorMsgCallback)
  , serial_port_(NULL)
  , read_thread_ptr_(nullptr)
  , reading_status_(false)
  , buffer_index_(0)
{
}

CfDisplay::~CfDisplay()
{
}

bool CfDisplay::Initialize(std::string device,
                           int         baud_rate,
                           size_t      num_attempts)
{
  serial_port_ =
    new serial::Serial(device, baud_rate, serial::Timeout::simpleTimeout(50));

  if (!serial_port_->isOpen())
  {
    std::stringstream output;
    output << "Serial port: " << device << " failed to open.";
    log_error_(output.str());
    delete serial_port_;
    serial_port_ = NULL;
    return false;
  }

  std::stringstream output;
  output << "Serial port: " << device << " opened successfully.";
  log_info_(output.str());
  if (Ping(num_attempts))
  {
    log_info_("Cf display detected.");
    reading_status_  = true;
    read_thread_ptr_ = std::make_shared<std::thread>(
      std::thread(std::bind(&CfDisplay::ReadSerialPort, this)));
    log_info_("Cf Display successfully initialized.");
    return true;
  }
  else
  {
    log_error_("Cf display not detected.");
    return false;
  }
}

void CfDisplay::Shutdown()
{
  reading_status_ = false;

  try
  {
    if ((serial_port_ != NULL) && (serial_port_->isOpen()))
    {
      serial_port_->close();
      delete serial_port_;
      serial_port_ = NULL;
    }
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error during disconnect: " << e.what();
    log_error_(output.str());
  }
}

bool CfDisplay::Ping(size_t num_attempts)
{
  //  0 (0x00): Ping Command
  //  The CFA633 will return the Ping Command to the host.
  //    type: 0x00 = 010
  //    valid data_length is 0 to 16
  //    data[0-(data_length-1)] can be filled with any arbitrary data
  //  The return packet is identical to the packet sent, except the type will
  //  be 0x40 (normal response, Ping Command):
  //    type: 0x40 | 0x00 = 0x40 = 6410
  //    data_length = (identical to received packet) data[0-(data_length-1)] =
  //       (identical to received packet)

  try
  {
    for (size_t attempt = 0; attempt < num_attempts; attempt++)
    {
      std::stringstream msg_attempt;
      msg_attempt << "Pinging display [" << attempt + 1 << ":" << num_attempts
                  << "]";
      log_info_(msg_attempt.str());

      serial_port_->flush();
      // read out any data currently in the buffer
      std::string read_data = serial_port_->read(10);
      while (read_data.length())
        read_data = serial_port_->read(10);
      CommandPacket cmd;
      cmd.command     = CMD_PING;
      cmd.data_length = 0;
      // send ping request
      SendPacket(cmd);

      // wait for response from the receiver - will respond in < 250ms by spec
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
      // read from the serial port - up to 20 bytes - message should be 4
      uint8_t buffer[20];
      size_t  bytes_read = serial_port_->read(buffer, 20);
      if (bytes_read == 0)
      {
        log_warning_("No bytes received in response to ping.");
        continue;
      }

      // search for ping response
      // only loop through bytes_read-3 so there are enough remaining
      // bytes for data_length and checksum
      for (size_t index = 0; index < bytes_read - 3; index++)
      {
        // std::cout << "Byte " << index << ": " << std::hex <<
        // (int)buffer[index] << std::endl;
        // look for checksum command
        if (buffer[index] == 0x40 && buffer[index + 1] == 0x0 &&
            buffer[index + 2] == 0x21 && buffer[index + 3] == 0x49)
        {
          // check checksum
          log_info_("Ping response received.");
          return true;
        }
      }
      std::stringstream msg;
      msg << "Read " << bytes_read << " bytes but no ping response found.";
      log_warning_(msg.str());
    }
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error pinging display: " << e.what();
    log_error_(output.str());
  }

  // finished loop without getting a ping response
  log_error_("No ping response received from display");
  return false;
}

bool CfDisplay::IsConnected()
{
  if ((serial_port_ != NULL) && (serial_port_->isOpen()))
    return true;
  else
    return false;
}

void CfDisplay::SetLED(uint8_t ledNum, uint8_t gpio, uint8_t level)
{
  if ((serial_port_ == NULL) || (!serial_port_->isOpen()))
  {
    log_error_("Unable to set LED.  Serial port is not open.");
    return;
  }

  try
  {
    CommandPacket cmd;
    cmd.command = CMD_SET_LED;
    if (gpio >= 5 && gpio <= 12 && level < 101)
    {
      switch (ledNum)
      {
        case 0:
          if (gpio % 2 == 0)
          {
            cmd.data[0] = 12;
            cmd.data[1] = level;
          }
          else
          {
            cmd.data[0] = 11;
            cmd.data[1] = level;
          }
          break;
        case 1:
          if (gpio % 2 == 0)
          {
            cmd.data[0] = 10;
            cmd.data[1] = level;
          }
          else
          {
            cmd.data[0] = 9;
            cmd.data[1] = level;
          }
          break;
        case 2:
          if (gpio % 2 == 0)
          {
            cmd.data[0] = 8;
            cmd.data[1] = level;
          }
          else
          {
            cmd.data[0] = 7;
            cmd.data[1] = level;
          }
          break;
        case 3:
          if (gpio % 2 == 0)
          {
            cmd.data[0] = 6;
            cmd.data[1] = level;
          }
          else
          {
            cmd.data[0] = 5;
            cmd.data[1] = level;
          }
          break;
      }
    }
    cmd.data_length = 2;
    SendPacket(cmd);
  }

  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error setting LED: " << e.what();
    log_error_(output.str());
  }
}

void CfDisplay::SetLEDColor(uint8_t ledNum, const LedColor& ledColor)
{
  if ((serial_port_ == NULL) || (!serial_port_->isOpen()))
  {
    log_error_("Unable to set LED.  Serial port is not open.");
    return;
  }

  try
  {
    CommandPacket cmd;
    cmd.command = CMD_SET_LED;

    ClearAllLeds();

    switch (ledColor)
    {
      case LedColor::LED_COLOR_GREEN:
      {
        ClearLED(ledNum, 12);
        SetLED(ledNum, 11, 100);
        break;
      }
      case LedColor::LED_COLOR_RED:
      {
        ClearLED(ledNum, 11);
        SetLED(ledNum, 12, 100);
        break;
      }
      case LedColor::LED_COLOR_YELLOW:
      {
        SetLED(ledNum, 11, 100);
        SetLED(ledNum, 12, 100);
        break;
      }
    }
    cmd.data_length = 2;
    SendPacket(cmd);
  }

  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error setting LED: " << e.what();
    log_error_(output.str());
  }
}
void CfDisplay::SetData(std::string data, uint8_t row, uint8_t col)
{
  //  This command allows data to be placed at any position on the LCD.
  //  type: 0x1F = 3110 data_length = 3 to 18
  //     data[0]: col = x = 0 to 15
  //     data[1]: row = y = 0 to 1
  //     data[2-21]: text to place on the LCD, variable from 1 to 16 characters
  //  The return packet will be:
  //    type: 0x40 | 0x1F = 0x5F = 9510 data_length = 0

  if ((serial_port_ == NULL) || (!serial_port_->isOpen()))
  {
    log_error_("Unable to clear display.  Serial port is not open.");
    return;
  }

  try
  {
    CommandPacket cmd;
    cmd.command = CMD_SEND_DATA;
    cmd.data[0] = col;  // col
    cmd.data[1] = row;  // row
    memcpy(&cmd.data[2], data.c_str(), data.length());
    cmd.data_length = data.length() + 2;
    SendPacket(cmd);
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error sending data to display: " << e.what();
    log_error_(output.str());
  }
}
void CfDisplay::ClearCheckName()
{
  for (unsigned int ii = 0; ii < 19; ii++)
  {
    SetData(" ", 3, ii);
  }
}
void CfDisplay::replaceUnderscore(std::string checkName)
{
  std::replace(checkName.begin(), checkName.end(), '_', ' ');
  std::transform(
    checkName.begin(), checkName.end(), checkName.begin(), ::toupper);
  SetData(checkName, 3, 0);
}

void CfDisplay::ClearDisplay()
{
  // Sets the contents of the LCD screen DDRAM to ' ' = 0x20 = 32 and moves
  // the cursor to the left-most column of the top line.
  //   type: 0x06 = 610
  //   valid data_length is 0
  // The return packet will be:
  //   type: 0x40 | 0x06 = 0x46 = 7010 data_length = 0

  // send command to clear display
  if ((serial_port_ == NULL) || (!serial_port_->isOpen()))
  {
    log_error_("Unable to clear display.  Serial port is not open.");
    return;
  }

  try
  {
    CommandPacket cmd;
    cmd.command     = CMD_CLEAR;
    cmd.data_length = 0;
    SendPacket(cmd);
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error clearing display: " << e.what();
    log_error_(output.str());
  }
}
void CfDisplay::ClearAllLeds()
{
  // Loop that clears led every time Assurance Level changes
  for (int clear_led = 0; clear_led < 4; clear_led++)
  {
    ClearLED(clear_led, 11);  // clears green
    ClearLED(clear_led, 12);  // clears red
  }
}

void CfDisplay::ClearLED(uint8_t ledNum, uint8_t gpio)
{
  // Turns LEDs OFF
  if ((serial_port_ == NULL) || (!serial_port_->isOpen()))
  {
    log_error_("Unable to clear LEDs.  Serial port is not open.");
    return;
  }

  try
  {
    CommandPacket cmd;
    cmd.command = CMD_SET_LED;
    switch (ledNum)
    {
      case 0:
        if (gpio % 2 == 0)
        {
          cmd.data[0] = 12;
          cmd.data[1] = 0;
        }
        else
        {
          cmd.data[0] = 11;
          cmd.data[1] = 0;
        }
        break;
      case 1:
        if (gpio % 2 == 0)
        {
          cmd.data[0] = 10;
          cmd.data[1] = 0;
        }
        else
        {
          cmd.data[0] = 9;
          cmd.data[1] = 0;
        }
        break;
      case 2:
        if (gpio % 2 == 0)
        {
          cmd.data[0] = 8;
          cmd.data[1] = 0;
        }
        else
        {
          cmd.data[0] = 7;
          cmd.data[1] = 0;
        }
        break;
      case 3:
        if (gpio % 2 == 0)
        {
          cmd.data[0] = 6;
          cmd.data[1] = 0;
        }
        else
        {
          cmd.data[0] = 5;
          cmd.data[1] = 0;
        }
        break;
    }

    cmd.data_length = 2;
    SendPacket(cmd);
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error clearing LEDs: " << e.what();
    log_error_(output.str());
  }
}

void CfDisplay::SetCursorPosition(uint8_t row, uint8_t col)
{
  // This command allows the cursor to be placed at the
  // desired location on the CFA633’s LCD screen.  The cursor
  // must also be made visible using command 12 (0x0C)
  //   type: 0x0B = 1110 valid data_length is 2
  //    data[0] = column (0-15 valid)
  //    data[1] = row (0-1 valid)
  // The return packet will be:
  //   type: 0x40 | 0x0B = 0x4B = 7510 data_length = 0

  if ((serial_port_ == NULL) || (!serial_port_->isOpen()))
  {
    log_error_("Unable to set cursor style.  Serial port is not open.");
    return;
  }

  try
  {
    CommandPacket cmd;
    cmd.command     = CMD_PLACE_CURSOR;
    cmd.data[0]     = col;  // col
    cmd.data[1]     = row;  // row
    cmd.data_length = 2;
    SendPacket(cmd);
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error setting cursor position: " << e.what();
    log_error_(output.str());
  }
}

void CfDisplay::SetCursorStyle(uint8_t style)
{
  // This command allows you to select among four hardware generated cursor
  // options. type: 0x0C = 1210 valid data_length is 1 data[0] = cursor style
  // (0-3 valid)
  //            0 = no cursor.
  //            1 = blinking block cursor.
  //            2 = static underscore cursor.
  //            3 = blinking underscore cursor.
  //            (Note for 3: This behavior is different from the previous CFA633
  //            HW v1.x family which had a blinking block with a static
  //            underscore cursor.)
  // The return packet will be:
  // type: 0x40 | 0x0C = 0x4C = 7610 data_length = 0

  if ((serial_port_ == NULL) || (!serial_port_->isOpen()))
  {
    log_error_("Unable to set cursor position.  Serial port is not open.");
    return;
  }

  try
  {
    CommandPacket cmd;
    cmd.command     = CMD_SET_CURSOR_STYLE;
    cmd.data[0]     = style;  // style
    cmd.data_length = 1;
    SendPacket(cmd);
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error setting cursor style: " << e.what();
    log_error_(output.str());
  }
}

bool CfDisplay::SendPacket(CommandPacket packet)
{
  uint16_t* packed_CRC_position;

  packed_CRC_position = (uint16_t*)&packet.data[packet.data_length];

  *packed_CRC_position =
    GetCrc((uint8_t*)&packet, packet.data_length + 2, 0xFFFF);

  if ((serial_port_ != NULL) && (serial_port_->isOpen()))
  {
    // command, length, data[data_length], crc, crc
    serial_port_->write((const unsigned char*)&packet, packet.data_length + 4);
    return true;
  }
  else
  {
    return false;
  }
}

uint16_t CfDisplay::GetCrc(uint8_t* bufptr, uint16_t len, uint16_t seed)
{
  // CRC lookup table to avoid bit-shifting loops.
  static const uint16_t crcLookupTable[256] = {
    0x00000, 0x01189, 0x02312, 0x0329B, 0x04624, 0x057AD, 0x06536, 0x074BF,
    0x08C48, 0x09DC1, 0x0AF5A, 0x0BED3, 0x0CA6C, 0x0DBE5, 0x0E97E, 0x0F8F7,
    0x01081, 0x00108, 0x03393, 0x0221A, 0x056A5, 0x0472C, 0x075B7, 0x0643E,
    0x09CC9, 0x08D40, 0x0BFDB, 0x0AE52, 0x0DAED, 0x0CB64, 0x0F9FF, 0x0E876,
    0x02102, 0x0308B, 0x00210, 0x01399, 0x06726, 0x076AF, 0x04434, 0x055BD,
    0x0AD4A, 0x0BCC3, 0x08E58, 0x09FD1, 0x0EB6E, 0x0FAE7, 0x0C87C, 0x0D9F5,
    0x03183, 0x0200A, 0x01291, 0x00318, 0x077A7, 0x0662E, 0x054B5, 0x0453C,
    0x0BDCB, 0x0AC42, 0x09ED9, 0x08F50, 0x0FBEF, 0x0EA66, 0x0D8FD, 0x0C974,
    0x04204, 0x0538D, 0x06116, 0x0709F, 0x00420, 0x015A9, 0x02732, 0x036BB,
    0x0CE4C, 0x0DFC5, 0x0ED5E, 0x0FCD7, 0x08868, 0x099E1, 0x0AB7A, 0x0BAF3,
    0x05285, 0x0430C, 0x07197, 0x0601E, 0x014A1, 0x00528, 0x037B3, 0x0263A,
    0x0DECD, 0x0CF44, 0x0FDDF, 0x0EC56, 0x098E9, 0x08960, 0x0BBFB, 0x0AA72,
    0x06306, 0x0728F, 0x04014, 0x0519D, 0x02522, 0x034AB, 0x00630, 0x017B9,
    0x0EF4E, 0x0FEC7, 0x0CC5C, 0x0DDD5, 0x0A96A, 0x0B8E3, 0x08A78, 0x09BF1,
    0x07387, 0x0620E, 0x05095, 0x0411C, 0x035A3, 0x0242A, 0x016B1, 0x00738,
    0x0FFCF, 0x0EE46, 0x0DCDD, 0x0CD54, 0x0B9EB, 0x0A862, 0x09AF9, 0x08B70,
    0x08408, 0x09581, 0x0A71A, 0x0B693, 0x0C22C, 0x0D3A5, 0x0E13E, 0x0F0B7,
    0x00840, 0x019C9, 0x02B52, 0x03ADB, 0x04E64, 0x05FED, 0x06D76, 0x07CFF,
    0x09489, 0x08500, 0x0B79B, 0x0A612, 0x0D2AD, 0x0C324, 0x0F1BF, 0x0E036,
    0x018C1, 0x00948, 0x03BD3, 0x02A5A, 0x05EE5, 0x04F6C, 0x07DF7, 0x06C7E,
    0x0A50A, 0x0B483, 0x08618, 0x09791, 0x0E32E, 0x0F2A7, 0x0C03C, 0x0D1B5,
    0x02942, 0x038CB, 0x00A50, 0x01BD9, 0x06F66, 0x07EEF, 0x04C74, 0x05DFD,
    0x0B58B, 0x0A402, 0x09699, 0x08710, 0x0F3AF, 0x0E226, 0x0D0BD, 0x0C134,
    0x039C3, 0x0284A, 0x01AD1, 0x00B58, 0x07FE7, 0x06E6E, 0x05CF5, 0x04D7C,
    0x0C60C, 0x0D785, 0x0E51E, 0x0F497, 0x08028, 0x091A1, 0x0A33A, 0x0B2B3,
    0x04A44, 0x05BCD, 0x06956, 0x078DF, 0x00C60, 0x01DE9, 0x02F72, 0x03EFB,
    0x0D68D, 0x0C704, 0x0F59F, 0x0E416, 0x090A9, 0x08120, 0x0B3BB, 0x0A232,
    0x05AC5, 0x04B4C, 0x079D7, 0x0685E, 0x01CE1, 0x00D68, 0x03FF3, 0x02E7A,
    0x0E70E, 0x0F687, 0x0C41C, 0x0D595, 0x0A12A, 0x0B0A3, 0x08238, 0x093B1,
    0x06B46, 0x07ACF, 0x04854, 0x059DD, 0x02D62, 0x03CEB, 0x00E70, 0x01FF9,
    0x0F78F, 0x0E606, 0x0D49D, 0x0C514, 0x0B1AB, 0x0A022, 0x092B9, 0x08330,
    0x07BC7, 0x06A4E, 0x058D5, 0x0495C, 0x03DE3, 0x02C6A, 0x01EF1, 0x00F78};

  // Initial CRC value is 0x0FFFF.
  uint16_t newCrc;
  newCrc = seed;
  // This algorithm is based on the IrDA LAP example.
  while (len--)
    newCrc = (newCrc >> 8) ^ crcLookupTable[(newCrc ^ *bufptr++) & 0xff];

  // Make this CRC match the one's complement that is sent in the packet.
  return ~newCrc;
}

void CfDisplay::ReadSerialPort()
{
  uint8_t buffer[MAX_DATA_LENGTH];
  size_t  len;
  log_info_("Started Cf Display read thread.");

  // continuously read data from serial port
  while (reading_status_)
  {
    try
    {
      // read data
      len = serial_port_->read(buffer, MAX_DATA_LENGTH);
      // std::cout << "Read " << len << std::endl;
    }
    catch (std::exception& e)
    {
      std::stringstream output;
      output << "Error reading from serial port: " << e.what();
      log_error_(output.str());
      // return;
    }

    // add data to the buffer to be parsed
    BufferIncomingData(buffer, len);
  }
}

void CfDisplay::BufferIncomingData(uint8_t* message, size_t length)
{
  // If a key is pressed or released, the CFA633 sends a Key Activity report
  // packet to the host. Key event reporting may be individually enabled or
  // disabled by command 23 (0x17): Configure Key Reporting (Pg. 48).
  //   type = 0x80
  //   data_length = 1
  //   data[0] is the type of keyboard activity:
  //          KEY_UP_PRESS              1
  //          KEY_DOWN_PRESS            2
  //          KEY_LEFT_PRESS            3
  //          KEY_RIGHT_PRESS           4
  //          KEY_ENTER_PRESS           5
  //          KEY_EXIT_PRESS            6
  //          KEY_UP_RELEASE            7
  //          KEY_DOWN_RELEASE          8
  //          KEY_LEFT_RELEASE          9
  //          KEY_RIGHT_RELEASE        10
  //          KEY_ENTER_RELEASE        11
  //          KEY_EXIT_RELEASE         12

  // loop through received data and look for keypress message
  for (size_t ii = 0; ii < length; ii++)
  {
    // std::cout << "Byte " << ii << std::hex << "  0x" << (int)message[ii] <<
    // std::dec << std::endl;
    // make sure bufIndex is not larger than buffer
    if (buffer_index_ >= MAX_DATA_LENGTH)
    {
      buffer_index_ = 0;
      log_warning_("Overflowed receive buffer. Buffer cleared.");
    }

    if (buffer_index_ == 0)
    {  // looking for beginning of message
      if (message[ii] == 0x80)
      {  // beginning of msg found - add to buffer
        data_buffer_[buffer_index_++] = message[ii];
      }
      else
      {
        // log_debug_("BufferIncomingData::Received unknown data.");
      }
    }
    else if (buffer_index_ == 1)
    {  // verify message length
      if (message[ii] == 0x01)
      {  // 2nd byte ok - add to buffer
        data_buffer_[buffer_index_++] = message[ii];
      }
    }
    else if (buffer_index_ == 4)
    {  // completed message
      data_buffer_[buffer_index_++] = message[ii];
      // check checksum and call callback
      uint16_t* message_CRC_position;
      message_CRC_position = (uint16_t*)&data_buffer_[3];
      uint16_t correct_crc = GetCrc((uint8_t*)&data_buffer_, 3, 0xFFFF);

      if (*message_CRC_position == correct_crc)
      {
        log_debug_("Keypress received.");
        if (keypress_callback_)
          keypress_callback_(data_buffer_[2]);
      }
      else
      {
        std::stringstream err_msg;
        err_msg << "Bad checksum. Expected: 0x" << std::hex << (int)correct_crc
                << " Received: " << std::hex << (int)*message_CRC_position;
        log_error_(err_msg.str());
      }
      buffer_index_ = 0;
    }
    else
    {
      data_buffer_[buffer_index_++] = message[ii];
    }
  }  // end for
}
