//============================================================================//
//----------------- integrity_ui/ToolkitApplication.hpp --------*- C++ -*-----//
//============================================================================//
// This file is part of the integrity_ui library.
//
// The integrity_ui library is free software: you can redistribute it
// and/or modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation, either version 3 of the License,
// or (at your option) any later version.
//
// The integrity_ui library is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with The integrity_ui library.  If not, see
// <https://www.gnu.org/licenses/>.
//----------------------------------------------------------------------------//
/// \file
/// \brief    Class definition for main toolkit application
/// \author   Josh Clanton <josh.clanton@is4s.com>
/// \date     April 20, 2021
//============================================================================//
#ifndef INTEGRITY_TOOLKIT__TOOLKIT_LCD_DISPLAY_HPP
#define INTEGRITY_TOOLKIT__TOOLKIT_LCD_DISPLAY_HPP

#include "cf_display/cf_display.h"
#include "cf_display/typedefs.h"
#include "logutils/logutils.hpp"
#include "pnt_integrity/IntegrityData.hpp"
namespace integrity_toolkit
{
/// Container class for interacting with the Crystal Fontz LCD display in the
/// toolkit
class ToolkitLCDDisplay
{
public:
  /// \brief Construct for the LCD object
  /// \param port The serial port for the device
  /// \param baudRate The baud rate for serial comms with the device
  ToolkitLCDDisplay(const std::string& port, int baudRate);

  /// \brief Default constructor for the LCD Object
  ToolkitLCDDisplay() : initialized_(false), log_(logutils::printLogToStdOut){};

  /// \brief Initializes the LCD display with provided port and baud rate
  /// \param port The serial port for the device
  /// \param baudRate The baud rate for serial comms with the device
  bool initialize(const std::string& port, int baudRate);

  /// \brief  Sets and displays the total assurance level on the display
  /// \param level The current total assurance level
  void SetAssuranceLevel(const pnt_integrity::data::AssuranceLevel& level);

  /// \brief Displays the assurance level of a particular state lower on the
  /// display
  /// \param state The provided Assurance State
  void displayCheckState(pnt_integrity::data::AssuranceState& state);

  /// \brief Destructor for the LCD display object
  ~ToolkitLCDDisplay() { display.Shutdown(); };

  /// \brief Clears display
  void clearDisplay()
  {
    display.ClearDisplay();
  }

  /// \brief Clears LEDs
  void clearLeds()
  {
    display.ClearAllLeds();
  }

private:
  cf_display::CfDisplay display;

  bool initialized_;

  logutils::LogCallback log_;
};
}  // namespace integrity_toolkit

#endif