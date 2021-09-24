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
// \file
// \brief    Class definition for main toolkit application
// \author   Josh Clanton <josh.clanton@is4s.com>
// \date     April 20, 2021
//============================================================================//
#include "integrity_toolkit/ToolkitLCDDisplay.hpp"
#include <sstream>

using namespace integrity_toolkit;

ToolkitLCDDisplay::ToolkitLCDDisplay(const std::string& port, int baud_rate)
  : log_(logutils::printLogToStdOut)
{
  initialize(port, baud_rate);
}

bool ToolkitLCDDisplay::initialize(const std::string& port, int baud_rate)
{
  if (display.Initialize(port, baud_rate))
  {
    initialized_ = true;
  }
  return initialized_;
}

void ToolkitLCDDisplay::SetAssuranceLevel(
  const pnt_integrity::data::AssuranceLevel& alevel)
{
  if (!initialized_)
  {
    std::stringstream errMsg;
    errMsg << __FUNCTION__ << ": Display not initialized";
    log_(errMsg.str(), logutils::LogLevel::Error);
    return;
  }

  switch (alevel)
  {
    case pnt_integrity::data::AssuranceLevel::Unavailable: {
      display.ClearDisplay();
      display.SetData("Assurance Level:", 0, 0);
      display.SetData("UNAVAILABLE: 0", 1, 0);
      display.SetLEDColor(3, cf_display::LedColor::LED_COLOR_RED);
      break;
    }
    case pnt_integrity::data::AssuranceLevel::Unassured: {
      display.ClearDisplay();
      display.SetData("Assurance Level:", 0, 0);
      display.SetData("Unassured: 1", 1, 0);
      display.SetLEDColor(2, cf_display::LedColor::LED_COLOR_RED);
      break;
    }
    case pnt_integrity::data::AssuranceLevel::Inconsistent: {
      display.ClearDisplay();
      display.SetData("Assurance Level:", 0, 0);
      display.SetData("Inconsistent: 2", 1, 0);
      display.SetLEDColor(1, cf_display::LedColor::LED_COLOR_YELLOW);
      break;
    }
    case pnt_integrity::data::AssuranceLevel::Assured: {
      display.ClearDisplay();
      display.SetData("Assurance Level:", 0, 0);
      display.SetData("Assured: 3", 1, 0);
      display.SetLEDColor(0, cf_display::LedColor::LED_COLOR_GREEN);
      break;
    }
  }
}
void ToolkitLCDDisplay::displayCheckState(
  pnt_integrity::data::AssuranceState& state)
{
  if (!initialized_)
  {
    std::stringstream errMsg;
    errMsg << __FUNCTION__ << ": Display not initialized";
    log_(errMsg.str(), logutils::LogLevel::Error);
    return;
  }

  display.ClearCheckName();
  std::string new_report  = state.getName();
  std::string check_value = std::to_string(state.getIntegerAssuranceValue());
  display.replaceUnderscore(new_report);
  display.SetData(check_value, 3, 19);
}
