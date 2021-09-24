//============================================================================//
//------------- integrity_ui/toolkitMainApplication.cpp --------*- C++ -*-----//
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
/// \brief    Main for toolkit application
/// \author   Josh Clanton <josh.clanton@is4s.com>
/// \date     April 20, 2020
//============================================================================//
#include "integrity_toolkit/ToolkitApplication.hpp"

/// \brief The main function for the toolkit application
int main(int argc, char* argv[])
{
  integrity_toolkit::ToolkitApplication mainApp(argc, argv);
  return 0;
}