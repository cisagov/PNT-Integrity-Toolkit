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
// \brief   Class for writing assurance data to csv files
// \author  Anna Levasseur
// \author  <josh.clanton@is4s.com>
// \date    2/22/2021
//============================================================================//
#include "integrity_toolkit/CsvWrite.hpp"

#include <sys/stat.h>   // no clue why required -- man pages say so
#include <sys/types.h>  // required for stat.h
#include <iomanip>      // std::setprecision
#include <iostream>
#include <sstream>

namespace csv_write
{
inline void findAndReplaceAll(std::string& data,
                              std::string  toSearch,
                              std::string  replaceStr)
{
  // Get the first occurrence
  size_t pos = data.find(toSearch);

  // Repeat till end is reached
  while (pos != std::string::npos)
  {
    // Replace this occurrence of Sub String
    data.replace(pos, toSearch.size(), replaceStr);
    // Get the next occurrence from the current position
    pos = data.find(toSearch, pos + replaceStr.size());
  }
}

bool MsgCsvWrite::openFile(std::string relPath,
                           std::string baseFilename,
                           bool        appendDate)
{
  if (file_.is_open())
  {
    log_("Cannot open file.  Already open.", logutils::LogLevel::Warn);
    return false;
  }

  // Use of the local path if no relPath provided
  if (relPath == "")
    relPath_ = "./";
  else
    relPath_ = relPath;

  baseFilename_ = baseFilename;
  appendDate_   = appendDate;

  // mode_t nMode = 0733; // UNIX style permissionscd bi
  int nError = 0;
#if defined(_WIN32)
  // log("Creating windows style directory.", is4s_common::LogLevel::Debug);
  nError = _mkdir(relPath_.c_str());  // can be used on Windows
#else
  // log("Creating linux style directory.", is4s_common::LogLevel::Debug);
  nError = mkdir(relPath_.c_str(),
                 S_IRWXU | S_IRGRP | S_IXGRP);  // can be used on non-Windows
#endif
  if (errno == EEXIST)
  {
    if (verbose_)
    {
      std::cerr << "EEXIST data directory already exists." << relPath_;
    }
  }
  else if (nError != 0)
  {
    // did not successfully create directories
    std::cerr << "Error creating data directory. Logging disabled. " << nError;
    return false;
  }

  try
  {
    std::string curDateTime = getCurrentDateTime();
    findAndReplaceAll(curDateTime, "/", "_");
    findAndReplaceAll(curDateTime, ":", "_");
    // log(curDateTime, is4s_common::LogLevel::Info);
    if (appendDate_)
      filename_ = relPath_ + "/" + baseFilename_ + "_" + curDateTime + ".csv";
    else
      filename_ = relPath_ + "/" + baseFilename_ + ".csv";

    file_.open(filename_.c_str());

    if (verbose_)
    {
      std::stringstream msg;
      msg << "Started log file: " << filename_;
      log_(msg.str(), logutils::LogLevel::Info);
    }
  }
  catch (std::exception& e)
  {
    std::cerr << "Error opening log file: " << e.what() << std::endl;
    closeFile();
    return false;
  }

  return file_.is_open();
}

bool MsgCsvWrite::closeFile()
{
  if (file_.is_open())
  {
    if (verbose_)
    {
      std::stringstream msg;
      msg << "Closing log file: " << filename_;
      log_(msg.str(), logutils::LogLevel::Info);
    }
    file_.close();

    return true;
  }
  else
  {
    return false;
  }
}

std::string toString(pnt_integrity::data::AssuranceState msg)
{
  std::stringstream out;

  out << msg.getName();

  return out.str();
}

std::string toString(pnt_integrity::data::AssuranceReport msg)
{
  std::stringstream out;

  out << toString(msg.header) << "," << msg.state.getIntegerAssuranceValue()
      << "," << msg.state.getAssuranceValue();

  return out.str();
}

std::string toString(pnt_integrity::data::Header msg)
{
  std::stringstream out;
  out << msg.seq_num << "," << msg.timestampArrival.sec << ","
      << msg.timestampArrival.nanoseconds << ","
      << msg.timestampArrival.timecode << "," << msg.timestampValid.sec << ","
      << msg.timestampValid.nanoseconds << "," << msg.timestampValid.timecode
      << "," << msg.deviceId;

  return out.str();
}

std::string toString(pnt_integrity::data::AssuranceReports msg)
{
  std::stringstream out;

  out << toString(msg.header) << "," << msg.numStates;

  for (auto state : msg.states)
  {
    out << "," << state.getName() << "," << state.getWeight() << ","
        << state.getIntegerAssuranceValue();
  }

  return out.str();
}

std::string toString(pnt_integrity::data::AssuranceLevel val)
{
  std::string retVal;
  switch (val)
  {
    case pnt_integrity::data::AssuranceLevel::Unavailable:
      retVal = "Unavailable";
      break;
    case pnt_integrity::data::AssuranceLevel::Unassured:
      retVal = "Unassured";
      break;
    case pnt_integrity::data::AssuranceLevel::Inconsistent:
      retVal = "Inconsistent";
      break;
    case pnt_integrity::data::AssuranceLevel::Assured:
      retVal = "Assured";
      break;
  }
  return retVal;
}

const std::string MsgCsvWrite::getCurrentDateTime()
{
  char buf[80];
  try
  {
    time_t    now = time(0);
    struct tm tstruct;
    tstruct = *localtime(&now);
    // Visit http://www.cplusplus.com/reference/clibrary/ctime/strftime/
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d_%X", &tstruct);
  }
  catch (std::exception& e)
  {
    std::cerr << "Error in currentDateTime: " << e.what();
  }
  return buf;
}

/// \brief  Writer Header to 1st line of CSV file
void MsgCsvWrite::writeHeader(const std::string& line)
{
  // if (headerWritten_)
  //   return;

  // make sure file is open
  if (!file_.is_open())
    return;

  try
  {
    // write line to file
    file_ << line << std::endl;
  }
  catch (std::exception& e)
  {
    std::cerr << "Error writing line to file: " << e.what();
  }
}

void MsgCsvWrite::setVerboseOutput(bool verbose)
{
  verbose_ = verbose;
}

void MsgCsvWrite::writeLine(std::string line)
{
  if (timestampLogs_)
    line = getCurrentDateTime() + "," + line;
  return;
}
void MsgCsvWrite::writeLine(double timestamp, std::string line)
{
  // make sure file is open
  if (!file_.is_open())
    return;

  // add time time to line before writing if desired
  if (timestampLogs_)
    line = getCurrentDateTime() + "," + line;

  try
  {
    // write line to file
    file_ << std::fixed << std::setprecision(9) << timestamp << "," << line
          << std::endl;
  }
  catch (std::exception& e)
  {
    std::cerr << "Error writing line to file: " << e.what();
  }
}

}  // namespace csv_write