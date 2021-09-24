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
/// \brief   Class for writing assurance data to csv files
/// \author  Anna Levasseur
/// \author  <josh.clanton@is4s.com>
/// \date    2/22/2021
//============================================================================//
#ifndef CSV_WRITE__CSV_WRITE_HPP
#define CSV_WRITE__CSV_WRITE_HPP

#include <algorithm>
#include <atomic>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "cf_display/cf_display.h"
#include "cf_display/typedefs.h"
#include "integrity_toolkit/ToolkitLCDDisplay.hpp"
#include "logutils/logutils.hpp"
#include "pnt_integrity/AssuranceCheck.hpp"
#include "pnt_integrity/IntegrityData.hpp"
#include "pnt_integrity/IntegrityMonitor.hpp"

/// Namespace for the csv writer functions
namespace csv_write
{
/// Header string for Assurance Report log file
const std::string AssuranceReportHeaderString(
  "timestamp, seq_num, arrival_sec, arrival_nsec, arrival_timecode, valid_sec, "
  "valid_nsec, valid_timecode, device_id, assurance_level, assurance_score");

/// Header string for Assurance Reports log file
const std::string AssuranceReportsHeaderString(
  "timestamp, seq_num, arrival_sec, arrival_nsec, arrival_timecode, valid_sec, "
  "valid_nsec, valid_timecode, device_id, num_states, [state_name, weight, "
  "assurance_level]");

/// \brief Conversion function for AssuranceState
/// \param msg AssuranceState for conversion
std::string toString(pnt_integrity::data::AssuranceState msg);

/// \brief Conversion function for AssuranceReport
/// \param msg AssuranceReport for conversion
std::string toString(pnt_integrity::data::AssuranceReport msg);

/// \brief Conversion function for Header
/// \param msg Header for conversion
std::string toString(pnt_integrity::data::Header msg);

/// \brief Conversion function for AssuranceReports
/// \param msg AssuranceReports for conversion
std::string toString(pnt_integrity::data::AssuranceReports msg);

/// \brief Conversion function for AssuranceLevel
/// \param val AssuranceLevel for conversion
std::string toString(pnt_integrity::data::AssuranceLevel val);

/// \brief CSV Writer class
class MsgCsvWrite
{
public:
  /// \brief Construct for writer object
  MsgCsvWrite()
    : filename_("")
    , timestampLogs_(false)
    , verbose_(false)
    , log_(logutils::printLogToStdOut)
    , minLogLevel_(logutils::LogLevel::Info){};

  /// \brief Destructor for writer object
  ~MsgCsvWrite()
  {
    // make sure the file is closed before destructing
    closeFile();
  }

  /// \brief Open the file for writing
  ///
  /// Creates a CSV file at the specified location with a file name consisting
  /// of the base filename plus the current date and time.
  /// Format [current_directory]/[relPath]/[baseFilename]????.[extension]
  ///
  /// \param relPath Relative path from the working directory to store log file
  /// \param baseFilename Base portion of file name before date and time
  /// \param appendDate If true, the current data and time is added to the base
  /// filename
  /// \returns true if the file is successfully opened
  bool openFile(std::string relPath,
                std::string baseFilename,
                bool        appendDate = true);

  /// \brief Close the CSV file
  /// \returns true if the log file is successfully closed
  bool closeFile();

  /// \brief Gets the name of the CSV file
  std::string getFilename() { return filename_; };

  /// \brief Indicates if the CSV file has been opened
  /// \returns true if the log file is open
  bool isFileOpen() { return file_.is_open(); };

  /// \brief Enable or disable time stamping of log file lines
  /// \param enabled If true, log file lines are timestamped
  void setTimestampsEnabled(bool enabled) { timestampLogs_ = enabled; };

  // Creates an automatic timestamp for every new csv file line
  template <class Outputmsg>
  void logOutput(Outputmsg msg)
  {
    auto current_time = std::chrono::system_clock::now();
    auto duration_in_seconds =
      std::chrono::duration<double>(current_time.time_since_epoch());
    double timestamp = duration_in_seconds.count();
    writeLine(timestamp, csv_write::toString(msg));
  }

  /// \brief Sets whether verbose output should be used
  /// \param verbose set to true for additional status data to print to stdout
  void setVerboseOutput(bool verbose);

  /// \brief Writes the heaer string to the file
  /// \param line The header string to write
  void writeHeader(const std::string& line);

  // Get the current date and time formatted for use in the filename
  const std::string getCurrentDateTime();

  /// \brief Writes the provied string to the CSV with a timestamp on a unique
  /// line \param timestamp The timestamp associated with the log entry \param
  /// line The string for the log entry
  void writeLine(double timestamp, std::string line);

  /// \brief Writes the provied string to the CSV on a unique line
  /// \param line The string for the log entry
  void writeLine(std::string line);

private:
  // Name of the currently opened log file
  std::string filename_;

  // Filename settings
  std::string relPath_;
  std::string baseFilename_;
  bool        appendDate_;

  // Log file object
  std::ofstream file_;

  // Determines if timestamps are prepended to lines in log file
  bool timestampLogs_;

  // Prevents multiple header lines from being added
  // bool headerWritten_;
  std::atomic<bool> verbose_;

  logutils::LogCallback log_;
  logutils::LogLevel    minLogLevel_;
};

}  // namespace csv_write
#endif