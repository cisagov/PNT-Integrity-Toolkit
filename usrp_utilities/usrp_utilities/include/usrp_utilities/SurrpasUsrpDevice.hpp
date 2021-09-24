//============================================================================//
//----------------- usrp_utilities/SurrpasUsrpDevice.hpp -------*- C++ -*-----//
//============================================================================//
//
// This file is part of the usrp_utilities library.
//
// The usrp_utilities library is free software: you can redistribute it
// and/or modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation, either version 3 of the License,
// or (at your option) any later version.
//
// The usrp_utilities library is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with The usrp_utilities library.  If not, see
// <https://www.gnu.org/licenses/>.
//
//===----------------------------------------------------------------------===//
///
/// \file
/// \brief    This file contains the declaration of the SurrpasUsrpRecord class.
/// \details
/// \author   Josh Clanton <josh.clanton@is4s.com>
/// \date     April 18, 2018
///
//===----------------------------------------------------------------------===//
#ifndef SURRPAS_USRP_DEVICE_HPP
#define SURRPAS_USRP_DEVICE_HPP

#include <uhd/usrp/multi_usrp.hpp>
#include "if_data_utils/IFSampleData.hpp"
#include "logutils/logutils.hpp"

/// \brief Defining the size of each element within then circular buffer
#define CB_ELEMENT_SIZE 4096

namespace usrp_utilities
{


/// \brief converts utc time in a buffer to a std::time_t
std::time_t utcToSpecT(
  const char* buffer,
  const logutils::LogCallback& log = logutils::printLogToStdOut);

/// \brief A byte array for each element of the buffer
struct circbuff_element
{
  /// A buffer element "chunk" of the larger circular buffer
  char a[CB_ELEMENT_SIZE];
};

/// \brief A defined type for each buffer element
using circbuff_element_t = circbuff_element;

/// \brief function type for retrieving sensor values
using get_sensor_fn_t =
  std::function<uhd::sensor_value_t(const std::string&)>;

// static pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

/// \brief Structure to define Usrp Clock Reference Types
struct UsrpClockRef
{
  /// \brief Defined types for available clock reference types
  enum ClockRefType
  {
    INTERNAL = 0,
    EXTERNAL,
    MIMO,
    GPSDO
  };

  /// \brief Constructor with ClockRefType argument
  ///
  /// \param typeIn The clock reference type
  /// \param log    A log callback for message display (defaults to std::cout)
  UsrpClockRef(const ClockRefType& typeIn       = ClockRefType::INTERNAL,
               const logutils::LogCallback& log = logutils::printLogToStdOut)
    : refType_(typeIn), log_(log){};

  /// \brief Constructor with clock reference type string
  ///
  /// \param typeStrIn A string representing the clock reference type
  /// \param log     A log callback for message display (defaults to std::cout)
  UsrpClockRef(std::string typeStrIn,
               const logutils::LogCallback& log = logutils::printLogToStdOut);

  /// \brief Returns a string representation of the clock reference type
  std::string getTypeStr() const;

  /// \brief The reference type
  ClockRefType refType_;

  /// \brief Local storage for log message callback
  logutils::LogCallback log_;
};

/// \brief Structure to define possible wire formats
struct UsrpWireFormat
{
  /// \brief Possible wire formats
  enum WireFormatType
  {
    SC8 = 0,
    SC16
  };

  /// \brief Constructor with WireFormatType argument
  ///
  /// \param wireFmtIn The provided wire format (defaults to SC16)
  /// \param log    A log callback for message display (defaults to std::cout)
  UsrpWireFormat(const WireFormatType& wireFmtIn  = WireFormatType::SC16,
                 const logutils::LogCallback& log = logutils::printLogToStdOut)
    : wireFmt_(wireFmtIn), log_(log){};

  /// \brief Constructor with wire format type string
  ///
  /// \param typeStrIn A string representing the wire format type
  /// \param log    A log callback for message display (defaults to std::cout)
  UsrpWireFormat(std::string typeStrIn,
                 const logutils::LogCallback& log = logutils::printLogToStdOut);

  /// \brief Returns a string representation of the wire format type
  std::string getTypeStr() const;

  /// \brief The wire format type
  WireFormatType wireFmt_;

  /// \brief Local storage for log message calback
  logutils::LogCallback log_;
};

/// \brief Structure to define possible data types
struct UsrpSampleType
{
  /// \brief Constructor with SampleType argument
  ///
  /// \param type The provided sample type (defaults to SHORT)
  /// \param log    A log callback for message display (defaults to std::cout)
  UsrpSampleType(
    const if_data_utils::IFSampleType& type = if_data_utils::IFSampleType::SC16,
    const logutils::LogCallback& log        = logutils::printLogToStdOut)
    : type_(type), log_(log){};

  /// \brief Constructor with data format string
  ///
  /// \param typeStrIn A string representing data type
  /// \param log    A log callback for message display (defaults to std::cout)
  UsrpSampleType(std::string typeStrIn,
                 const logutils::LogCallback& log = logutils::printLogToStdOut);

  /// \brief Returns a string representation of the data type
  std::string getTypeStr() const;

  /// \brief The data type
  if_data_utils::IFSampleType type_;

  /// \brief Local storage for log message callback
  logutils::LogCallback log_;
};

/// \brief Mode to determine configuration settings
enum class UsrpDeviceMode
{
  RECEIVE = 0,
  TRANSMIT
};

/// \brief Structure for SURRPAS-USRP Record Arguments
struct SurrpasUsrpArgs
{
  /// \brief Constructor for device arguments
  SurrpasUsrpArgs(
    const std::string& deviceArgs,
    const UsrpDeviceMode& mode = UsrpDeviceMode::RECEIVE,
    const double& recordTimeSec         = 30.0,
    const std::vector<double>& freq     = {100e6},
    const std::vector<double>& rate     = {100e3},
    const std::vector<double>& gain     = {0.0},
    const std::vector<std::string>& ant = {std::string()},
    const std::vector<double>& bw = {std::numeric_limits<double>::quiet_NaN()},
    const std::vector<UsrpClockRef>& clockRef = {UsrpClockRef()},
    const std::string& subdev                 = std::string(),
    const std::vector<std::string>& filenames = {"recorded_samples.dat"},
    const UsrpSampleType& type                = UsrpSampleType(),
    const UsrpWireFormat& wireFmt             = UsrpWireFormat(),
    const size_t& numElementsInBuff           = 4096,
    const std::string& startTime              = std::string(),
    const bool& progress                      = false,
    const double& setupTime                   = 1.0,
    const size_t& numSamples                  = 0,  // overrides record time!
    const bool& qcal                          = false,
    const bool& publish                       = false,
    const bool& checkLo                       = true,
    const bool& intnTuning                    = false,
    const bool& stats                         = false,
    const bool& sizemap                       = false,
    const bool& null                          = false,
    const bool& cont                          = true);

  /// \brief Function call to print out an arguments list
  void displayArgs(
    const logutils::LogCallback& log = logutils::printLogToStdOut) const;

  /// \brief Prints the usrp arguments to the screen
  void printArgs(std::stringstream& argPrintOut) const;

  /// The number of channels that will be in the data stream
  size_t numChannels_;
  /// The device args
  std::string args_;
  /// The device mode
  UsrpDeviceMode mode_;
  /// The recording (streaming) duration
  double recordTimeSec_;
  /// The frequency for each channel (numChannels_ long)
  std::vector<double> freq_;
  /// The sample rate of each channel (numChannels_ long)
  std::vector<double> rate_;
  /// The gain for each channel (numChannels_ long)
  std::vector<double> gain_;
  /// The antenna to be used in each channel (numChannels_ long)
  std::vector<std::string> ant_;
  /// The bandwidth of each channel (numChannels_ long)
  std::vector<double> bw_;
  /// The clock reference source for each channel (numChannels_ long)
  std::vector<UsrpClockRef> clockRef_;
  /// THe subdevice id string
  std::string subdev_;
  /// The filenames that will be streamed to (or from)
  std::vector<std::string> filenames_;
  /// The sample type for the data stream
  UsrpSampleType sampleType_;
  /// Format for data on the "wire" in the USRP
  UsrpWireFormat wireFmt_;
  /// Number of elements (chunks) in the circular buffer
  size_t numElementsInBuff_;
  /// String representation of the start time (GPS Time)
  std::string startTime_;
  /// Flag to trigger progress updates during the stream
  bool progress_;
  /// The amount of setup time allowed to the USRP
  double setupTime_;
  /// The total number of samples to be received (superceded by recordTimeSec_)
  size_t numSamples_;
  /// Flag to indicate quantiazation calibratio mode
  bool qcal_;
  /// Flag to indicate publishing of sample snippets (for wrapper nodes)
  bool publish_;
  /// Flag to indicate checking of LO status before prodeeding
  bool checkLo_;
  /// Tells the USRP to use integer tuning
  bool useIntNTuning_;
  /// Flag to indicate publishing of stats
  bool stats_;
  /// Flag to enable size mapping on stream
  bool sizemap_;
  /// Null paramter (for future use)
  bool null_;
  /// Flag to indicate process should continue on a bad packet
  bool continue_;
};

/// \brief Class to represent a USRP device for SURRPAS
class SurrpasUsrpDevice
{
public:
  /// \brief Constructor for SURRPAS USRP Device
  ///
  /// \param args SurrpasUsrpArgs structure for device arguments
  /// \param log    A log callback for message display (defaults to std::cout)
  SurrpasUsrpDevice(
    const SurrpasUsrpArgs& args,
    const logutils::LogCallback& log = logutils::printLogToStdOut);

  /// \brief Function to get a pointer to the UHD usrp object embedded in the
  /// class
  ///
  /// \returns A pointer to the UHD usrp object
  uhd::usrp::multi_usrp::sptr getUsrp();

  /// \brief Function to return the device argument structure embedded in the
  /// class
  ///
  /// \returns The SurrpasUsrpArgs structure
  SurrpasUsrpArgs getUsrpArgs();

  /// \brief Function to retrieve the number of channels configured in the
  /// device
  ///
  /// \returns The number of channels
  size_t getNumChannels();

  /// \brief Function to check if a clock(s) is locked to the designated
  /// reference source
  ///
  /// \returns A bool to indicate lock status
  bool checkLockedSensor(std::vector<std::string> sensor_names,
                         const char* sensor_name,
                         get_sensor_fn_t get_sensor_fn,
                         double setup_time);

private:
  SurrpasUsrpArgs args_;

  uhd::usrp::multi_usrp::sptr usrp_;

  logutils::LogCallback log_;

  size_t numChannels_;
};

//------------------------------------------------------------------------------
//   inline functions
//------------------------------------------------------------------------------
inline uhd::usrp::multi_usrp::sptr SurrpasUsrpDevice::getUsrp()
{
  return usrp_;
}

inline SurrpasUsrpArgs SurrpasUsrpDevice::getUsrpArgs()
{
  return args_;
}

inline size_t SurrpasUsrpDevice::getNumChannels()
{
  return numChannels_;
}

}  // namespace usrp_utilities

//------------------------------------------------------------------------------
//                                  UNCLASSIFIED
//------------------------------------------------------------------------------

#endif
