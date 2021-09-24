//============================================================================//
//----------------- usrp_utilities/SurrpasUsrpDevice.cpp -------*- C++ -*-----//
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
//
//   This file contains the definition of the SurrpasUsrpDevice class.
//
//   Josh Clanton <josh.clanton@is4s.com>
//   April 18, 2018
//
//===----------------------------------------------------------------------===//
#include "usrp_utilities/SurrpasUsrpDevice.hpp"
#include <boost/algorithm/string.hpp>
#include <uhd/exception.hpp>
#include <thread>
#include <chrono>

using namespace if_data_utils;
using namespace usrp_utilities;
using namespace logutils;

constexpr std::chrono::system_clock::time_point invalidTime = std::chrono::system_clock::time_point::max();

UsrpClockRef::UsrpClockRef(std::string typeStrIn, const LogCallback& log)
  : log_(log)
{
  std::transform(
    typeStrIn.begin(), typeStrIn.end(), typeStrIn.begin(), ::tolower);

  if (typeStrIn == "internal")
  {
    refType_ = ClockRefType::INTERNAL;
  }
  else if (typeStrIn == "external")
  {
    refType_ = ClockRefType::EXTERNAL;
  }
  else if (typeStrIn == "mimo")
  {
    refType_ = ClockRefType::MIMO;
  }
  else if (typeStrIn == "gpsdo")
  {
    refType_ = ClockRefType::GPSDO;
  }
  else
  {
    std::stringstream errStr;
    errStr << "Invalid clock reference type ' " << typeStrIn << "' ";
    log_(errStr.str(), LogLevel::Error);
  }
}

std::string UsrpClockRef::getTypeStr() const
{
  std::string refTypeStr = "internal";
  switch (refType_)
  {
    case INTERNAL:
      refTypeStr = "internal";
      break;
    case EXTERNAL:
      refTypeStr = "external";
      break;
    case MIMO:
      refTypeStr = "mimo";
      break;
    case GPSDO:
      refTypeStr = "gpsdo";
      break;
  }
  return refTypeStr;
}

UsrpWireFormat::UsrpWireFormat(std::string typeStrIn, const LogCallback& log)
  : log_(log)
{
  std::transform(
    typeStrIn.begin(), typeStrIn.end(), typeStrIn.begin(), ::tolower);

  if (typeStrIn == "sc8")
  {
    wireFmt_ = UsrpWireFormat::SC8;
  }
  else if (typeStrIn == "sc16")
  {
    wireFmt_ = UsrpWireFormat::SC16;
  }
  else
  {
    std::stringstream errStr;
    errStr << "Invalid wire format type '" << typeStrIn << "' ";
    log_(errStr.str(), LogLevel::Error);
  }
}

std::string UsrpWireFormat::getTypeStr() const
{
  std::string wireFmtStr = "sc8";
  switch (wireFmt_)
  {
    case SC8:
      wireFmtStr = "sc8";
      break;
    case SC16:
      wireFmtStr = "sc16";
      break;
  }
  return wireFmtStr;
}

UsrpSampleType::UsrpSampleType(std::string typeStrIn, const LogCallback& log)
  : log_(log)
{
  std::transform(
    typeStrIn.begin(), typeStrIn.end(), typeStrIn.begin(), ::tolower);

  if (typeStrIn == "byte")
  {
    type_ = IFSampleType::SC8;
  }
  else if (typeStrIn == "short")
  {
    type_ = IFSampleType::SC16;
  }
  else if (typeStrIn == "float")
  {
    type_ = IFSampleType::FC32;
  }
  else if (typeStrIn == "double")
  {
    type_ = IFSampleType::FC64;
  }
  else
  {
    std::stringstream errStr;
    errStr << "Invalid sample type '" << typeStrIn << "' ";
    log_(errStr.str(), LogLevel::Error);
  }
}

std::string UsrpSampleType::getTypeStr() const
{
  std::string typeStr = "sc8";
  switch (type_)
  {
    case IFSampleType::SC8:
      typeStr = "sc8";
      break;
    case IFSampleType::SC16:
      typeStr = "sc16";
      break;
    case IFSampleType::FC64:
      typeStr = "fc64";
      break;
    case IFSampleType::FC32:
      typeStr = "fc32";
      break;
  }
  return typeStr;
}

std::time_t usrp_utilities::utcToSpecT(const char* buffer,
                                       const logutils::LogCallback& log)
{
  struct tm newtime = {0,0,0,1,0,1900,0,0,0,0,0};
  strptime(buffer, "%F %H:%M:%S %z", &newtime);

  // adjust daylight time saving
  newtime.tm_isdst = -1;

  // utc to time_t
  std::time_t std_time = mktime(&newtime);

  // Print out starting time in future
  std::stringstream startStr;
  startStr << "Requested Start time: " << ctime(&std_time);
  log(startStr.str(), LogLevel::Info);

  return std_time;
}

/// \brief Constructor for device arguments
SurrpasUsrpArgs::SurrpasUsrpArgs(
  const std::string& deviceArgs,
  const UsrpDeviceMode& mode,
  const double& recordTimeSec,
  const std::vector<double>& freq,
  const std::vector<double>& rate,
  const std::vector<double>& gain,
  const std::vector<std::string>& ant,
  const std::vector<double>& bw,
  const std::vector<UsrpClockRef>& clockRef,
  const std::string& subdev,
  const std::vector<std::string>& filenames,
  const UsrpSampleType& type,
  const UsrpWireFormat& wireFmt,
  const size_t& numElementsInBuff,
  const std::string& startTime,
  const bool& progress,
  const double& setupTime,
  const size_t& numSamples,  // overrides record time!
  const bool& qcal,
  const bool& publish,
  const bool& checkLo,
  const bool& intnTuning,
  const bool& stats,
  const bool& sizemap,
  const bool& null,
  const bool& cont)
  : args_(deviceArgs)
  , mode_(mode)
  , recordTimeSec_(recordTimeSec)
  , freq_(freq)
  , rate_(rate)
  , gain_(gain)
  , ant_(ant)
  , bw_(bw)
  , clockRef_(clockRef)
  , subdev_(subdev)
  , filenames_(filenames)
  , sampleType_(type)
  , wireFmt_(wireFmt)
  , numElementsInBuff_(numElementsInBuff)
  , startTime_(startTime)
  , progress_(progress)
  , setupTime_(setupTime)
  , numSamples_(numSamples)
  , qcal_(qcal)
  , publish_(publish)
  , checkLo_(checkLo)
  , useIntNTuning_(intnTuning)
  , stats_(stats)
  , sizemap_(sizemap)
  , null_(null)
  , continue_(cont)
{
  std::vector<std::string> subdevices;
  boost::split(subdevices, subdev_, boost::is_any_of(" "));
  numChannels_ = subdevices.size();  //
}

void SurrpasUsrpArgs::displayArgs(const logutils::LogCallback& log) const
{
  std::stringstream argStr;
  printArgs(argStr);
  log(argStr.str(), LogLevel::Info);
}

void SurrpasUsrpArgs::printArgs(std::stringstream& argPrintOut) const
{
  std::string modeStr = ((int)mode_ == 0) ? "RECEIVE" : "TRANSMIT"; 

  argPrintOut << "SURRPAS USRP Device Arguments:" << std::endl

              << "args:              " << args_ << std::endl
              << "mode:              " << modeStr << std::endl
              << "recordTimeSec:     " << recordTimeSec_ << std::endl
              << "subdev:            " << subdev_ << std::endl
              << "sampleType:        " << sampleType_.getTypeStr() << std::endl
              << "wireFmt:           " << wireFmt_.getTypeStr() << std::endl
              << "numElementsInBuff: " << numElementsInBuff_ << std::endl
              << "startTime:         " << startTime_ << std::endl
              << "progress :         " << progress_ << std::endl
              << "setupTime:         " << setupTime_ << std::endl
              << "numSamples:        " << numSamples_ << std::endl
              << "checkLo:           " << checkLo_ << std::endl
              << "useIntNTuning:     " << useIntNTuning_ << std::endl
              << "stats:             " << stats_ << std::endl
              << "sizemap:           " << sizemap_ << std::endl
              << "null:              " << null_ << std::endl
              << "continue:          " << continue_ 
              << std::endl;

  for (size_t ii = 0; ii < numChannels_; ++ii)
  {
    argPrintOut << "----------Channel[" << ii << "]----------" << std::endl
                << "freq:              " << freq_[ii] << std::endl
                << "rate:              " << rate_[ii] << std::endl
                << "gain:              " << gain_[ii] << std::endl
                << "ant:               " << ant_[ii] << std::endl
                << "bw:                " << bw_[ii] << std::endl
                << "clockRef:          " << clockRef_[ii].getTypeStr()
                << std::endl;
  }

  argPrintOut << "-------Associated Files---------" << std::endl;
  for (auto fileIt = filenames_.begin(); fileIt != filenames_.end(); ++fileIt)
  {
    argPrintOut << "file: " << *fileIt << std::endl;
  }
}

SurrpasUsrpDevice::SurrpasUsrpDevice(const SurrpasUsrpArgs& deviceArgs,
                                     const logutils::LogCallback& log)
  : args_(deviceArgs), log_(log)
{
  deviceArgs.displayArgs(log);

  // create a usrp device
  std::stringstream argStr;
  argStr << "Creating the usrp device with: " << std::endl << deviceArgs.args_;
  log_(argStr.str(), LogLevel::Info);

  usrp_ = uhd::usrp::multi_usrp::make(args_.args_);

  //  std::vector<std::string> subdevices;
  //  boost::split(subdevices,deviceArgs.subdev_,boost::is_any_of(" "));
  //  numChannels_ = subdevices.size();// * numMotherboards_;
  numChannels_ = deviceArgs.numChannels_;

  // always select the subdev first, the channel map affects the other settings
  if (!deviceArgs.subdev_.empty())
  {
    switch (args_.mode_)
    {
      case UsrpDeviceMode::RECEIVE:
          usrp_->set_rx_subdev_spec(uhd::usrp::subdev_spec_t(deviceArgs.subdev_));
          break;
      case UsrpDeviceMode::TRANSMIT:
          usrp_->set_tx_subdev_spec(uhd::usrp::subdev_spec_t(deviceArgs.subdev_));
      break;
    }
  }
  else
  {
    log_("No 'subdev' argument found. Default channel mapping will be used",
         LogLevel::Warn);
  }

  for (size_t ii = 0; ii < numChannels_; ++ii)
  {
    std::stringstream channelStr;
    channelStr << std::endl
               << "============================================================"
                  "=========="
               << std::endl
               << "Setting Arguments for Channel : " << ii << std::endl
               << "============================================================"
                  "==========";
    log_(channelStr.str(), LogLevel::Info);

    std::stringstream deviceStr;
    deviceStr << "Using Device: " << usrp_->get_pp_string();
    log_(deviceStr.str(), LogLevel::Info);

    // Lock mboard clocks
    try
    {
      usrp_->set_clock_source(args_.clockRef_[ii].getTypeStr(), ii);
    }
    catch (uhd::index_error e)
    {
      std::stringstream msg;
      msg << "Could not set clock source for mboard " << ii
          << ". This is not necessarily a problem. Check your device "
             "configuration";
      log_(msg.str(), LogLevel::Warn);
    }

    // set the sample rate
    if (deviceArgs.rate_[ii] <= 0.0)
    {
      log_("Please specify a valid sample rate", LogLevel::Info);
    }
    else
    {
      std::stringstream rateStr1;
      rateStr1 << "Setting RX Rate: " << deviceArgs.rate_[ii] / 1e6 << " Msps";
      log_(rateStr1.str(), LogLevel::Info);

      switch (args_.mode_)
      {
        case UsrpDeviceMode::RECEIVE:
            usrp_->set_rx_rate(deviceArgs.rate_[ii], ii);
            break;
        case UsrpDeviceMode::TRANSMIT:
            usrp_->set_tx_rate(deviceArgs.rate_[ii], ii);
        break;
      }
      // set the rate

      std::stringstream rateStr2;
      rateStr2 << "Actual Rx Rate: " << usrp_->get_rx_rate(ii) / 1e6 << " Msps";
      log_(rateStr2.str(), LogLevel::Info);
    }

    // set the center frequency
    std::stringstream freqMsg;
    freqMsg << "Setting RX Freq:" << deviceArgs.freq_[ii] / 1e6 << " MHz...";
    log_(freqMsg.str(), LogLevel::Info);
    uhd::tune_request_t tune_request(deviceArgs.freq_[ii]);

    if (deviceArgs.useIntNTuning_)
    {
      tune_request.args = uhd::device_addr_t("mode_n=integer");
    }
    
    switch (args_.mode_)
    {
      case UsrpDeviceMode::RECEIVE:
        usrp_->set_rx_freq(tune_request, ii);
        break;
      case UsrpDeviceMode::TRANSMIT:
        usrp_->set_tx_freq(tune_request, ii);
        break;
    }

    std::stringstream tuneMsg;
    tuneMsg << "Actual RX Freq: " << usrp_->get_rx_freq(ii) / 1e6 << "MHz...";
    log_(tuneMsg.str(), LogLevel::Info);

    // set the rf gain
    std::stringstream gainStr1;
    gainStr1 << "Setting RX Gain: " << deviceArgs.gain_[ii] << " dB";
    log_(gainStr1.str(), LogLevel::Info);

    // set the rate
    switch (args_.mode_)
    {
      case UsrpDeviceMode::RECEIVE:
        usrp_->set_rx_gain(deviceArgs.gain_[ii], ii);
        break;
      case UsrpDeviceMode::TRANSMIT:
        usrp_->set_tx_gain(deviceArgs.gain_[ii], ii);
        break;
    }


    std::stringstream gainStr2;
    gainStr2 << "Actual Rx Gain: " << usrp_->get_rx_gain(ii) << " dB";
    log_(gainStr2.str(), LogLevel::Info);

    // set the IF filter bandwidth
    if (!std::isnan(deviceArgs.bw_[ii]))
    {
      // TODO: Change to logutils
      // set the rf gain
      std::stringstream bwStr1;
      bwStr1 << "Setting RX Bandwidth: " << deviceArgs.bw_[ii] / 1e6 << " MHz";
      log_(bwStr1.str(), LogLevel::Info);

     // set the rate
    switch (args_.mode_)
    {
      case UsrpDeviceMode::RECEIVE:
        usrp_->set_rx_bandwidth(deviceArgs.bw_[ii], ii);
        break;
      case UsrpDeviceMode::TRANSMIT:
        usrp_->set_tx_bandwidth(deviceArgs.bw_[ii], ii);
        break;
    }

      std::stringstream bwStr2;
      bwStr2 << "Actual Rx Bandwith: " << usrp_->get_rx_bandwidth(ii) / 1e6
             << " MHz";
      log_(bwStr2.str(), LogLevel::Info);
    }

    // set the antenna
    if (!deviceArgs.ant_.empty())
    {
      // TODO: Add a logutils message about setting antenna
      std::stringstream antStr;
      antStr << "Setting Antenna to: " << deviceArgs.ant_[ii];
      log_(antStr.str(), LogLevel::Info);

      switch (args_.mode_)
      {
        case UsrpDeviceMode::RECEIVE:
          usrp_->set_rx_antenna(deviceArgs.ant_[ii], ii);
          break;
        case UsrpDeviceMode::TRANSMIT:
          usrp_->set_tx_antenna(deviceArgs.ant_[ii], ii);
          break;
      }

    }
  }
  // allow for some setup time
 std::this_thread::sleep_for(std::chrono::seconds((int)deviceArgs.setupTime_));

  // check Ref and LO Lock detect
  if (deviceArgs.checkLo_)
  {
    for (size_t ii = 0; ii < numChannels_; ++ii)
    {
      checkLockedSensor(
        usrp_->get_rx_sensor_names(0),
        "lo_locked",
        boost::bind(&uhd::usrp::multi_usrp::get_rx_sensor, usrp_, _1, 0),
        deviceArgs.setupTime_);

      if (deviceArgs.clockRef_[ii].refType_ == UsrpClockRef::MIMO)
      {
        checkLockedSensor(
          usrp_->get_mboard_sensor_names(0),
          "mimo_locked",
          boost::bind(&uhd::usrp::multi_usrp::get_mboard_sensor, usrp_, _1, 0),
          deviceArgs.setupTime_);
      }
      else if (deviceArgs.clockRef_[ii].refType_ == UsrpClockRef::EXTERNAL)
      {
        checkLockedSensor(
          usrp_->get_mboard_sensor_names(0),
          "ref_locked",
          boost::bind(&uhd::usrp::multi_usrp::get_mboard_sensor, usrp_, _1, 0),
          deviceArgs.setupTime_);
      }
    }
  }

  // Get sensor names from the usrp
  std::vector<std::string> sensor_names = usrp_->get_mboard_sensor_names(0);

  // Set the USRP initial time
  if (std::find(sensor_names.begin(), sensor_names.end(), "gps_locked") !=
      sensor_names.end())
  {
    uhd::sensor_value_t gps_locked = usrp_->get_mboard_sensor("gps_locked", 0);
    if (gps_locked.to_bool())
    {
      uhd::sensor_value_t gps_time = usrp_->get_mboard_sensor("gps_time");
      uhd::time_spec_t usrp_time(gps_time.to_real());
      usrp_->set_time_now(usrp_time);
      time_t stdtime = gps_time.to_real();

      std::stringstream timeSetMsg;
      timeSetMsg << "Set USRP with GPS time: " << ctime(&stdtime);
      log_(timeSetMsg.str(), LogLevel::Info);
    }
    else
    {
      log_("Found GPSDO but no GPS lock, setting usrp time to system time",
           LogLevel::Warn);
      time_t pc_time = time(0);
      usrp_->set_time_now(uhd::time_spec_t(pc_time, 0.0));

      std::stringstream pcTimeStr;
      pcTimeStr << "Set USRP time with PC system time: " << ctime(&pc_time);
      log_(pcTimeStr.str(), LogLevel::Warn);
    }
  }
  else
  {
    log_("No GPSDO found, setting usrp time to system time", LogLevel::Warn);

    time_t pc_time = time(0);
    usrp_->set_time_now(uhd::time_spec_t(pc_time, 0.0));

    std::stringstream pcTimeStr;
    pcTimeStr << "Set USRP time with PC system time: " << ctime(&pc_time);

    log_(pcTimeStr.str(), LogLevel::Warn);
  }
}

bool SurrpasUsrpDevice::checkLockedSensor(std::vector<std::string> sensor_names,
                                          const char* sensor_name,
                                          get_sensor_fn_t get_sensor_fn,
                                          double setup_time)
{
  if (std::find(sensor_names.begin(), sensor_names.end(), sensor_name) ==
      sensor_names.end())
    return false;

  std::chrono::system_clock::time_point first_lock_time = invalidTime;

  std::stringstream waitStr;
  waitStr << "Waiting for \" " << sensor_name << " \" : ";
  log_(waitStr.str(), LogLevel::Info);

  while (true)
  {
    auto time_since_first_lock = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - first_lock_time).count();
    double time_since_first_lock_sec = static_cast<double>(time_since_first_lock) / 1000.;

    if ((first_lock_time!=invalidTime) && (time_since_first_lock_sec > setup_time))
    {
      log_(" locked.", LogLevel::Info);
      break;
    }

    if (get_sensor_fn(sensor_name).to_bool())
    {
      if (first_lock_time==invalidTime)
        first_lock_time =std::chrono::system_clock::now();
      std::cout << "+";
      std::cout.flush();
    }
    else
    {
      first_lock_time = invalidTime;  // reset to invalid time value

      if (time_since_first_lock_sec >setup_time)
      {
        std::cout << std::endl;
        std::stringstream errStr;
        errStr << "Timed out waiting for consecutive locks on sensor \" "
               << sensor_name << "\"";
        log_(errStr.str(), LogLevel::Error);
        throw std::runtime_error(sensor_name);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

  }
  return true;
}
