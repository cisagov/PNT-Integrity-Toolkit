//==---------------- src/ublox.cpp - Ublox class definition --------------===//
// BSD 3-Clause License
//
// Copyright (C) 2019 Integrated Solutions for Systems, Inc
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
// may be used to endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//===---------------------------------------------------------------------===//
// This file contains the declaration of the Ublox class.
//
// Chris Collins <chris.collins@is4s.com>
// December 2015
//
//===---------------------------------------------------------------------===//

#include "ublox/ublox.h"
#include <iomanip>
#include <iostream>
#include <thread>

namespace ublox
{
inline void printHex(char* data, int length)
{
  for (int i = 0; i < length; ++i)
  {
    printf("0x%.2X ", (unsigned)(unsigned char)data[i]);
  }
  printf("\n");
}

void DefaultAcknowledgementHandler()
{
  // std::cout << "Acknowledgment received." << std::endl;
}

inline void DefaultPortSettingsCallback(const CfgPrt          port_settings,
                                        const SysClkTimePoint& time_stamp)
{
  std::cout << "CFG-PRT:" << std::endl;
}

inline void DefaultConfigureNavigationParametersCallback(
  const CfgNav5         cfg_nav,
  const SysClkTimePoint& time_stamp)
{
  std::cout << "CFG-NAV5:" << std::endl;
}

inline void DefaultErrorMsgCallback(const std::string& msg)
{
  std::cout << "Ublox Error: " << msg << std::endl;
}

inline void DefaultNavSolCallback(const ublox::NavSol   nav_sol,
                                  const SysClkTimePoint& time_stamp)
{
  std::cout << "NAV-SOL: " << std::endl;
  std::cout << "   Fix Type: " << (int)nav_sol.gpsFix
            << " # SVs: " << (int)nav_sol.numSV << std::endl;
  std::cout << "   X: " << nav_sol.ecefX << " Y:" << nav_sol.ecefY
            << " Z:" << nav_sol.ecefZ << std::endl;
}

inline void DefaultNavPvtCallback(const ublox::NavPvt   nav_pvt,
                                  const SysClkTimePoint& time_stamp)
{
  std::cout << "NAV-PVT: " << std::endl;
  std::cout << "   Fix Type: " << (int)nav_pvt.gpsFix
            << " # SVs: " << (int)nav_pvt.numSV << std::endl;

  std::cout << "  " << (int)nav_pvt.month << "/" << (int)nav_pvt.day << "/"
            << (int)nav_pvt.year << "  " << (int)nav_pvt.hour << ":"
            << (int)nav_pvt.min << ":" << (int)nav_pvt.sec << std::endl;
  std::cout << "  GPS milliseconds: " << nav_pvt.iTOW << std::endl
            << "  Latitude: " << nav_pvt.latitude_scaled * LAT_LONG_SCALING
            << std::endl
            << "  Longitude: " << nav_pvt.longitude_scaled * LAT_LONG_SCALING
            << std::endl
            << "  Height: " << nav_pvt.height * MM_TO_M
            << "  Height MSL: " << nav_pvt.height_mean_sea_level * MM_TO_M
            << "  Speed: N: " << nav_pvt.velocity_north * MM_TO_M
            << " E: " << nav_pvt.velocity_east * MM_TO_M
            << " D: " << nav_pvt.velocity_down * MM_TO_M << std::endl;
}

inline void DefaultNavStatusCallback(const ublox::NavStatus nav_status,
                                     const SysClkTimePoint&  time_stamp)
{
  std::cout << "GPS Fix Type: ";
  if (nav_status.fixtype == 0x00)
  {
    std::cout << "No Fix" << std::endl;
    std::cout << "TTFF: "
              << " none ms" << std::endl;
    std::cout << "Milliseconds since Startup/Reset: " << nav_status.msss
              << std::endl;
  }
  else if (nav_status.fixtype == 0x01)
  {
    std::cout << "Dead Reckoning Only" << std::endl;
  }
  else if (nav_status.fixtype == 0x02)
  {
    std::cout << "2D Fix" << std::endl;
  }
  else if (nav_status.fixtype == 0x03)
  {
    std::cout << "3D Fix" << std::endl;
  }
  else if (nav_status.fixtype == 0x04)
  {
    std::cout << "GPS + Dead Reckoning" << std::endl;
  }
  else if (nav_status.fixtype == 0x05)
  {
    std::cout << "Time Only" << std::endl;
  }
  else
  {
    std::cout << std::endl;
  }

  if (nav_status.fixtype != 0x00)
  {
    std::cout << "TTFF: " << (nav_status.ttff / 1000.) << " sec" << std::endl;
    std::cout << "Milliseconds since Startup/Reset: "
              << (nav_status.msss / 1000.) << " sec" << std::endl;
  }
}

inline void DefaultNavVelNedCallback(const ublox::NavVelNed nav_vel_ned,
                                     const SysClkTimePoint&  time_stamp)
{
  std::cout << "NAV-VELNED: " << std::endl;
}

inline void DefaultNavSVInfoCallback(const ublox::NavSVInfo nav_sv_info,
                                     const SysClkTimePoint&  time_stamp)
{
  std::cout << "NAV-SVINFO: " << std::endl;
}

inline void DefaultNavSatCallback(const ublox::NavSat   nav_sat,
                                  const SysClkTimePoint& time_stamp)
{
  std::cout << "NAV-SAT: "
            << " Time: " << nav_sat.iTOW << " Num SVs: " << (int)nav_sat.numSvs
            << std::endl;
}

inline void DefaultNavSigCallback(const ublox::NavSig   navSig,
                                  const SysClkTimePoint& time_stamp)
{
  std::cout << "NAV-SIG: "
            << " Time: " << navSig.iTOW
            << " Num Signals: " << (int)navSig.numSigs << std::endl;
}

inline void DefaultNavGPSTimeCallback(const ublox::NavGPSTime nav_gps_time,
                                      const SysClkTimePoint&   time_stamp)
{
  std::cout << "NAV-GPSTIME: " << std::endl;
}

inline void DefaultNavUTCTimeCallback(const ublox::NavUTCTime nav_utc_time,
                                      const SysClkTimePoint&   time_stamp)
{
  std::cout << "NAV-UTCTIME: " << std::endl;
}

inline void DefaultNavDOPCallback(const ublox::NavDOP   nav_dop,
                                  const SysClkTimePoint& time_stamp)
{
  std::cout << "NAV-DOP: " << std::endl;
}

inline void DefaultNavDGPSCallback(const ublox::NavDGPS  nav_dgps,
                                   const SysClkTimePoint& time_stamp)
{
  std::cout << "NAV-DGPS: " << std::endl;
}

inline void DefaultNavClockCallback(const ublox::NavClock nav_clock,
                                    const SysClkTimePoint& time_stamp)
{
  std::cout << "NAV-CLK: " << std::endl;
}

inline void DefaultNavPosEcefCallback(const ublox::NavPosECEF position,
                                      const SysClkTimePoint&   time_stamp)
{
  std::cout << "NAV-POSECEF: " << std::endl;
}

inline void DefaultNavPosLlhCallback(const ublox::NavPosLLH nav_position,
                                     const SysClkTimePoint&  time_stamp)
{
  /*std:: cout << "NAV-POSLLH: " <<std::endl <<
                "  GPS milliseconds: " << nav_position.iTOW << std::endl <<
                "  Latitude: " << nav_position.latitude_scaled << std::endl <<
                "  Longitude: " << nav_position.longitude_scaled << std::endl <<
                "  Height: " << nav_position.height << std::endl << std::endl;*/
}

inline void DefaultAidEphCallback(const ublox::EphemSV  eph_sv,
                                  const SysClkTimePoint& time_stamp)
{
  std::cout << "AID-EPH: " << std::endl;
}

inline void DefaultAidAlmCallback(const ublox::AlmSV    alm_sv,
                                  const SysClkTimePoint& time_stamp)
{
  std::cout << "AID-ALM: " << std::endl;
}

inline void DefaultAidHuiCallback(const ublox::AidHui   aid_hui,
                                  const SysClkTimePoint& time_stamp)
{
  std::cout << "AID-HUI: " << std::endl;
}

inline void DefaultAidIniCallback(const ublox::AidIni   aid_ini,
                                  const SysClkTimePoint& time_stamp)
{
  std::cout << "AID-INI: " << std::endl;
}

inline void DefaultRxmRawCallback(const ublox::RawMeas  raw_meas,
                                  const SysClkTimePoint& time_stamp)
{
  std::cout << "RXM-RAW: " << std::endl;
}

inline void DefaultRxmxRawCallback(const ublox::RawMeasX raw_measx,
                                   const SysClkTimePoint& time_stamp)
{
  std::cout << "RXM-RAWX: " << (int)raw_measx.numMeas
            << " Tow: " << raw_measx.rcvTow << std::endl;

  for (int ii = 0; ii < raw_measx.numMeas; ii++)
  {
    std::cout << "Svid: " << (int)raw_measx.rawxmeasreap[ii].svid
              << " GNSS ID: " << (int)raw_measx.rawxmeasreap[ii].gnssId
              << " Psr: " << raw_measx.rawxmeasreap[ii].psuedorange
              << std::endl;
  }
}

inline void DefaultRxmSubframeCallback(const ublox::SubframeData subframe,
                                       const SysClkTimePoint&     time_stamp)
{
  std::cout << "RXM-SFRB: " << std::endl;
}
inline void DefaultRxmSubframeXCallback(const ublox::SubframeDataX subframe,
                                        const SysClkTimePoint&      time_stamp)
{
  std::cout << "RXM-SFRBX: " << std::endl;
}

inline void DefaultRxmSvsiCallback(const ublox::SVStatus sv_stat,
                                   const SysClkTimePoint& time_stamp)
{
  std::cout << "RXM-SVSI: " << std::endl;
}

inline void DefaultParsedEphemCallback(
  const ublox::ParsedEphemData parsed_ephem_data,
  const SysClkTimePoint&        time_stamp)
{
  /*
  std::cout << "Parsed ephemeris: " << std::endl;
  //Display Parsed Eph Data:
  cout << "PRN: " << parsed_ephem_data.prn << std::endl;
  cout << "T_GD: " << parsed_ephem_data.tgd << std::endl;
  cout << "t_oc: " << parsed_ephem_data.toc << std::endl;
  cout << "af0: " << parsed_ephem_data.af0 << std::endl;
  cout << "af1: " << parsed_ephem_data.af1 << std::endl;
  cout << "af2: " << parsed_ephem_data.af2 << std::endl;
  cout << "M_0: " << parsed_ephem_data.anrtime << std::endl;
  cout << "deltan: " << parsed_ephem_data.dN << std::endl;
  cout << "ecc: " << parsed_ephem_data.ecc << std::endl;
  cout << "sqrtA: " << parsed_ephem_data.majaxis << std::endl;
  cout << "OMEGA_0: " << parsed_ephem_data.wo << std::endl;
  cout << "i_0: " << parsed_ephem_data.ia << std::endl;
  cout << "Omega: " << parsed_ephem_data.omega << std::endl;
  cout << "Omega dot: " << parsed_ephem_data.dwo << std::endl;
  cout << "IDOT: " << parsed_ephem_data.dia << std::endl;
  cout << "C_uc: " << parsed_ephem_data.cuc << std::endl;
  cout << "C_us: " << parsed_ephem_data.cus << std::endl;
  cout << "C_rc: " << parsed_ephem_data.crc << std::endl;
  cout << "C_rs: " << parsed_ephem_data.crs << std::endl;
  cout << "C_is: " << parsed_ephem_data.cis << std::endl;
  cout << "t_oe: " << parsed_ephem_data.toe << std::endl;
  cout << "----------------------------------" << std::endl;
  cout << std::endl;
*/
}

inline void DefaultMonitorGNSS(const ublox::MonGnss&  status,
                               const SysClkTimePoint& time_stamp)
{
  std::cout << "MON-GNSS: " << std::endl;
  ;
  // TODO populate message with relevant data
}

inline void DefaultMonitorHardware(const ublox::MonHw&    status,
                                   const SysClkTimePoint& time_stamp)
{
  std::cout << "MON-HW: " << std::endl;
  ;
  // TODO populate message with relevant data
}

inline void DefaultMonitorExtendedHardware(const ublox::MonHw2&   status,
                                           const SysClkTimePoint& time_stamp)
{
  std::cout << "MON-HW2: " << std::endl;
  ;
  // TODO populate message with relevant data
}

inline void DefaultMonitorVersionCallback(const ublox::MonVer&   version,
                                          const SysClkTimePoint& time_stamp)
{
  std::cout << "MON_VER received: " << std::endl;
  ;
  // TODO populate message with relevant data
}

// Default message display for MON-SPEC message
inline void DefaultMonSpanCallback(const ublox::MonSpan&  monSpan,
                                   const SysClkTimePoint& time_stamp)
{
  std::cout << "MON_SPAN received: Payload Size = "
            << monSpan.header.payload_length << std::endl
            << "  numRfBlocks = " << monSpan.numRfBlocks << std::endl
            << "  version = 0x" << std::hex << monSpan.version << std::dec
            << std::endl;

  for (std::vector<MonSpanData>::const_iterator it = monSpan.data.begin();
       it != monSpan.data.end();
       ++it)
  {
    std::cout << "  =============" << std::endl
              << "  span = " << it->span << " Hz" << std::endl
              << "  res = " << it->resolution << " Hz" << std::endl
              << "  center = " << it->center << " Hz" << std::endl
              << "  pga = " << it->pga << " dB" << std::endl;
  }
}

// Default message display for MON-RF message
inline void DefaultMonRfCallback(const ublox::MonRf&    monRf,
                                 const SysClkTimePoint& time_stamp)
{
  std::cout << "MON_RF received: Num Blocks = "
            << static_cast<int>(monRf.numBlocks);
  for (uint8_t ii = 0; ii < monRf.numBlocks; ii++)
  {
    std::cout << ii << " : blockid = " << (int)monRf.rfBlocks[ii].blockId
              << " : agc = " << monRf.rfBlocks[ii].agcCnt
              << " ant: " << static_cast<int>(monRf.rfBlocks[ii].antennaStatus)
              << " jamInd: "
              << static_cast<int>(monRf.rfBlocks[ii].jammingIndicator)
              << "  |   ";
  }
  std::cout << std::endl;
}

inline void DefaultRawSerialDataCallback(const char* buffer, const int length)
{
  // std::cout << "Raw serial data available ..." << std::endl;
}
inline void DefaultNavHPPosEcefCallback(const ublox::NavHPPosECEF pos,
                                        const SysClkTimePoint&     time_stamp)
{
  std::cout << "NAV-HPPOSECEF: " << std::endl;
}
inline void DefaultNavHPPosLlhCallback(const ublox::NavHPPosLLH pos,
                                       const SysClkTimePoint&    time_stamp)
{
  std::cout << "NAV-HPPOSLLH: " << std::endl;
}

Ublox::Ublox()
  : isConnected_(false)
  , reading_(false)
  , stopReading_(false)
  , activeMonitorConnectionThread_(false)
  , stopMonitorConnectionThread_(false)
  , serialPort_(nullptr)
  , protocolUbxIn_(true)
  , protocolUbxOut_(true)
  , protocolNmeaIn_(false)
  , protocolNmeaOut_(false)
  , dynamicModel_("automotive")
  , fixMode_("auto")
  , cfgItfmEnable_(false)
  , cfgItfmBbJamThreshold_(3)
  , cfgItfmCwJamThreshold_(15)
  , monitorConnectionTimeout_(0)
  , monitorConnectionPeriod_(1)
  , receiverAgpsRequestPeriod_(0)
  , port_("")
  , baudrate_(115200)
  , pollForPvt_(false)
  , pollForEphem_(false)
  , pollForAlmanac_(false)
  , pollForUtcIono_(false)
  , pollForRaw_(false)
  , log_(logutils::printLogToStdOut)
  // handleAcknowledgement_(DefaultAcknowledgementHandler),
  // portSettingsHandler_(DefaultPortSettingsCallback),
  // navigationParametersHandler_(DefaultConfigureNavigationParametersCallback),
  // navPosEcefHandler_(DefaultNavPosEcefCallback),
  // navPosLlhHandler_(DefaultNavPosLlhCallback),
  // navSolHandler_(DefaultNavSolCallback),
  // navPvtHandler_(DefaultNavPvtCallback),
  // navStatusHandler_(DefaultNavStatusCallback),
  // navVelNedHandler_(DefaultNavVelNedCallback),
  // navSvInfoHandler_(DefaultNavSVInfoCallback),
  // navSatHandler_(DefaultNavSatCallback),
  // navSigHandler_(DefaultNavSigCallback),
  // navGpsTimeHandler_(DefaultNavGPSTimeCallback),
  // navUtcTimeHandler_(DefaultNavUTCTimeCallback),
  // navDopHandler_(DefaultNavDOPCallback),
  // navDgpsHandler_(DefaultNavDGPSCallback),
  // navHpPosEcefHandler_(DefaultNavHPPosEcefCallback),
  // navHpPosLlhHandler_(DefaultNavHPPosLlhCallback),
  // navClockHandler_(DefaultNavClockCallback),
  // aidAlmHandler_(DefaultAidAlmCallback),
  // aidEphHandler_(DefaultAidEphCallback),
  // aidHuiHandler_(DefaultAidHuiCallback),
  // aidIniHandler_(DefaultAidIniCallback),
  // rawSerialDataHandler_(DefaultRawSerialDataCallback),
  // rxmRawHandler_(DefaultRxmRawCallback),
  // rxmRawxHandler_(DefaultRxmxRawCallback),
  // rxmSubframeHandler_(DefaultRxmSubframeCallback),
  // rxmSubframeXHandler_(DefaultRxmSubframeXCallback),
  // rxmSvsiHandler_(DefaultRxmSvsiCallback),
  // parsedEphemHandler_(nullptr),
  // monVersionHandler_(DefaultMonitorVersionCallback),
  // monGnssHandler_(DefaultMonitorGNSS),
  // monHardwareHandler_(DefaultMonitorHardware),
  // monExtHardwareHandler_(DefaultMonitorExtendedHardware),
  // monSpanHandler_(DefaultMonSpanCallback),
  // monRfHandler_(DefaultMonRfCallback),
  , bytesRemaining_(0)
  , bufferIndex_(0)
  , msgId_(0)
{
}

Ublox::~Ublox()
{
  log_("Ublox::~Ublox()", logutils::LogLevel::Debug);
  shutdown();
}

void Ublox::shutdown()
{
  // Stop monitorconnection thread
  stopMonitorConnectionThread_ = true;
  while (activeMonitorConnectionThread_)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Stop the readSerialPort() thread
  stopReading_ = true;
  while (reading_)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // disconnect serialPort_
  if (isConnected_)
    disconnect();
}

bool Ublox::connect(std::string port, int desiredBaud, bool search)
{
  bool connected = connect_(port, desiredBaud);

  if (!connected && search)
  {
    // search additional baud rates
    int  bauds_to_search[6] = {4800, 9600, 19200, 38400, 57600, 115200};
    bool found              = false;
    for (int ii = 0; ii < 6; ++ii)
    {
      std::stringstream search_msg;
      search_msg << "Searching for ublox at port " << port
                 << " , baudrate: " << bauds_to_search[ii];
      log_(search_msg.str(), logutils::LogLevel::Debug);
      if (connect_(port, bauds_to_search[ii]))
      {
        found = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // if the receiver was found on a different baud rate,
    // change its setting to the desired baud rate and reconnect
    if (found)
    {
      // Get current Port settings (port identifier)
      pollCurrentPortConfiguration();
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      // read data from serial port and look for version
      unsigned char result[MAX_UBX_SIZE];
      size_t        bytesRead;
      bytesRead = serialPort_->read(result, MAX_UBX_SIZE);
      // make sure we read enough bytes for there to be a response
      if (bytesRead < 8)
      {
        std::stringstream output;
        output << "Only read " << bytesRead
               << " bytes in response to pollCurrentPortConfiguration().";
        log_(output.str(), logutils::LogLevel::Error);
        disconnect();
        return false;
      }

      uint16_t       payload_length = 0;  // payload length
      unsigned char* msgPtr;
      // search through result for version message
      for (unsigned int ii = 0; ii < (bytesRead - 8); ++ii)
      {
        // std::cout << hex << (unsigned int)result[ii] << std::endl;
        if (result[ii] == UBX_SYNC_BYTE_1)
        {
          if (result[ii + 1] != UBX_SYNC_BYTE_2)
            continue;
          if (result[ii + 2] != MSG_CLASS_CFG)
            continue;
          if (result[ii + 3] != MSG_ID_CFG_PRT)
            continue;

          msgPtr = &result[ii];
          payload_length =
            (((uint16_t) * (msgPtr + 5)) << 8) + ((uint16_t) * (msgPtr + 4));
          if (payload_length != 20)
          {
            log_("pollCurrentPortConfiguration response payload_length != 20",
                 logutils::LogLevel::Error);
            disconnect();
            return false;
          }

          break;  // CFG-PRT response found
        }
        if (ii == (bytesRead - 9))  // No CFG-PRT found
        {
          std::stringstream output;
          output << "Read " << bytesRead
                 << " bytes, but CFG-PRT message not found.";
          log_(output.str(), logutils::LogLevel::Error);
          disconnect();
          return false;
        }
      }

      ublox::CfgPrt cur_port_settings;
      memcpy(&cur_port_settings, msgPtr, payload_length + HDR_CHKSM_LENGTH);
      connectedPortId_ = (ublox::PortIdentifier)cur_port_settings.port_id;
      baudrate_        = (uint32_t)desiredBaud;
      log_("Current port id: " + std::to_string((int)cur_port_settings.port_id),
           logutils::LogLevel::Info);
      log_(
        "Current baudrate: " + std::to_string((int)cur_port_settings.reserved3),
        logutils::LogLevel::Info);

      // Change baudrate and reconnect
      try
      {
        cur_port_settings.input_mask  = 0x0001;  // UBX
        cur_port_settings.output_mask = 0x0001;  // UBX
        cur_port_settings.reserved3   = (uint32_t)desiredBaud;
        cur_port_settings.reserved4   = 0;
        cur_port_settings.reserved5   = 0;
        msgPtr                        = (unsigned char*)&cur_port_settings;
        calculateCheckSum(msgPtr + 2, 24, msgPtr + 26);
        serialPort_->write(msgPtr, sizeof(cur_port_settings));
      }
      catch (std::exception& e)
      {
        std::stringstream output;
        output << "Error changing baud rate: " << e.what();
        log_(output.str(), logutils::LogLevel::Error);
        disconnect();
        return false;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      serialPort_->setBaudrate(desiredBaud);
      serialPort_->flush();
      connected = connect_(port, desiredBaud);
    }
  }

  if (connected)
  {
    // start reading
    isConnected_ = true;
    log_("Ublox connected.", logutils::LogLevel::Info);
    startReading();
    return true;
  }
  else
  {
    log_("Failed to connect.", logutils::LogLevel::Error);
    isConnected_ = false;
    return false;
  }
}

bool Ublox::connect_(std::string port, int baudrate)
{
  try
  {
    if (serialPort_ != nullptr)
    {
      delete serialPort_;
      serialPort_ = nullptr;
    }
    serialPort_ =
      new serial::Serial(port, baudrate, serial::Timeout::simpleTimeout(50));

    if (!serialPort_->isOpen())
    {
      std::stringstream output;
      output << "Serial port: " << port << " failed to open.";
      log_(output.str(), logutils::LogLevel::Error);
      delete serialPort_;
      serialPort_ = nullptr;
      // isConnected_ = false;
      return false;
    }
    else
    {
      std::stringstream output;
      output << "Serial port: " << port << " opened successfully.";
      log_(output.str(), logutils::LogLevel::Debug);
    }

    serialPort_->flush();

    // look for GPS by sending ping and waiting for response
    if (!ping())
    {
      std::stringstream output;
      output << "Ublox GPS not found on port: " << port << std::endl;
      log_(output.str(), logutils::LogLevel::Error);
      delete serialPort_;
      serialPort_ = nullptr;
      // isConnected_ = false;
      return false;
    }
  }
  catch (std::exception &e)
  {
    std::stringstream output;
    output << "Failed to open port " << port << "  Err: " << e.what();
    log_(output.str(), logutils::LogLevel::Debug);
    delete serialPort_;
    serialPort_ = nullptr;
    // isConnected_ = false;
    return false;
  }

  //  startReading();
  //  isConnected_ = true;
  return true;
}

bool Ublox::ping(int numAttempts, int timeout)
{
  try
  {
    while (numAttempts-- > 0)
    {
      log_("Searching for Ublox receiver...", logutils::LogLevel::Debug);

      // ask for version
      pollMessage(MSG_CLASS_MON, MSG_ID_MON_VER);

      std::this_thread::sleep_for(std::chrono::milliseconds(timeout));

      // read data from serial port and look for version
      unsigned char result[MAX_UBX_SIZE];
      size_t        bytesRead;
      bytesRead = serialPort_->read(result, MAX_UBX_SIZE);

      // make sure we read enough bytes for there to be a response
      if (bytesRead < 8)
      {
        std::stringstream output;
        output << "Only read " << bytesRead << " bytes in response to ping.";
        log_(output.str(), logutils::LogLevel::Debug);
        continue;
      }

      uint16_t length;
      // search through result for version messagels /d tty*
      for (unsigned int ii = 0; ii < (bytesRead - 8); ii++)
      {
        // std::cout << hex << (unsigned int)result[ii] << std::endl;
        if (result[ii] == UBX_SYNC_BYTE_1)
        {
          if (result[ii + 1] != UBX_SYNC_BYTE_2)
            continue;
          if (result[ii + 2] != MSG_CLASS_MON)
            continue;
          if (result[ii + 3] != MSG_ID_MON_VER)
            continue;
          length = (result[ii + 4]) + (result[ii + 5] << 8);
          if (length < 40)
          {
            log_("Incomplete version message received",
                 logutils::LogLevel::Debug);
            //    //return false;
            continue;
          }

          // parse the hardware and software versions from the data
          std::string sw_version;
          std::string hw_version;
          std::string rom_version;
          sw_version.append((char*)(result + ii + 6));
          hw_version.append((char*)(result + ii + 36));
          log_("Ublox receiver found.", logutils::LogLevel::Info);
          log_("Software Version: " + sw_version, logutils::LogLevel::Info);
          log_("Hardware Version: " + hw_version, logutils::LogLevel::Info);

          unsigned int number_of_extended_fields =
            (unsigned int)(length - 40) / 30;
          for (size_t jj = 0; jj < number_of_extended_fields; ++jj)
          {
            std::string extension;
            extension.append((char*)(result + ii + 46 + 30 * jj));
            log_("MON-VER Extension: " + extension, logutils::LogLevel::Info);
          }
          return true;
        }
      }
      std::stringstream output;
      output << "Read " << bytesRead
             << " bytes, but version message not found.";
      log_(output.str(), logutils::LogLevel::Debug);
    }
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error pinging receiver: " << e.what();
    log_(output.str(), logutils::LogLevel::Error);
    return false;
  }

  return false;
}

void Ublox::disconnect()
{
  try
  {
    isConnected_ = false;
    if (serialPort_ != nullptr)
    {
      if (serialPort_->isOpen())
        serialPort_->close();
      delete serialPort_;
      serialPort_ = nullptr;
    }
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error disconnecting from ublox: " << e.what();
    log_(output.str(), logutils::LogLevel::Error);
  }
}

void Ublox::startReading()
{
  try
  {
    // create thread to read from sensor
    stopReading_ = false;
    std::thread readingThread(&Ublox::readSerialPort, this);
    readingThread.detach();
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error starting ublox read thread: " << e.what();
    log_(output.str(), logutils::LogLevel::Error);
  }
}

void Ublox::readSerialPort()
{
  size_t len;
  reading_ = true;
  // continuously read data from serial port
  while (!stopReading_)
  {
    // read data
    try
    {
      len = serialPort_->read(buffer_, MAX_UBX_SIZE);
      // timestamp the read
      readTimestamp_ = std::chrono::system_clock::now();
      if (rawSerialDataHandler_)
        rawSerialDataHandler_((char*)buffer_, len);
    }
    catch (serial::SerialException& e)
    {
      std::stringstream output;
      output << "Error reading serial port: " << e.what();
      log_(output.str(), logutils::LogLevel::Debug);
      continue;
    }
    catch (std::exception& e)
    {
      std::stringstream output;
      output << "Error reading serial port: " << e.what();
      log_(output.str(), logutils::LogLevel::Error);
      reading_ = false;
      // Error w/ serial port, need to reset it
      disconnect();
      break;
    }

    // add data to the buffer to be parsed
    bufferIncomingData(buffer_, len);
  }
  log_("exiting readSerialPort()", logutils::LogLevel::Debug);
  reading_ = false;
}

bool Ublox::pollMessage(uint8_t class_id, uint8_t msg_id)
{
  try
  {
    uint8_t message[8];

    message[0] = UBX_SYNC_BYTE_1;  // sync 1
    message[1] = UBX_SYNC_BYTE_2;  // sync 2
    message[2] = class_id;
    message[3] = msg_id;
    message[4] = 0;  // length 1
    message[5] = 0;  // length 2
    message[6] = 0;  // checksum 1
    message[7] = 0;  // checksum 2

    uint8_t* msg_ptr = (uint8_t*)&message;

    calculateCheckSum(msg_ptr + 2, 4, msg_ptr + 6);

    if ((serialPort_ != nullptr) && (serialPort_->isOpen()))
    {
      size_t bytes_written = serialPort_->write(message, 8);
      return bytes_written == 8;
    }
    else
    {
      log_("Unable to send poll message. Serial port not open.",
           logutils::LogLevel::Error);
      return false;
    }
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error sending ublox poll message: " << e.what();
    log_(output.str(), logutils::LogLevel::Error);
    return 0;
  }
}

bool Ublox::pollMessageIndSV(uint8_t class_id, uint8_t msg_id, uint8_t svid)
{
  try
  {
    uint8_t message[9];

    message[0] = UBX_SYNC_BYTE_1;  // sync 1
    message[1] = UBX_SYNC_BYTE_2;  // sync 2
    message[2] = class_id;
    message[3] = msg_id;
    message[4] = 1;     // length 1
    message[5] = 0;     // length 2
    message[6] = svid;  // Payload
    message[7] = 0;     // checksum 1
    message[8] = 0;     // checksum 2

    uint8_t* msg_ptr = (uint8_t*)&message;
    calculateCheckSum(msg_ptr + 2, 5, msg_ptr + 7);

    if ((serialPort_ != nullptr) && (serialPort_->isOpen()))
    {
      size_t bytes_written = serialPort_->write(msg_ptr, 9);
      return bytes_written == 9;
    }
    else
    {
      log_("Unable to send poll ind. sv. message. Serial port not open.",
           logutils::LogLevel::Error);
      return false;
    }
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error polling individual svs: " << e.what();
    log_(output.str(), logutils::LogLevel::Error);
    return 0;
  }
}

bool Ublox::pollNavigationParamterConfiguration()
{
  log_("Polling for CFG-NAV5..", logutils::LogLevel::Info);
  return pollMessage(MSG_CLASS_CFG, MSG_ID_CFG_NAV5);
}

bool Ublox::cfgItfmEnable(bool enable)
{
  try
  { 
    uint8_t message[17];
    message[0] = UBX_SYNC_BYTE_1;  // sync 1 
    message[1] = UBX_SYNC_BYTE_2;  // sync 2
    message[2] = MSG_CLASS_CFG;
    message[3] = MSG_ID_CFG_VALSET;
    message[4] = 9;  // length 1
    message[5] = 0;  // length 2
    message[6] = 0;  // message version
    message[7] = 1;  // Save layers 1-RAM
    message[8] = 0;  // reserved 0
    message[9] = 0;  // reserved 0
    message[10] = 0x0d; // key id 0x1041000d
    message[11] = 0x00;
    message[12] = 0x41;
    message[13] = 0x10;
    message[14] = (uint8_t)enable; // data
    message[15] = 0;  // checksum 1
    message[16] = 0;  // checksum 2

    uint8_t* msg_ptr = (uint8_t*)&message;

    calculateCheckSum(msg_ptr + 2, 13, msg_ptr + 15);

    // printHex(reinterpret_cast<char*>(msg_ptr),17);

    if ((serialPort_ != nullptr) && (serialPort_->isOpen()))
    {
      size_t bytes_written = serialPort_->write(message, 17);
      return bytes_written == 17;
    }
    else
    {
      log_("Unable to send CFG-ITFM-ENABLE message. Serial port not open.",
           logutils::LogLevel::Error);
      return false;
    }
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error sending ublox CFG-ITFM-ENABLE message: " << e.what();
    log_(output.str(), logutils::LogLevel::Error);
    return false;
  }
  return true;
}

bool Ublox::cfgItfmBbJamThreshold(uint8_t threshold)
{
  try
  { 
    uint8_t message[17];
    message[0] = UBX_SYNC_BYTE_1;  // sync 1 
    message[1] = UBX_SYNC_BYTE_2;  // sync 2
    message[2] = MSG_CLASS_CFG;
    message[3] = MSG_ID_CFG_VALSET;
    message[4] = 9;  // length 1
    message[5] = 0;  // length 2
    message[6] = 0;  // message version
    message[7] = 1;  // Save layers 1-RAM
    message[8] = 0;  // reserved 0
    message[9] = 0;  // reserved 0
    message[10] = 0x01; // key id
    message[11] = 0x00;
    message[12] = 0x41;
    message[13] = 0x20;
    message[14] = (uint8_t)threshold; // data
    message[15] = 0;  // checksum 1
    message[16] = 0;  // checksum 2

    uint8_t* msg_ptr = (uint8_t*)&message;

    calculateCheckSum(msg_ptr + 2, 13, msg_ptr + 15);

    // printHex(reinterpret_cast<char*>(msg_ptr),17);

    if ((serialPort_ != nullptr) && (serialPort_->isOpen()))
    {
      size_t bytes_written = serialPort_->write(message, 17);
      return bytes_written == 17;
    }
    else
    {
      log_("Unable to send CFG-ITFM-BBTHRESHOLD message. Serial port not open.",
           logutils::LogLevel::Error);
      return false;
    }
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error sending ublox CFG-ITFM-BBTHRESHOLD message: " << e.what();
    log_(output.str(), logutils::LogLevel::Error);
    return false;
  }
  return true;
}

bool Ublox::cfgItfmCwJamThreshold(uint8_t threshold)
{
  try
  { 
    uint8_t message[17];
    message[0] = UBX_SYNC_BYTE_1;  // sync 1 
    message[1] = UBX_SYNC_BYTE_2;  // sync 2
    message[2] = MSG_CLASS_CFG;
    message[3] = MSG_ID_CFG_VALSET;
    message[4] = 9;  // length 1
    message[5] = 0;  // length 2
    message[6] = 0;  // message version
    message[7] = 1;  // Save layers 1-RAM
    message[8] = 0;  // reserved 0
    message[9] = 0;  // reserved 0
    message[10] = 0x02; // key id
    message[11] = 0x00;
    message[12] = 0x41;
    message[13] = 0x20;
    message[14] = (uint8_t)threshold; // data
    message[15] = 0;  // checksum 1
    message[16] = 0;  // checksum 2

    uint8_t* msg_ptr = (uint8_t*)&message;

    calculateCheckSum(msg_ptr + 2, 13, msg_ptr + 15);

    // printHex(reinterpret_cast<char*>(msg_ptr),17);

    if ((serialPort_ != nullptr) && (serialPort_->isOpen()))
    {
      size_t bytes_written = serialPort_->write(message, 17);
      return bytes_written == 17;
    }
    else
    {
      log_("Unable to send CFG-ITFM-BBTHRESHOLD message. Serial port not open.",
           logutils::LogLevel::Error);
      return false;
    }
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error sending ublox CFG-ITFM-BBTHRESHOLD message: " << e.what();
    log_(output.str(), logutils::LogLevel::Error);
    return false;
  }
  return true;
}

bool Ublox::pollEphem(int8_t svid)
{
  if (svid < -1)
  {
    log_("Error in PollEphem: Invalid input 'svid'", logutils::LogLevel::Error);
    return 0;
  }
  else if (svid == -1)
  {  // Requests Ephemerides for all SVs
    log_("Polling for all Ephemerides..", logutils::LogLevel::Debug);
    return pollMessage(MSG_CLASS_AID, MSG_ID_AID_EPH);
  }
  else if (svid > 0)
  {  // Requests Ephemeris for a single SV
    std::stringstream output;
    output << "Polling for SV# " << (int)svid << " Ephemeris..";
    log_(output.str(), logutils::LogLevel::Debug);
    return pollMessageIndSV(MSG_CLASS_AID, MSG_ID_AID_EPH, (uint8_t)svid);
  }
  else
  {
    log_("Error in PollEphem: Invalid input 'svid'", logutils::LogLevel::Error);
    return 0;
  }
}

bool Ublox::pollAlmanac(int8_t svid)
{
  if (svid < -1)
  {
    log_("Error in PollAlmanac: Invalid input 'svid'",
         logutils::LogLevel::Error);
    return 0;
  }
  else if (svid == -1)
  {  // Requests Almanac Data for all SVs
    log_("Polling for all Almanac Data..", logutils::LogLevel::Debug);
    return pollMessage(MSG_CLASS_AID, MSG_ID_AID_ALM);
  }
  else if (svid > 0)
  {  // Requests Almanac Data for a single SV
    std::stringstream output;
    output << "Polling for SV# " << (int)svid << " Almanac Data..";
    log_(output.str(), logutils::LogLevel::Debug);
    return pollMessageIndSV(MSG_CLASS_AID, MSG_ID_AID_ALM, (uint8_t)svid);
  }
  else
  {
    log_("Error in PollAlmanac: Invalid input 'svid'",
         logutils::LogLevel::Error);
    return 0;
  }
}

bool Ublox::pollHui()
{
  log_("Polling for AID-HUI..", logutils::LogLevel::Debug);
  return pollMessage(MSG_CLASS_AID, MSG_ID_AID_HUI);
}

bool Ublox::pollAidIni()
{
  log_("Polling for AID-INI..", logutils::LogLevel::Debug);
  return pollMessage(MSG_CLASS_AID, MSG_ID_AID_INI);
}

bool Ublox::pollAllAidData()
{
  log_("Polling for AID-HUI, AID-INI, AID-EPH, & AID-ALM..",
       logutils::LogLevel::Debug);
  return pollMessage(MSG_CLASS_AID, MSG_ID_AID_DATA);
}

bool Ublox::pollRawMeasurementData()
{
  log_("Polling for RXM-RAW..", logutils::LogLevel::Debug);
  return pollMessage(MSG_CLASS_RXM, MSG_ID_RXM_RAW);
}

bool Ublox::pollSvStatus()
{
  log_("Polling for RXM-SVSI..", logutils::LogLevel::Debug);
  return pollMessage(MSG_CLASS_RXM, MSG_ID_RXM_SVSI);
}

bool Ublox::pollSvInfo()
{
  log_("Polling for NAV-SVINFO..", logutils::LogLevel::Debug);
  return pollMessage(MSG_CLASS_NAV, MSG_ID_NAV_SVINFO);
}

bool Ublox::pollNavStatus()
{
  log_("Polling for Receiver NAV-STATUS..", logutils::LogLevel::Debug);
  return pollMessage(MSG_CLASS_NAV, MSG_ID_NAV_STATUS);
}

////////////////////////////////////////////////////////
// (CFG) Configuration Messages
////////////////////////////////////////////////////////
bool Ublox::reset(ResetMask nav_bbr_mask, ResetMode reset_mode)
{
  try
  {
    ublox::CfgRst message;

    message.header.sync1          = UBX_SYNC_BYTE_1;
    message.header.sync2          = UBX_SYNC_BYTE_2;
    message.header.message_class  = MSG_CLASS_CFG;
    message.header.message_id     = MSG_ID_CFG_RST;
    message.header.payload_length = 4;

    message.nav_bbr_mask = (uint16_t)nav_bbr_mask;  // X2-Bitfield?
    // Startup Modes
    // Hotstart 0x000
    // Warmstart 0x0001
    // Coldstart 0xFFFF
    message.reset_mode = (uint8_t)reset_mode;
    // Reset Modes:
    // Hardware Reset 0x00
    // Controlled Software Reset 0x01
    // Controlled Software Reset - Only GPS 0x02
    // Hardware Reset After Shutdown 0x04
    // Controlled GPS Stop 0x08
    // Controlled GPS Start 0x09

    // message.reserved = 0;

    unsigned char* msg_ptr = (unsigned char*)&message;
    calculateCheckSum(msg_ptr + 2, 8, message.checksum);
    if ((serialPort_ != nullptr) && (serialPort_->isOpen()))
    {
      return serialPort_->write(msg_ptr, sizeof(message)) == sizeof(message);
    }
    else
    {
      log_("Unable to send reset command. Serial port not open.",
           logutils::LogLevel::Error);
      return false;
    }
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error resetting ublox: " << e.what();
    log_(output.str(), logutils::LogLevel::Error);
    return false;
  }
}

bool Ublox::resetToColdStart(ResetMode reset_mode)
{
  log_("Receiver reset to cold start state.", logutils::LogLevel::Info);
  return reset(ResetMask::ColdStart, reset_mode);
}

bool Ublox::resetToWarmStart()
{
  log_("Receiver reset to warm start state.", logutils::LogLevel::Info);
  return reset(ResetMask::WarmStart, ResetMode::ControlledSoftwareGps);
}

bool Ublox::resetToHotStart()
{
  log_("Receiver reset to hot start state.", logutils::LogLevel::Info);
  return reset(ResetMask::HotStart, ResetMode::ControlledSoftwareGps);
}

bool Ublox::configureNavigationParameters(uint8_t dynamic_model,
                                          uint8_t fix_mode)
{
  try
  {
    ublox::CfgNav5 message;

    message.header.sync1          = UBX_SYNC_BYTE_1;
    message.header.sync2          = UBX_SYNC_BYTE_2;
    message.header.message_class  = MSG_CLASS_CFG;
    message.header.message_id     = MSG_ID_CFG_NAV5;
    message.header.payload_length = 36;

    message.mask                    = 0x05;  // 0b00000101
    message.dynamic_model           = dynamic_model;
    message.fix_mode                = fix_mode;
    message.fixed_altitude          = 0;
    message.fixed_altitude_variance = 0;
    message.min_elevation           = 0;
    message.dead_reckoning_limit    = 0;
    message.pdop                    = 0;
    message.tdop                    = 0;
    message.pos_accuracy_mask       = 0;
    message.time_accuracy_mask      = 0;
    message.static_hold_threshold   = 0;
    message.dgps_timeout            = 0;
    message.reserved2               = 0;
    message.reserved3               = 0;
    message.reserved4               = 0;

    unsigned char* msg_ptr = (unsigned char*)&message;
    calculateCheckSum(msg_ptr + 2, 36 + 4, message.checksum);

    return serialPort_->write(msg_ptr, sizeof(message)) == sizeof(message);
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error configuring ublox navigation parameters: " << e.what();
    log_(output.str(), logutils::LogLevel::Error);
    return false;
  }
}

bool Ublox::saveConfiguration()
{
  try
  {
    ublox::CfgCfg message;
    message.header.sync1          = UBX_SYNC_BYTE_1;
    message.header.sync2          = UBX_SYNC_BYTE_2;
    message.header.message_class  = MSG_CLASS_CFG;
    message.header.message_id     = MSG_ID_CFG_CNFGR;
    message.header.payload_length = 12;
    message.clear_mask            = 0;
    message.save_mask             = 0x061F;
    message.load_mask             = 0;

    unsigned char* msg_ptr = (unsigned char*)&message;
    calculateCheckSum(msg_ptr + 2, 12 + 4, message.checksum);

    if (serialPort_->write(msg_ptr, sizeof(message)) == sizeof(message))
    {
      log_("Ublox configuration saved.", logutils::LogLevel::Info);
      return true;
    }
    return false;
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error in Ublox::saveConfiguration(): " << e.what();
    log_(output.str(), logutils::LogLevel::Error);
    return false;
  }
}

bool Ublox::configureMessageRate(uint8_t class_id, uint8_t msg_id, uint8_t rate)
{
  try
  {
    ublox::CfgMsgRate message;
    message.header.sync1          = UBX_SYNC_BYTE_1;
    message.header.sync2          = UBX_SYNC_BYTE_2;
    message.header.message_class  = MSG_CLASS_CFG;
    message.header.message_id     = MSG_ID_CFG_MSG;
    message.header.payload_length = 3;

    message.message_class = class_id;
    message.message_id    = msg_id;
    message.rate          = rate;

    // std::cout << "Class: " << std::hex << (int) message.message_class << "
    // ID: "<<  (int)message.message_id  << std::dec << std::endl;

    unsigned char* msg_ptr = (unsigned char*)&message;
    calculateCheckSum(msg_ptr + 2, 7, message.checksum);

    return serialPort_->write(msg_ptr, sizeof(message)) == sizeof(message);
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error configuring ublox message rate: " << e.what();
    log_(output.str(), logutils::LogLevel::Error);
    return false;
  }
}

void Ublox::setPortConfiguration(bool ubxInput,
                                 bool ubxOutput,
                                 bool nmeaInput,
                                 bool nmeaOutput)
{
  try
  {
    ublox::CfgPrt message;
    message.header.sync1          = UBX_SYNC_BYTE_1;
    message.header.sync2          = UBX_SYNC_BYTE_2;
    message.header.message_class  = MSG_CLASS_CFG;
    message.header.message_id     = MSG_ID_CFG_PRT;
    message.header.payload_length = 20;

    message.port_id     = (uint8_t)connectedPortId_;  // Port identifier
    message.reserved    = 0;
    message.tx_ready    = 0;
    message.reserved2   = 0;
    message.reserved3   = baudrate_;  // baudrate
    message.input_mask  = 0;          // Specifies input protocols
    message.output_mask = 0;          // Specifies output protocols
    message.reserved4   = 0;
    message.reserved5   = 0;

    if (ubxInput)
      message.input_mask = message.input_mask | 0x0001;  // set first bit
    else
      message.input_mask = message.input_mask & 0xFFFE;  // clear first bit

    if (nmeaInput)
      message.input_mask = message.input_mask | 0x0002;  // set second bit
    else
      message.input_mask = message.input_mask & 0xFFFD;  // clear second bit

    if (ubxOutput)
      message.output_mask = message.output_mask | 0x0001;  // set first bit
    else
      message.output_mask = message.output_mask & 0xFFFE;  // clear first bit

    if (nmeaOutput)
      message.output_mask = message.output_mask | 0x0002;  // set second bit
    else
      message.output_mask = message.output_mask & 0xFFFD;  // clear second bit

    unsigned char* msg_ptr = (unsigned char*)&message;
    calculateCheckSum(msg_ptr + 2, 24, msg_ptr + 26);

    log_("Set Port Settings Message Sent", logutils::LogLevel::Info);

    // printHex((char*) &message, sizeof(message));

    serialPort_->write(msg_ptr, sizeof(message));
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error configuring ublox port: " << e.what();
    log_(output.str(), logutils::LogLevel::Error);
  }
}

void Ublox::pollPortConfiguration(uint8_t port_identifier)
{  // Port identifier
   // = 3 for USB
   // (default value
   // if left blank)
  //                 = 1 or 2 for UART
  try
  {
    uint8_t message[9];
    message[0] = UBX_SYNC_BYTE_1;
    message[1] = UBX_SYNC_BYTE_2;
    message[2] = MSG_CLASS_CFG;
    message[3] = MSG_ID_CFG_PRT;
    message[4] = 1;
    message[5] = 0;
    message[6] = port_identifier;  // Port identifier for USB Port (3)
    message[7] = 0;                // Checksum A
    message[8] = 0;                // Checksum B

    unsigned char* msg_ptr = (unsigned char*)&message;
    calculateCheckSum(msg_ptr + 2, 5, msg_ptr + 7);

    serialPort_->write(msg_ptr, sizeof(message));
    log_("Polling for Port Protocol Configuration.", logutils::LogLevel::Info);
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error polling ublox port configuration: " << e.what();
    log_(output.str(), logutils::LogLevel::Error);
  }
}

void Ublox::pollCurrentPortConfiguration()
{
  try
  {
    uint8_t message[8];
    message[0] = UBX_SYNC_BYTE_1;
    message[1] = UBX_SYNC_BYTE_2;
    message[2] = MSG_CLASS_CFG;
    message[3] = MSG_ID_CFG_PRT;
    message[4] = 0;  // Payload length
    message[5] = 0;  // Payload length
    message[6] = 0;  // Checksum A
    message[7] = 0;  // Checksum B

    unsigned char* msg_ptr = (unsigned char*)&message;
    calculateCheckSum(msg_ptr + 2, 4, msg_ptr + 6);

    serialPort_->write(msg_ptr, sizeof(message));
    log_("Polling for Current Port Protocol Configuration.",
         logutils::LogLevel::Info);
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error polling ublox port configuration: " << e.what();
    log_(output.str(), logutils::LogLevel::Error);
  }
}

//////////////////////////////////////////////////////////////
// Functions to  Aiding Data to Receiver
//////////////////////////////////////////////////////////////
bool Ublox::sendMessage(uint8_t* msg, size_t length)
{
  try
  {
    std::stringstream output1;
    size_t            bytes_written;

    if ((serialPort_ != nullptr) && (serialPort_->isOpen()))
    {
      bytes_written = serialPort_->write(msg, length);
    }
    else
    {
      log_("Unable to send message. Serial port not open.",
           logutils::LogLevel::Error);
      return false;
    }
    // check that full message was sent to serial port
    if (bytes_written == length)
    {
      return true;
    }
    else
    {
      log_("Full message was not sent over serial port.",
           logutils::LogLevel::Error);
      output1 << "Attempted to send " << length << "bytes. " << bytes_written
              << " bytes sent.";
      log_(output1.str(), logutils::LogLevel::Error);
      return false;
    }
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error sending ublox message: " << e.what();
    log_(output.str(), logutils::LogLevel::Error);
    return false;
  }
}

// Send AID-INI to Receiver
bool Ublox::sendAidIni(AidIni ini)
{
  // std::stringstream output;
  try
  {
    unsigned char* msg_ptr = (unsigned char*)&ini;
    calculateCheckSum(msg_ptr + 2, PAYLOAD_LENGTH_AID_INI + 4, ini.checksum);

    // Check that provided ini message is correct size before sending
    if (sizeof(ini) == FULL_LENGTH_AID_INI)
    {
      bool sent_ini = sendMessage(msg_ptr, FULL_LENGTH_AID_INI);
      return sent_ini;
    }
    else
    {
      std::stringstream output;
      output << "Provided AID-INI message not of correct length.";
      log_(output.str(), logutils::LogLevel::Error);
      return false;
    }
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error sending aid ini data to ublox: " << e.what();
    log_(output.str(), logutils::LogLevel::Error);
    return false;
  }
}

// Send AID-EPH to Receiver
bool Ublox::sendAidEphem(Ephemerides ephems)
{
  try
  {
    // bool sent_ephem[32];

    for (uint8_t prn_index = 1; prn_index <= 32; prn_index++)
    {
      // std::stringstream output;
      if (ephems.ephemsv[prn_index].header.payload_length ==
          PAYLOAD_LENGTH_AID_EPH_WITH_DATA)
      {
        // output << "Sending AID-EPH for PRN # "
        //<< (int) ephems.ephemsv[prn_index].svprn << " ..";
        uint8_t* msg_ptr = (uint8_t*)&ephems.ephemsv[prn_index];
        calculateCheckSum(msg_ptr + 2,
                          PAYLOAD_LENGTH_AID_EPH_WITH_DATA + 4,
                          ephems.ephemsv[prn_index].checksum);
        // sent_ephem[prn_index] =
        sendMessage(msg_ptr, FULL_LENGTH_AID_EPH_WITH_DATA);
      }
      else
      {  // not a full ephemeris message
        // output << "No AID-EPH data for PRN # " << (int) prn_index << " ..";
      }
    }
    return true;
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error sending ephemeris data to ublox: " << e.what();
    log_(output.str(), logutils::LogLevel::Error);
    return false;
  }
}

bool Ublox::sendAidAlm(Almanac almanac)
{
  try
  {
    // bool sent_alm[32];

    for (uint8_t prn_index = 1; prn_index <= 32; prn_index++)
    {
      // std::stringstream output;

      if (almanac.almsv[prn_index].header.payload_length ==
          PAYLOAD_LENGTH_AID_ALM_WITH_DATA)
      {
        // output << "Sending AID-ALM for PRN # "
        //<< (int) almanac.almsv[prn_index].svprn << " ..";
        uint8_t* msg_ptr = (uint8_t*)&almanac.almsv[prn_index];
        calculateCheckSum(msg_ptr + 2,
                          PAYLOAD_LENGTH_AID_ALM_WITH_DATA + 4,
                          almanac.almsv[prn_index].checksum);
        // sent_alm[prn_index] =
        sendMessage(msg_ptr, FULL_LENGTH_AID_ALM_WITH_DATA);
      }
      else
      {
        // output << "No AID-ALM data for PRN # " << (int) prn_index << " ..";
      }
      // log_(output.str());
    }
    return true;
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error sending almanac data to ublox: " << e.what();
    log_(output.str(), logutils::LogLevel::Error);
    return false;
  }
}

bool Ublox::sendAidHui(AidHui hui)
{
  try
  {
    // std::stringstream output;

    unsigned char* msg_ptr = (unsigned char*)&hui;
    calculateCheckSum(msg_ptr + 2, PAYLOAD_LENGTH_AID_HUI + 4, hui.checksum);

    if (sizeof(hui) == FULL_LENGTH_AID_HUI)
    {
      bool sent_hui = sendMessage(msg_ptr, FULL_LENGTH_AID_HUI);
      // output << "Sending AID-HUI to receiver..";
      // log_(output.str());
      return sent_hui;
    }
    else
    {
      // output << "Provided AID-HUI message not of correct length.";
      // log_(output.str());
      return false;
    }
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error sending hui data to ublox: " << e.what();
    log_(output.str(), logutils::LogLevel::Error);
    return false;
  }
}

// NOTE: needs fixing still
bool Ublox::sendRawMeasurement(RawMeas raw_meas)
{
  try
  {
    std::stringstream output;

    output << "Sending RXM-RAW to receiver..";
    log_(output.str(), logutils::LogLevel::Info);
    unsigned char* msg_ptr = (unsigned char*)&raw_meas;
    return sendMessage(msg_ptr, sizeof(raw_meas));
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error sending raw data to ublox: " << e.what();
    log_(output.str(), logutils::LogLevel::Error);
    return false;
  }
}

void Ublox::bufferIncomingData(uint8_t* msg, size_t length)
{
  // printHex(reinterpret_cast<char*>(msg),length);
  try
  {
    for (size_t i = 0; i < length; i++)
    {
      // make sure bufferIndex_ is not larger than buffer
      if (bufferIndex_ >= MAX_UBX_SIZE)
      {
        bufferIndex_ = 0;
        log_("Overflowed receiver buffer. See ublox.cpp BufferIncomingData",
             logutils::LogLevel::Warn);
      }

      if (bufferIndex_ == 0)
      {  // looking for beginning of message
        if (msg[i] == UBX_SYNC_BYTE_1)
        {  // beginning of msg found - add to buffer
          dataBuffer_[bufferIndex_++] = msg[i];
          bytesRemaining_             = 0;
        }  // end if (msg[i]
      }    // end if (bufferIndex_==0)
      else if (bufferIndex_ == 1)
      {  // verify 2nd character of header
        if (msg[i] == UBX_SYNC_BYTE_2)
        {  // 2nd byte ok - add to buffer
          dataBuffer_[bufferIndex_++] = msg[i];
        }
        else
        {
          // start looking for new message again
          bufferIndex_    = 0;
          bytesRemaining_ = 0;
          // readingACK=false;
        }  // end if (msg[i]==UBX_SYNC_BYTE_2)
      }    // end else if (bufferIndex_==1)
      else if (bufferIndex_ == 2)
      {  // look for ack

        if (msg[i] == MSG_CLASS_ACK)  // ACK or NAK message class
        {
          // Get message id from payload
          // char* class_id = reinterpret_cast<char*>(msg[i + 4]);
          // char* msg_id = reinterpret_cast<char*>(msg[i + 5]);

          // Add function which takes class_id and msg_id and returns name of
          // corresponding message

          if (msg[i + 1] == MSG_ID_ACK_ACK)  // ACK Message
          {
            // std::cout << "Receiver Acknowledged Message " << std::endl;
          }
          else if (msg[i + 1] == MSG_ID_ACK_NAK)  // NAK Message
          {
            // std::cout << "Receiver Did Not Acknowledged Message " <<
          }

          bufferIndex_    = 0;
          bytesRemaining_ = 0;
          // readingACK = false;			//? Why is readingACK = false in
          // if
          // &
          // else statement? - CC
        }
        else
        {
          dataBuffer_[bufferIndex_++] = msg[i];
          // readingACK = false;
        }
      }
      else if (bufferIndex_ == 3)
      {
        // msg[i] and msg[i-1] define message ID
        dataBuffer_[bufferIndex_++] = msg[i];
        // length of header is in byte 4
        msgId_ = ((dataBuffer_[bufferIndex_ - 2]) << 8) +
                 dataBuffer_[bufferIndex_ - 1];
      }
      else if (bufferIndex_ == 5)
      {
        // add byte to buffer
        dataBuffer_[bufferIndex_++] = msg[i];
        // length of message (payload + 2 byte check sum )
        bytesRemaining_ = ((dataBuffer_[bufferIndex_ - 1]) << 8) +
                          dataBuffer_[bufferIndex_ - 2] + 2;
      }
      else if (bufferIndex_ == 6)
      {  // set number of bytes
        dataBuffer_[bufferIndex_++] = msg[i];
        bytesRemaining_--;
      }
      else if (bytesRemaining_ == 1)
      {  // add last byte and parse
        dataBuffer_[bufferIndex_++] = msg[i];
        parseLog(dataBuffer_, msgId_);
        // reset counters
        bufferIndex_    = 0;
        bytesRemaining_ = 0;
      }  // end else if (bytesRemaining_==1)
      else
      {  // add data to buffer
        dataBuffer_[bufferIndex_++] = msg[i];
        bytesRemaining_--;
      }
    }  // end for
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error buffering incoming ublox data: " << e.what();
    log_(output.str(), logutils::LogLevel::Error);
  }
}

void Ublox::parseLog(uint8_t* log, size_t logID)
{
  try
  {
    uint16_t payload_length;
    uint8_t  num_of_svs;
    uint8_t  num_of_channels;

    setLastMessageTime(
      readTimestamp_);  // Set timestamp for last valid message received

    switch (logID)
    {
      case AID_REQ:  // Receiver outputs if accurate internally stored pos and
                     // time aren't available
        log_("AID-REQ message received by computer.", logutils::LogLevel::Info);
        break;

      case CFG_PRT:
        if (portSettingsHandler_)
        {
          ublox::CfgPrt cur_port_settings;
          payload_length =
            (((uint16_t) * (log + 5)) << 8) + ((uint16_t) * (log + 4));
          memcpy(&cur_port_settings, log, payload_length + HDR_CHKSM_LENGTH);

          portSettingsHandler_(cur_port_settings, readTimestamp_);
        }
        break;

      case CFG_NAV5:
        if (navigationParametersHandler_)
        {
          ublox::CfgNav5 cur_nav5_settings;
          payload_length =
            (((uint16_t) * (log + 5)) << 8) + ((uint16_t) * (log + 4));
          memcpy(&cur_nav5_settings, log, payload_length + HDR_CHKSM_LENGTH);

          navigationParametersHandler_(cur_nav5_settings, readTimestamp_);
        }
        break;

      case NAV_STATUS:
        if (navStatusHandler_)
        {
          ublox::NavStatus cur_nav_status;
          payload_length =
            (((uint16_t) * (log + 5)) << 8) + ((uint16_t) * (log + 4));
          memcpy(&cur_nav_status, log, payload_length + HDR_CHKSM_LENGTH);

          navStatusHandler_(cur_nav_status, readTimestamp_);
        }
        break;

      case NAV_SOL:
        if (navSolHandler_)
        {
          ublox::NavSol cur_nav_sol;
          payload_length =
            (((uint16_t) * (log + 5)) << 8) + ((uint16_t) * (log + 4));
          memcpy(&cur_nav_sol, log, payload_length + HDR_CHKSM_LENGTH);

          navSolHandler_(cur_nav_sol, readTimestamp_);
        }
        break;

      case NAV_PVT:
        if (navPvtHandler_)
        {
          ublox::NavPvt cur_nav_pvt;
          payload_length =
            (((uint16_t) * (log + 5)) << 8) + ((uint16_t) * (log + 4));
          memcpy(&cur_nav_pvt, log, payload_length + HDR_CHKSM_LENGTH);

          navPvtHandler_(cur_nav_pvt, readTimestamp_);
        }
        break;

      case NAV_VELNED:
        if (navVelNedHandler_)
        {
          ublox::NavVelNed cur_nav_vel_ned;
          payload_length =
            (((uint16_t) * (log + 5)) << 8) + ((uint16_t) * (log + 4));
          memcpy(&cur_nav_vel_ned, log, payload_length + HDR_CHKSM_LENGTH);

          navVelNedHandler_(cur_nav_vel_ned, readTimestamp_);
        }
        break;

      case NAV_POSECEF:
        if (navPosEcefHandler_)
        {
          ublox::NavPosECEF cur_ecef_position;
          payload_length =
            (((uint16_t) * (log + 5)) << 8) + ((uint16_t) * (log + 4));
          memcpy(&cur_ecef_position, log, payload_length + HDR_CHKSM_LENGTH);

          navPosEcefHandler_(cur_ecef_position, readTimestamp_);
        }
        break;

      case NAV_POSLLH:
        if (navPosLlhHandler_)
        {
          ublox::NavPosLLH cur_nav_position;
          payload_length =
            (((uint16_t) * (log + 5)) << 8) + ((uint16_t) * (log + 4));
          memcpy(&cur_nav_position, log, payload_length + HDR_CHKSM_LENGTH);

          navPosLlhHandler_(cur_nav_position, readTimestamp_);
        }
        break;

      case NAV_HPPOSECEF:
        if (navHpPosEcefHandler_)
        {
          ublox::NavHPPosECEF cur_nav_hppecef;
          payload_length =
            (((uint16_t) * (log + 5)) << 8) + ((uint16_t) * (log + 4));
          memcpy(&cur_nav_hppecef, log, payload_length + HDR_CHKSM_LENGTH);

          navHpPosEcefHandler_(cur_nav_hppecef, readTimestamp_);
        }
        break;

      case NAV_HPPOSLLH:
        if (navHpPosLlhHandler_)
        {
          ublox::NavHPPosLLH cur_nav_hppllh;
          payload_length =
            (((uint16_t) * (log + 5)) << 8) + ((uint16_t) * (log + 4));
          memcpy(&cur_nav_hppllh, log, payload_length + HDR_CHKSM_LENGTH);

          navHpPosLlhHandler_(cur_nav_hppllh, readTimestamp_);
        }
        break;

      case NAV_SVINFO:
        if (navSvInfoHandler_)
        {
          ublox::NavSVInfo cur_nav_svinfo;
          payload_length =
            (((uint16_t) * (log + 5)) << 8) + ((uint16_t) * (log + 4));
          num_of_channels = (uint8_t) * (log + 10);

          // Copy portion of NAV-INFO before repeated block (8 + header length)
          memcpy(&cur_nav_svinfo, log, 6 + 8);
          // Copy repeated block
          for (int index = 0; index < num_of_channels; index++)
          {
            memcpy(
              &cur_nav_svinfo.svinfo_reap[index], log + 14 + (index * 12), 12);
          }
          // Copy Checksum
          memcpy(
            &cur_nav_svinfo.checksum, log + 14 + (num_of_channels * 12), 2);

          navSvInfoHandler_(cur_nav_svinfo, readTimestamp_);
        }
        break;

      case NAV_SAT:
        if (navSatHandler_)
        {
          ublox::NavSat cur_nav_sat;
          payload_length =
            (((uint16_t) * (log + 5)) << 8) + ((uint16_t) * (log + 4));

          // Copy portion of NAV-INFO before repeated block (8 + header length)
          memcpy(&cur_nav_sat, log, 14);

          // Copy repeated block
          memcpy(&cur_nav_sat.satInfo,
                 log + 14,
                 cur_nav_sat.numSvs * sizeof(NavSatBlock));

          // Copy Checksum
          memcpy(&cur_nav_sat.checksum,
                 log + 14 + (cur_nav_sat.numSvs * sizeof(NavSatBlock)),
                 2);

          navSatHandler_(cur_nav_sat, readTimestamp_);
        }
        break;

      case NAV_SIG:
        if (navSigHandler_)
        {
          ublox::NavSig curNavSig;
          payload_length =
            (((uint16_t) * (log + 5)) << 8) + ((uint16_t) * (log + 4));

          // Copy portion of NAV-SIG before repeated block (8 + header length)
          memcpy(&curNavSig, log, 14);

          // Copy repeated block
          memcpy(&curNavSig.sigInfo,
                 log + 14,
                 curNavSig.numSigs * sizeof(NavSigBlock));

          // Copy Checksum
          memcpy(&curNavSig.checksum,
                 log + 14 + (curNavSig.numSigs * sizeof(NavSigBlock)),
                 2);

          navSigHandler_(curNavSig, readTimestamp_);
        }
        break;

      case NAV_TIMEGPS:
        if (navGpsTimeHandler_)
        {
          ublox::NavGPSTime cur_nav_gps_time;
          payload_length =
            (((uint16_t) * (log + 5)) << 8) + ((uint16_t) * (log + 4));
          memcpy(&cur_nav_gps_time, log, payload_length + HDR_CHKSM_LENGTH);

          navGpsTimeHandler_(cur_nav_gps_time, readTimestamp_);
        }
        break;

      case NAV_TIMEUTC:
        if (navUtcTimeHandler_)
        {
          ublox::NavUTCTime cur_nav_utc_time;
          payload_length =
            (((uint16_t) * (log + 5)) << 8) + ((uint16_t) * (log + 4));
          memcpy(&cur_nav_utc_time, log, payload_length + HDR_CHKSM_LENGTH);

          navUtcTimeHandler_(cur_nav_utc_time, readTimestamp_);
        }
        break;

      case NAV_DOP:
        if (navDopHandler_)
        {
          ublox::NavDOP cur_nav_dop;
          payload_length =
            (((uint16_t) * (log + 5)) << 8) + ((uint16_t) * (log + 4));
          memcpy(&cur_nav_dop, log, payload_length + HDR_CHKSM_LENGTH);

          navDopHandler_(cur_nav_dop, readTimestamp_);
        }
        break;

      case NAV_DGPS:
        if (navDgpsHandler_)
        {
          ublox::NavDGPS cur_nav_dgps;
          payload_length =
            (((uint16_t) * (log + 5)) << 8) + ((uint16_t) * (log + 4));
          memcpy(&cur_nav_dgps, log, payload_length + HDR_CHKSM_LENGTH);

          navDgpsHandler_(cur_nav_dgps, readTimestamp_);
        }
        break;

      case NAV_CLK:
        if (navClockHandler_)
        {
          ublox::NavClock cur_nav_clock;
          payload_length =
            (((uint16_t) * (log + 5)) << 8) + ((uint16_t) * (log + 4));
          memcpy(&cur_nav_clock, log, payload_length + HDR_CHKSM_LENGTH);

          navClockHandler_(cur_nav_clock, readTimestamp_);
        }
        break;

      case AID_EPH:
        if (aidEphHandler_)
        {
          ublox::EphemSV cur_ephem_sv;

          payload_length =
            (((uint16_t) * (log + 5)) << 8) + ((uint16_t) * (log + 4));

          // printHex((char*) &cur_ephem_sv, sizeof(cur_ephem_sv));
          memcpy(&cur_ephem_sv, log, payload_length + HDR_CHKSM_LENGTH);

          // If Ephemeris for SV is not present (payload_length is 8 bytes)
          if (payload_length == PAYLOAD_LENGTH_AID_EPH_NO_DATA)
          {
            // std::stringstream output;
            // output << "SV# " << (int) *(log+6) << "- no ephemeris data";
            // log_(output.str());
          }
          // If Ephemeris for SV is present (payload_length is 104 bytes)
          else if (payload_length == PAYLOAD_LENGTH_AID_EPH_WITH_DATA)
          {
            // std::stringstream output;
            // output << "SV# " << (int) *(log+6) << "- has ephemeris data";
            // log_(output.str());
          }
          else
          {
            std::stringstream out;
            out << "Error! AID-EPH log payload is not a valid length ("
                << payload_length
                << " bytes)! (See "
                   "ParseLog case AID_EPH)";
            log_(out.str(), logutils::LogLevel::Error);
          }

          // make sure function pointer is set and call callback

          aidEphHandler_(cur_ephem_sv, readTimestamp_);

          //      if (parsedEphemHandler_) {
          //        ublox::ParsedEphemData parsed_ephem =
          //        Parse_aid_eph(cur_ephem_sv);
          //        parsedEphemHandler_(parsed_ephem, readTimestamp_);
          //      }
        }
        break;

      case AID_ALM:
        if (aidAlmHandler_)
        {
          ublox::AlmSV cur_alm_sv;

          payload_length =
            (((uint16_t) * (log + 5)) << 8) + ((uint16_t) * (log + 4));

          memcpy(&cur_alm_sv, log, payload_length + HDR_CHKSM_LENGTH);

          // If Almanac data for SV is not present (payload_length is 8 bytes)
          if (payload_length == 8)
          {
            // std::stringstream output;
            // output << "SV# " << (int) *(log+6) << "- no almanac data";
            // log_(output.str());
          }

          // If Almanac data for SV is present (payload_length is 40 bytes)
          else if (payload_length == 40)
          {
            // std::stringstream output;
            // output << "SV# " << (int) *(log+6) << "- has almanac data";
            // log_(output.str());
          }
          else
          {
            log_(
              "Error! AID-ALM log payload is not 8 or 40 bytes long! (See "
              "ParseLog case AID_ALM)",
              logutils::LogLevel::Error);
          }

          // make sure function pointer is set and call callback

          aidAlmHandler_(cur_alm_sv, readTimestamp_);
        }
        break;

      case AID_HUI:
        if (aidHuiHandler_)
        {
          ublox::AidHui cur_aid_hui;

          payload_length =
            (((uint16_t) * (log + 5)) << 8) + ((uint16_t) * (log + 4));

          memcpy(&cur_aid_hui, log, payload_length + HDR_CHKSM_LENGTH);

          // make sure function pointer is set and call callback

          aidHuiHandler_(cur_aid_hui, readTimestamp_);
        }
        break;

      case AID_INI:
        if (aidIniHandler_)
        {
          ublox::AidIni cur_aid_ini;

          payload_length =
            (((uint16_t) * (log + 5)) << 8) + ((uint16_t) * (log + 4));

          memcpy(&cur_aid_ini, log, payload_length + HDR_CHKSM_LENGTH);

          aidIniHandler_(cur_aid_ini, readTimestamp_);
        }
        break;

      case RXM_RAW:
        if (rxmRawHandler_)
        {
          // NOTE: Needs to be checked/fixed
          ublox::RawMeas cur_raw_meas;

          payload_length =
            (((uint16_t) * (log + 5)) << 8) +
            ((uint16_t) * (log + 4));  // payload_length = 8+24*numSV
          num_of_svs = (uint8_t) * (log + 12);

          // Copy portion of RXM-SVSI before repeated block (8 + header length)
          memcpy(&cur_raw_meas, log, 14);
          // Copy repeated block
          for (uint8_t index = 0; index < num_of_svs; index++)
          {
            memcpy(
              &cur_raw_meas.rawmeasreap[index], log + 14 + (index * 24), 24);
          }
          // Copy Checksum
          memcpy(&cur_raw_meas.checksum, log + 14 + (24 * num_of_svs), 2);

          rxmRawHandler_(cur_raw_meas, readTimestamp_);
        }
        break;

      case RXM_RAWX:
        if (rxmRawxHandler_)
        {
          // NOTE: Needs to be checked/fixed
          ublox::RawMeasX cur_raw_measx;

          payload_length =
            (((uint16_t) * (log + 5)) << 8) +
            ((uint16_t) * (log + 4));  // payload_length = 16+32*numSV
          num_of_svs = (uint8_t) * (log + 17);

          // Copy portion of RXM-RAWX before repeated block (8 + header length)
          memcpy(&cur_raw_measx, log, 22);
          // std::cout << "Num svs copied: " <<
          // Copy repeated block
          for (uint8_t index = 0; index < num_of_svs; index++)
          {
            memcpy(&cur_raw_measx.rawxmeasreap[index],
                   log + 22 + (index * 32),
                   32);
          }
          // Copy Checksum
          memcpy(&cur_raw_measx.checksum, log + 6 + payload_length, 2);

          rxmRawxHandler_(cur_raw_measx, readTimestamp_);
        }
        break;

      case RXM_SFRB:
        if (rxmSubframeHandler_)
        {
          ublox::SubframeData cur_subframe;

          payload_length =
            (((uint16_t) * (log + 5)) << 8) + ((uint16_t) * (log + 4));
          memcpy(&cur_subframe, log, payload_length + HDR_CHKSM_LENGTH);

          rxmSubframeHandler_(cur_subframe, readTimestamp_);
        }
        break;

      case RXM_SFRBX:
        if (rxmSubframeXHandler_)
        {
          ublox::SubframeDataX cur_subframeX;
          payload_length =
            (((uint16_t) * (log + 5)) << 8) + ((uint16_t) * (log + 4));
          memcpy(&cur_subframeX, log, payload_length + HDR_CHKSM_LENGTH);

          rxmSubframeXHandler_(cur_subframeX, readTimestamp_);
        }
        break;

      case RXM_SVSI:
        if (rxmSvsiHandler_)
        {
          // NOTE: needs to be checked!!
          ublox::SVStatus cur_sv_status;

          payload_length =
            (((uint16_t) * (log + 5)) << 8) + ((uint16_t) * (log + 4));
          num_of_svs = (uint8_t) * (log + 13);

          // Copy portion of RXM-SVSI before repeated block (8 + header length)
          memcpy(&cur_sv_status, log, 6 + 8);
          // Copy repeated block
          for (uint8_t index = 0; index < num_of_svs; index++)
          {
            memcpy(
              &cur_sv_status.svstatusreap[index], log + 14 + (index * 6), 6);
          }
          // Copy Checksum
          memcpy(&cur_sv_status.checksum, log + 14 + (6 * num_of_svs), 2);

          rxmSvsiHandler_(cur_sv_status, readTimestamp_);
        }
        break;

      case MON_GNSS:
        if (monGnssHandler_)
        {
          ublox::MonGnss cur_mon_gnss;
          payload_length =
            (((uint16_t) * (log + 5)) << 8) + ((uint16_t) * (log + 4));
          memcpy(&cur_mon_gnss, log, payload_length + HDR_CHKSM_LENGTH);

          monGnssHandler_(cur_mon_gnss, readTimestamp_);
        }
        break;

      case MON_HW2:
        if (monExtHardwareHandler_)
        {
          ublox::MonHw2 cur_mon_hw2;
          payload_length =
            (((uint16_t) * (log + 5)) << 8) + ((uint16_t) * (log + 4));
          memcpy(&cur_mon_hw2, log, payload_length + HDR_CHKSM_LENGTH);

          monExtHardwareHandler_(cur_mon_hw2, readTimestamp_);
        }
        break;

      case MON_HW:
        if (monHardwareHandler_)
        {
          ublox::MonHw cur_mon_hw;
          payload_length =
            (((uint16_t) * (log + 5)) << 8) + ((uint16_t) * (log + 4));
          memcpy(&cur_mon_hw, log, payload_length + HDR_CHKSM_LENGTH);

          monHardwareHandler_(cur_mon_hw, readTimestamp_);
        }
        break;

      case MON_SPAN:
        if (monSpanHandler_)
        {
          // create a new MonSpan structure for data copying
          ublox::MonSpan monSpan;

          // copy Ublox header and unrepeated payload data
          memcpy(&monSpan, log, 10);  // size = HDR_LENGTH + 4

          // Copy the variable size data payload
          ublox::MonSpanData spanEntry;
          for (size_t i = 0; i < monSpan.numRfBlocks; i++)
          {
            memcpy(&spanEntry,
                   log + 10 + (i * sizeof(MonSpanData)),
                   sizeof(MonSpanData));
            monSpan.data.push_back(spanEntry);
          }

          // Copy checksum
          memcpy(&monSpan.checksum,
                 log + 10 + monSpan.numRfBlocks * sizeof(MonSpanData),
                 2);

          monSpanHandler_(monSpan, readTimestamp_);
        }
        break;

      case MON_RF:
        if (monRfHandler_)
        {
          // std::cout << "Received MON-RF" << std::endl;
          ublox::MonRf curMonRf;
          // copy ublox header and non-repeated message
          memcpy(&curMonRf, log, HDR_LENGTH + 4);
          // Copy repeated block
          for (uint8_t ii = 0; ii < curMonRf.numBlocks; ii++)
          {
            memcpy(&curMonRf.rfBlocks[ii],
                   log + (HDR_LENGTH + 4) + (ii * sizeof(MonRfBlock)),
                   sizeof(MonRfBlock));
          }
          // Copy Checksum
          memcpy(
            &curMonRf.checksum,
            log + (HDR_LENGTH + 4) + (curMonRf.numBlocks * sizeof(MonRfBlock)),
            2);

          monRfHandler_(curMonRf, readTimestamp_);
        }
        break;
      default:
        std::stringstream out;
        out << "Received unknown ublox message.  ID: "
            << "0x" << std::uppercase << std::setfill('0') << std::setw(4)
            << std::hex << logID;

        log_(out.str(), logutils::LogLevel::Debug);
    }  // end switch (logID)
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error parsing ublox log: " << e.what();
    log_(output.str(), logutils::LogLevel::Error);
  }
}  // end ParseLog()

bool Ublox::configureReceiver()
{
  try
  {
    if (!isConnected())
    {
      log_("Cannot configure receiver.  Not connected.",
           logutils::LogLevel::Error);
      return false;
    }

    //////////////////////////////////////////////////////////////////
    // set ublox port configuration - determines which message types are
    // allowed in and out of the currently connected serial port
    setPortConfiguration(
      protocolUbxIn_, protocolUbxOut_, protocolNmeaIn_, protocolNmeaOut_);

    ////////////////////////////////////////////////////////////////////
    // Request messages at a specified rate
    // make sure class, id, and rate vectors are all the same length
    if ((msgClasses_.size() != msgIds_.size()) ||
        (msgIds_.size() != msgRates_.size()))
    {
      log_("Invalid set of messages requested.", logutils::LogLevel::Error);
      return false;
    }
    else
    {
      // request selected messages
      for (unsigned int ii = 0; ii < msgClasses_.size(); ii++)
      {
        std::stringstream request;
        request << "Requesting " << msgClasses_[ii] << ":" << msgIds_[ii]
                << " at " << (int)msgRates_[ii] << " Hz.";
        log_(request.str(), logutils::LogLevel::Info);
        configureMessageRate(convertNavClass(msgClasses_[ii]),
                             convertMsgId(msgIds_[ii]),
                             msgRates_[ii]);
      }
    }

    ////////////////////////////////////////////////////////////////////
    // Configure navigation parameters

    size_t dynamicModel = convertModel(dynamicModel_);
    size_t fixMode      = convertFix(fixMode_);

    log_("Setting u-blox navigation parameters:", logutils::LogLevel::Info);
    log_("Dynamic Model: " + dynamicModel_ + "   Fix Mode: " + fixMode_,
         logutils::LogLevel::Info);
    configureNavigationParameters(dynamicModel, fixMode);

    // Enable ITFM
    if (cfgItfmEnable(cfgItfmEnable_))
    {
      log_("Ublox Interference Monitor Enabled = " + std::to_string((int)cfgItfmEnable_), logutils::LogLevel::Info);
    }

    // Set ITFM BB Jam threshold
    if(cfgItfmBbJamThreshold(cfgItfmBbJamThreshold_))
    {
      log_("Ublox Interference Monitor: BB Jam Threshold = " + std::to_string(cfgItfmBbJamThreshold_), logutils::LogLevel::Info);
    }

    // Set ITFM CW Jam threshold
    if(cfgItfmCwJamThreshold(cfgItfmCwJamThreshold_))
    {
      log_("Ublox Interference Monitor: CW Jam Threshold = " + std::to_string(cfgItfmCwJamThreshold_), logutils::LogLevel::Info);
    }

  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error in Ublox::configureReceiver(): " << e.what();
    log_(output.str(), logutils::LogLevel::Error);
    return false;
  }
  log_("Ublox configured.", logutils::LogLevel::Info);
  return true;
}

bool Ublox::pollForData(bool poll_pvt,
                        bool poll_ephemeris,
                        bool poll_almanac,
                        bool poll_utcIono,
                        bool poll_rawData)
{
  bool success = true;

  if (poll_pvt)
  {
    if (!pollAidIni())
    {
      success = false;
      log_("UbloxNode::PollForData - Error polling for PVT",
           logutils::LogLevel::Error);
    }
  }
  if (poll_ephemeris)
  {
    if (!pollEphem(-1))
    {
      success = false;
      log_("UbloxNode::PollForData - Error polling for Ephemeris",
           logutils::LogLevel::Error);
    }
  }
  if (poll_almanac)
  {
    if (!pollAlmanac(-1))
    {
      success = false;
      log_("UbloxNode::PollForData - Error polling for Almanac",
           logutils::LogLevel::Error);
    }
  }
  if (poll_utcIono)
  {
    if (!pollHui())
    {
      success = false;
      log_("UbloxNode::PollForData - Error polling for UTC-Ionosphere",
           logutils::LogLevel::Error);
    }
  }
  if (poll_rawData)
  {
    if (!pollRawMeasurementData())
    {
      success = false;
      log_("UbloxNode::PollForData - Error polling for Raw Data",
           logutils::LogLevel::Error);
    }
  }
  return (success);
}

void Ublox::pollForAgpsData()
{
  if (isConnected())
  {
    pollForData(pollForPvt_,
                pollForEphem_,
                pollForAlmanac_,
                pollForUtcIono_,
                pollForRaw_);
  }
}

void Ublox::run()
{
  startConnectionMonitor();
}

void Ublox::startConnectionMonitor()
{
  std::thread monitorThread(&Ublox::monitorConnection, this);
  monitorThread.detach();
}

void Ublox::monitorConnection()
{
  std::chrono::seconds timeout(monitorConnectionTimeout_);
  std::chrono::seconds monitorPeriod(monitorConnectionPeriod_);
  std::chrono::seconds agpsPollPeriod((unsigned int)receiverAgpsRequestPeriod_);
  std::chrono::seconds loopPeriod(1);

  // Initialize times
  std::chrono::system_clock::time_point lastAgpsPoll =
    std::chrono::system_clock::now();
  std::chrono::system_clock::time_point lastConnectionCheck =
    std::chrono::system_clock::now();
  setLastMessageTime(std::chrono::system_clock::now());
  std::chrono::system_clock::time_point curTime =
    std::chrono::system_clock::now();

  activeMonitorConnectionThread_ = true;
  while (!stopMonitorConnectionThread_)
  {
    std::this_thread::sleep_for(loopPeriod);
    curTime = std::chrono::system_clock::now();

    try
    {
      if (!isConnected())
      {
        log_("Attempting connection to receiver on port: " + port_ +
               " , with desired baudrate " + std::to_string(baudrate_) + " .",
             logutils::LogLevel::Info);

        // Recevier is not connected. try to connect
        if (connect(port_, baudrate_))
        {
          // if the connection was successful, configure the receiver
          if (configureReceiver())
            saveConfiguration();  // TODO: This needs to be smarter, Don't save
                                  // configuration every time
        }
        // reinit timestamps
        setLastMessageTime(std::chrono::system_clock::now());
        lastAgpsPoll        = std::chrono::system_clock::now();
        lastConnectionCheck = std::chrono::system_clock::now();
        continue;
      }

      // Check that I'm still receiving messages
      if ((monitorConnectionTimeout_ > 0) &&
          (std::chrono::duration_cast<std::chrono::seconds>(
             curTime - lastConnectionCheck) > monitorPeriod) &&
          (std::chrono::duration_cast<std::chrono::seconds>(
             curTime - getLastMessageTime()) > timeout))
      {
        lastConnectionCheck = curTime;
        log_("No GPS messages for " + std::to_string(timeout.count()) +
               ", seconds. Connection to receiver lost.",
             logutils::LogLevel::Debug);
        disconnect();
        continue;
      }

      // Poll receiver for AGPS messages
      if ((receiverAgpsRequestPeriod_ > 0) &&
          (std::chrono::duration_cast<std::chrono::seconds>(
             curTime - lastAgpsPoll) > agpsPollPeriod))
      {
        lastAgpsPoll = curTime;
        pollForAgpsData();
        continue;
      }
    }
    catch (std::exception& e)
    {
      std::string er(e.what());
      log_("Error in Ublox::monitorConnection(): " + er,
           logutils::LogLevel::Error);
      disconnect();
    }
  }  // end while (!stopMonitorConnectionThread_)
  activeMonitorConnectionThread_ = false;
  log_("Ublox::monitorConnection() finished", logutils::LogLevel::Debug);
}

void Ublox::calculateCheckSum(uint8_t* in, size_t length, uint8_t* out)
{
  try
  {
    uint8_t a = 0;
    uint8_t b = 0;

    for (uint8_t i = 0; i < length; i++)
    {
      a = a + in[i];
      b = b + a;
    }

    out[0] = (a & 0xFF);
    out[1] = (b & 0xFF);
  }
  catch (std::exception& e)
  {
    std::stringstream output;
    output << "Error calculating ublox checksum: " << e.what();
    log_(output.str(), logutils::LogLevel::Error);
  }
}

}  // namespace ublox
