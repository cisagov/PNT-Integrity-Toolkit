//==---------------- ublox/ublox.h - Ublox class definition ---------------===//
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
//===----------------------------------------------------------------------===//
///
/// \file
/// \brief    This file contains the declaration of the Ublox class.
/// \details  The Ublox class provides a cross platform interface for uBlox GPS
///           receivers using the UBX protocol.
/// \author   David Hodo <david.hodo@is4s.com>
/// \author   Chris Collins <chris.collins@is4s.com>
/// \date     January 26, 2015
///
//===----------------------------------------------------------------------===//

#ifndef UBLOX_H
#define UBLOX_H

#include <atomic>
#include <chrono>
#include <fstream>
#include <functional>
#include <logutils/logutils.hpp>
#include <mutex>
#include <thread>
#include <vector>

#include <serial/serial.h>

#include "ublox/ublox_structures.h"

namespace ublox
{
using SysClkTimePoint = std::chrono::system_clock::time_point;

using GetTimeHandler               = std::function<double()>;
using HandleAcknowledgementHandler = std::function<void()>;

// GPS Data Handlers
using PortSettingsHandler =
  std::function<void(const CfgPrt&, const SysClkTimePoint&)>;
using ConfigureNavigationParametersHandler =
  std::function<void(const CfgNav5&, const SysClkTimePoint&)>;
using NavPosECEFHandler =
  std::function<void(const NavPosECEF&, const SysClkTimePoint&)>;
using NavPosLLHHandler =
  std::function<void(const NavPosLLH&, const SysClkTimePoint&)>;

using NavHPPosPECEFHandler =
  std::function<void(const NavHPPosECEF&, const SysClkTimePoint&)>;
using NavHPPosLLHHandler =
  std::function<void(const NavHPPosLLH&, const SysClkTimePoint&)>;

using NavSolHandler = std::function<void(const NavSol&, const SysClkTimePoint&)>;
using NavPvtHandler = std::function<void(const NavPvt&, const SysClkTimePoint&)>;
using NavStatusHandler =
  std::function<void(const NavStatus&, const SysClkTimePoint&)>;
using NavVelNedHandler =
  std::function<void(const NavVelNed&, const SysClkTimePoint&)>;
using NavSVInfoHandler =
  std::function<void(const NavSVInfo&, const SysClkTimePoint&)>;
using NavSatHandler = std::function<void(const NavSat&, const SysClkTimePoint&)>;
using NavSigHandler = std::function<void(const NavSig&, const SysClkTimePoint&)>;
using NavGPSTimeHandler =
  std::function<void(const NavGPSTime&, const SysClkTimePoint&)>;
using NavUTCTimeHandler =
  std::function<void(const NavUTCTime&, const SysClkTimePoint&)>;
using NavDOPHandler = std::function<void(const NavDOP&, const SysClkTimePoint&)>;
using NavDGPSHandler =
  std::function<void(const NavDGPS&, const SysClkTimePoint&)>;
using NavClockHandler =
  std::function<void(const NavClock&, const SysClkTimePoint&)>;
using AidEphHandler =
  std::function<void(const EphemSV&, const SysClkTimePoint&)>;
using AidAlmHandler = std::function<void(const AlmSV&, const SysClkTimePoint&)>;
using AidHuiHandler = std::function<void(const AidHui&, const SysClkTimePoint&)>;
using AidIniHandler = std::function<void(const AidIni&, const SysClkTimePoint&)>;
using RxmRawHandler =
  std::function<void(const RawMeas&, const SysClkTimePoint&)>;
using RxmRawxHandler =
  std::function<void(const RawMeasX&, const SysClkTimePoint&)>;
using RxmSubframeHandler =
  std::function<void(const SubframeData&, const SysClkTimePoint&)>;
using RxmSvsiHandler =
  std::function<void(const SVStatus&, const SysClkTimePoint&)>;
using ParsedEphemHandler =
  std::function<void(const ParsedEphemData&, const SysClkTimePoint&)>;
using RxmSubframeXHandler =
  std::function<void(const SubframeDataX&, const SysClkTimePoint&)>;
using rawSerialDataHandler = std::function<void(const char*, const int)>;

using MonitorHardwareHandler =
  std::function<void(const MonHw&, const SysClkTimePoint&)>;

using MonitorExtHardwareHandler =
  std::function<void(const MonHw2&, const SysClkTimePoint&)>;

using MonitorVersionHandler =
  std::function<void(const MonVer&, const SysClkTimePoint&)>;

using MonitorGnssHandler =
  std::function<void(const MonGnss&, const SysClkTimePoint&)>;

using MonSpanHandler =
  std::function<void(const MonSpan&, const SysClkTimePoint&)>;

using MonRfHandler = std::function<void(const MonRf&, const SysClkTimePoint&)>;

/// \brief Class to provide an interface to a Ublox GPS Receiver
class Ublox
{
public:
  /// \brief Constructs the Ublox object and initializes member variables
  Ublox();

  /// \brief Called on destruction of the Ublox object
  ~Ublox();

  /// \brief Disconnects from receiver, close serial port, shutdown node
  void shutdown();

  /// \brief Read params, configure, and detach monitorConnection and
  /// periodicPollForAgpsData threads
  void run();

  /// \brief Connects to the GPS receiver.
  ///
  /// Opens the given serial port at the specified baud rate and searches for
  /// the receiver.
  ///
  /// \param port Descriptor of the serial port to open (Ex: "/dev/ttyS0" or
  /// "COM1")
  /// \param baudrate Baud rate to use for connection (Default: 115200)
  ///
  /// \returns True if the serial port was opened and the receiver found, false
  /// otherwise
  bool connect(std::string port, int desiredBaud = 115200, bool search = true);
  bool connect_(std::string port, int baudrate = 115200);

  /// \brief Disconnects from the receiver and closes the serial port
  void disconnect();

  /// \brief Indicates if the serial port is open and a receiver is present
  ///
  /// \returns True if the receiver is connected, false otherwise.
  bool isConnected() { return isConnected_; }

  /// \brief Pings the GPS to determine if it is properly connected
  ///
  /// This method sends a ping to the GPS and waits for a response.
  ///
  /// \param numAttempts The number of times to ping the device (default = 5)
  /// \param timeout The time in milliseconds to wait (default = 1000)
  /// \returns True if the GPS was found, false otherwise.
  bool ping(int numAttempts = 5, int timeout = 200);

  /// \brief Resets the receiver
  ///
  /// \param resetMask Data to reset (Ephemeris, time, etc.)
  /// \param resetMode Method of reset to use. (Hardware, software, etc.)
  /// \returns True if the receiver is successfully reset, false otherwise
  bool reset(ResetMask resetMask, ResetMode resetMode);

  /// \brief Resets the receiver to a cold start state
  ///
  /// \param resetMode Method of reset to use. (Hardware, software, etc.)
  /// \returns True if the receiver is successfully reset, false otherwise
  bool resetToColdStart(ResetMode resetMode);

  /// \brief Resets the receiver to a warm start state
  /// \returns True if the receiver is successfully reset, false otherwise
  bool resetToWarmStart();

  /// \brief Resets the receiver to a hot start state
  /// \returns True if the receiver is successfully reset, false otherwise
  bool resetToHotStart();

  /// \brief Configures the input and output data types on the current port
  ///
  /// \param ubxInput Enable UBX format inputs on the current port
  /// \param ubxOutput Enable UBX format outputs on the current port
  /// \param nmeaInput Enable NMEA format inputs on the current port
  /// \param nmeaOutput Enable NMEA format outputs on the current port
  void setPortConfiguration(bool ubxInput,
                            bool ubxOutput,
                            bool nmeaInput,
                            bool nmeaOutput);

  /// \brief Configures the output rate of a given message
  ///
  /// \param classId Class ID of the message to configure (E.g. NAV = 0x01)
  /// \param msgId Message ID of the message to configure (E.g. NAV-SOL = 0x06)
  /// \param rate Rate relative to the event a message is registered on.
  ///              For example, if the rate of a navigation message is set to 2,
  ///              the message is sent every second navigation solution.
  /// \returns True if the configuraiton succeeded, false otherwise
  bool configureMessageRate(uint8_t classId, uint8_t msgId, uint8_t rate);

  /// \brief Configures the receivers navigation parameters
  ///
  /// \param dynamicModel Model type to use (E.g. pedestrian or automotive)
  /// \param fixMode Fix mode to use (E.g. ?????????)
  /// \returns True if the configuraiton succeeded, false otherwise
  bool configureNavigationParameters(uint8_t dynamicModel = 3,
                                     uint8_t fixMode      = 3);


  bool saveConfiguration();
  //----------------------------------------------------------------------------
  // Receiver Configuration Settings Polling Messages
  //----------------------------------------------------------------------------
  /// \brief Poll the receiver for the configuration of the given port
  ///
  /// \param portIdentifier Identifier of the port to poll config for
  void pollPortConfiguration(uint8_t portIdentifier = 3);

  /// \brief Poll the receiver for the configuration of the current port
  ///
  void pollCurrentPortConfiguration();

  /// \brief Poll the receiver for the current navigation parameters
  bool pollNavigationParamterConfiguration();

  //----------------------------------------------------------------------------
  // Receiver Gen9 ITFM Configuration Messages
  //----------------------------------------------------------------------------
  
  /// \brief Enable/Disable the Interference Monitor
  ///
  /// \param enable True/False to enable/disable
  bool cfgItfmEnable(bool enable);

  /// \brief Set the Broadband Jamming threshold in the Interference Monitor
  ///
  /// \param threshold 
  bool cfgItfmBbJamThreshold(uint8_t threshold = 3);

  /// \brief Set the CW Jamming threshold in the Interference Monitor
  ///
  /// \param threshold 
  bool cfgItfmCwJamThreshold(uint8_t threshold = 15);

  //----------------------------------------------------------------------------
  // Aiding Data Polling Messages
  //----------------------------------------------------------------------------

  /// \brief Poll the receiver for the given class and message ID
  /// \param classId Class ID of the message to poll (E.g. NAV = 0x01)
  /// \param msgId Message ID of the message to poll (E.g. NAV-SOL = 0x06)
  bool pollMessage(uint8_t classId, uint8_t msgId);

  /// \brief Poll the receiver for the given class and message ID
  ///
  /// Some messages that include repeated fields for each satellite allow
  /// data for a single SV to be polled.
  ///
  /// \param classId Class ID of the message to poll (E.g. NAV = 0x01)
  /// \param msgId Message ID of the message to poll (E.g. NAV-SOL = 0x06)
  /// \param msgId Satellite ID of the message to poll
  bool pollMessageIndSV(uint8_t classId, uint8_t msgId, uint8_t svId);

  /// \brief Poll the receiver for ephemeris data
  /// \param svId Satellite ID number to request data for.  All SVs = -1
  bool pollEphem(int8_t svid = -1);

  /// \brief Poll the receiver for almanac data
  /// \param svId Satellite ID number to request data for.  All SVs = -1
  bool pollAlmanac(int8_t svid = -1);

  /// \brief Poll the receiver for UTC and Ionospheric correction data
  bool pollHui();

  /// \brief Poll the receiver for initial position, time, and clock
  bool pollAidIni();

  /// \brief Poll the receiver for all available aiding data
  bool pollAllAidData();

  /// \brief Poll the receiver for raw measurements (e.g. pseudorange, etc.)
  bool pollRawMeasurementData();

  /// \brief Poll the receiver for the status of each satellite
  bool pollSvStatus();

  /// \brief Poll the receiver for the status of each satellite
  bool pollSvInfo();

  /// \brief Poll the receiver for the current navigation solution status
  bool pollNavStatus();

  //----------------------------------------------------------------------------
  // Send Aiding Data to Receiver
  //----------------------------------------------------------------------------

  /// \brief Send the given message to the receiver
  ///
  /// \param msg Byte array containing the data to send
  /// \param length Number of bytes to send
  /// \returns True if the data was send successfully
  bool sendMessage(uint8_t* msg, size_t length);

  /// \brief Send initial position and time data to the receiver
  bool sendAidIni(AidIni ini);

  /// \brief Send ephemeris data to the receiver
  bool sendAidEphem(Ephemerides ephems);

  // TODO: why would you ever do this???????
  /// \brief Send raw measurement data to the receiver
  bool sendRawMeasurement(RawMeas rawMeas);

  /// \brief Send UTC and Ionospheric data to the receiver
  bool sendAidHui(AidHui hui);

  /// \brief Send almanac data to the receiver
  bool sendAidAlm(Almanac almanac);

  //----------------------------------------------------------------------------
  // Send data handlers
  //----------------------------------------------------------------------------

  void set_rxm_svsi_Handler(const RxmSvsiHandler Handler)
  {
    rxmSvsiHandler_ = Handler;
  };
  void set_rxm_subframe_Handler(const RxmSubframeHandler Handler)
  {
    rxmSubframeHandler_ = Handler;
  };
  void set_rxm_raw_Handler(const RxmRawHandler Handler)
  {
    rxmRawHandler_ = Handler;
  };
  void set_rxm_rawx_Handler(const RxmRawxHandler Handler)
  {
    rxmRawxHandler_ = Handler;
  };
  void set_aid_alm_Handler(const AidAlmHandler Handler)
  {
    aidAlmHandler_ = Handler;
  };
  void set_aid_eph_Handler(const AidEphHandler Handler)
  {
    aidEphHandler_ = Handler;
  };
  void set_aid_hui_Handler(const AidHuiHandler Handler)
  {
    aidHuiHandler_ = Handler;
  };
  void set_aid_ini_Handler(const AidIniHandler Handler)
  {
    aidIniHandler_ = Handler;
  };
  void set_nav_status_Handler(const NavStatusHandler Handler)
  {
    navStatusHandler_ = Handler;
  };
  void set_nav_solution_Handler(const NavSolHandler Handler)
  {
    navSolHandler_ = Handler;
  };
  void set_nav_pvt_Handler(const NavPvtHandler Handler)
  {
    navPvtHandler_ = Handler;
  };
  void set_nav_position_ecef_Handler(const NavPosECEFHandler Handler)
  {
    navPosEcefHandler_ = Handler;
  };
  void set_nav_position_llh_Handler(const NavPosLLHHandler Handler)
  {
    navPosLlhHandler_ = Handler;
  };
  void set_nav_hp_position_ecef_Handler(const NavHPPosPECEFHandler Handler)
  {
    navHpPosEcefHandler_ = Handler;
  };
  void set_nav_hp_position_llh_Handler(const NavHPPosLLHHandler Handler)
  {
    navHpPosLlhHandler_ = Handler;
  };
  void set_nav_sv_info_Handler(const NavSVInfoHandler Handler)
  {
    navSvInfoHandler_ = Handler;
  };
  void set_nav_sat_Handler(const NavSatHandler Handler)
  {
    navSatHandler_ = Handler;
  };
  void set_nav_sig_Handler(const NavSigHandler Handler)
  {
    navSigHandler_ = Handler;
  };
  void set_nav_gps_time_Handler(const NavGPSTimeHandler Handler)
  {
    navGpsTimeHandler_ = Handler;
  };
  void set_nav_utc_time_Handler(const NavUTCTimeHandler Handler)
  {
    navUtcTimeHandler_ = Handler;
  };
  void set_nav_dop_Handler(const NavDOPHandler Handler)
  {
    navDopHandler_ = Handler;
  };
  void set_nav_dgps_Handler(const NavDGPSHandler Handler)
  {
    navDgpsHandler_ = Handler;
  };
  void set_nav_clock_Handler(const NavClockHandler Handler)
  {
    navClockHandler_ = Handler;
  };
  void set_nav_vel_ned_Handler(const NavVelNedHandler Handler)
  {
    navVelNedHandler_ = Handler;
  };
  void set_port_settings_Handler(const PortSettingsHandler Handler)
  {
    portSettingsHandler_ = Handler;
  };
  void set_configure_navigation_parameters_Handler(
    const ConfigureNavigationParametersHandler Handler)
  {
    navigationParametersHandler_ = Handler;
  };
  void set_parsed_ephem_Handler(const ParsedEphemHandler Handler)
  {
    parsedEphemHandler_ = Handler;
  };
  void set_mon_gnss_Handler(const MonitorGnssHandler& Handler)
  {
    monGnssHandler_ = Handler;
  };

  void set_mon_hardware_status_Handler(const MonitorHardwareHandler& Handler)
  {
    monHardwareHandler_ = Handler;
  };

  void set_mon_extended_hardware_status_Handler(
    const MonitorExtHardwareHandler& Handler)
  {
    monExtHardwareHandler_ = Handler;
  };

  void set_mon_version_info_Handler(const MonitorVersionHandler& Handler)
  {
    monVersionHandler_ = Handler;
  };

  void set_mon_span_handler(const MonSpanHandler& handler)
  {
    monSpanHandler_ = handler;
  };

  void set_mon_rf_handler(const MonRfHandler& handler)
  {
    monRfHandler_ = handler;
  };

  void set_rxm_subframe_X_Handler(const RxmSubframeXHandler Handler)
  {
    rxmSubframeXHandler_ = Handler;
  };

  void setRawSerialDataCallback(const rawSerialDataHandler Handler)
  {
    rawSerialDataHandler_ = Handler;
  }

  void setLogCallback(logutils::LogCallback logCallback)
  {
    this->log_ = logCallback;
  };

  void calculateCheckSum(uint8_t* in, size_t length, uint8_t* out);

  //****************************************************************************
  // Internal Receiver Interface Class Settings Setters

  void setProtocolSettings(bool ubx_in,
                           bool ubx_out,
                           bool nmea_in,
                           bool nmea_out)
  {
    protocolUbxIn_   = ubx_in;
    protocolUbxOut_  = ubx_out;
    protocolNmeaIn_  = nmea_in;
    protocolNmeaOut_ = nmea_out;
  }

  void setAgpsPollingSettings(bool pollForPvt,
                              bool pollForEphem,
                              bool pollForAlmanac,
                              bool pollForUtcIono,
                              bool pollForRaw)
  {
    pollForPvt_     = pollForPvt;
    pollForEphem_   = pollForEphem;
    pollForAlmanac_ = pollForAlmanac;
    pollForUtcIono_ = pollForUtcIono;
    pollForRaw_     = pollForRaw;
  }

  void setDynamicModel(std::string dm) { dynamicModel_ = dm; };
  void setFixMode(std::string fm) { fixMode_ = fm; };

  // ITFM Settings
  void setCfgItfmEnable(bool in){cfgItfmEnable_=in;};
  void setCfgItfmBbJamThreshold(uint8_t in){cfgItfmBbJamThreshold_=in;};
  void setCfgItfmCwJamThreshold(uint8_t in){cfgItfmCwJamThreshold_=in;};

  void setPort(std::string port) { port_ = port; };
  void setBaudrate(int baud) { baudrate_ = baud; };

  /// \brief Set monitorConnectionTimeout_ for thread monitorConnection()
  void setMonitorConnectionTimeout(unsigned int timeout)
  {
    monitorConnectionTimeout_ = timeout;
  };
  void setMonitorConnectionPeriod(unsigned int rate)
  {
    monitorConnectionPeriod_ = rate;
  };
  void setAgpsRequestPeriod(double period)
  {
    receiverAgpsRequestPeriod_ = period;
  };

  std::vector<std::string> msgClasses_;
  std::vector<std::string> msgIds_;
  std::vector<uint8_t>     msgRates_;

private:
  // Starts a thread that runs 'ReadSerialPort' which constatly reads
  // from the serial port.  When valid data is received, parse and then
  // the data Handler functions are called.
  void startReading();

  // Method run in a seperate thread that continuously reads from the
  // serial port.  When a complete packet is received, the parse
  // method is called to process the data
  void readSerialPort();

  // Buffer and parse data read from the serial port.  Called each time data
  // is read in readSerialPort().  Calls parseLog() when a complete UBX
  // message is found
  void bufferIncomingData(uint8_t* msg, size_t length);

  // Parse a completed UBX message into its given structure
  void parseLog(uint8_t* log, size_t logId);

  bool pollForData(bool poll_pvt,
                   bool poll_ephemeris,
                   bool poll_almanac,
                   bool poll_utcIono,
                   bool poll_rawData);

  void pollForAgpsData();

  /// \brief Send configuration parameters into receiver. Done after connection
  /// established
  bool configureReceiver();

  /// \brief Starts new thread running monitorConnection()
  void startConnectionMonitor();

  /// \brief  Thread monitoring the connection status of the GPS receiver
  void monitorConnection();

  std::atomic_bool isConnected_;  // True if the receiver is connected

  std::atomic_bool reading_;  // True if the read thread is running.
  std::atomic_bool
    stopReading_;  // Setting true shuts down the readSerialPort thread

  // Set to false to stop the thread
  std::atomic_bool activeMonitorConnectionThread_;  // True if MonitorConnection
                                                    // thread is running
  std::atomic_bool stopMonitorConnectionThread_;  // Setting true shuts down the
                                                  // MonitorConnection thread

  /// Mutex protecting lastMessageTime_
  std::mutex lastMessageTimeMutex_;
  /// Time at which last message was received from the receiver. Used to
  /// determine if a connection to the receiver has been lost
  std::chrono::system_clock::time_point lastMessageTime_;

  void setLastMessageTime(std::chrono::system_clock::time_point time)
  {
    std::lock_guard<std::mutex> lock(lastMessageTimeMutex_);
    lastMessageTime_ = time;
  }
  std::chrono::system_clock::time_point getLastMessageTime()
  {
    std::lock_guard<std::mutex> lock(lastMessageTimeMutex_);
    return lastMessageTime_;
  }

  // Serial port object for accessing the Ublox
  serial::Serial* serialPort_;

  // Function to parse out useful ephemeris parameters
  ParsedEphemData ParseAidEph(EphemSV ubxEph);

  //****************************************************************************
  // Receiver Configuration Setting Setters (PRIVATE)
  bool        protocolUbxIn_;
  bool        protocolUbxOut_;
  bool        protocolNmeaIn_;
  bool        protocolNmeaOut_;
  std::string dynamicModel_;
  // int dynamicModel_;
  std::string fixMode_;
  // int fixMode_;
  
  // ITFM Settings
  bool cfgItfmEnable_; // Enable/Disable ITFM
  uint8_t cfgItfmBbJamThreshold_; // ITFM Broadband Jamming Threshold
  uint8_t cfgItfmCwJamThreshold_; // ITFM CW Jamming Threshold

  std::atomic_uint monitorConnectionTimeout_;
  std::atomic_uint monitorConnectionPeriod_;
  double           receiverAgpsRequestPeriod_;
  std::string      port_;
  int              baudrate_;
  bool             pollForPvt_;
  bool             pollForEphem_;
  bool             pollForAlmanac_;
  bool             pollForUtcIono_;
  bool             pollForRaw_;

  //----------------------------------------------------------------------------
  // Diagnostic Handlers
  //----------------------------------------------------------------------------
  logutils::LogCallback log_;

  //----------------------------------------------------------------------------
  // Data Handlers
  //----------------------------------------------------------------------------
  HandleAcknowledgementHandler handleAcknowledgement_;

  //----------------------------------------------------------------------------
  // New Data Handlers
  //----------------------------------------------------------------------------
  PortSettingsHandler                  portSettingsHandler_;
  ConfigureNavigationParametersHandler navigationParametersHandler_;
  NavPosECEFHandler                    navPosEcefHandler_;
  NavPosLLHHandler                     navPosLlhHandler_;
  NavHPPosPECEFHandler                 navHpPosEcefHandler_;
  NavHPPosLLHHandler                   navHpPosLlhHandler_;
  NavSolHandler                        navSolHandler_;
  NavPvtHandler                        navPvtHandler_;
  NavStatusHandler                     navStatusHandler_;
  NavVelNedHandler                     navVelNedHandler_;
  NavSVInfoHandler                     navSvInfoHandler_;
  NavSatHandler                        navSatHandler_;
  NavSigHandler                        navSigHandler_;
  NavGPSTimeHandler                    navGpsTimeHandler_;
  NavUTCTimeHandler                    navUtcTimeHandler_;
  NavDOPHandler                        navDopHandler_;
  NavDGPSHandler                       navDgpsHandler_;
  NavClockHandler                      navClockHandler_;
  AidAlmHandler                        aidAlmHandler_;
  AidEphHandler                        aidEphHandler_;
  AidHuiHandler                        aidHuiHandler_;
  AidIniHandler                        aidIniHandler_;
  rawSerialDataHandler                 rawSerialDataHandler_;
  RxmRawHandler                        rxmRawHandler_;
  RxmRawxHandler                       rxmRawxHandler_;
  RxmSubframeHandler                   rxmSubframeHandler_;
  RxmSvsiHandler                       rxmSvsiHandler_;
  RxmSubframeXHandler                  rxmSubframeXHandler_;
  ParsedEphemHandler                   parsedEphemHandler_;
  MonitorGnssHandler                   monGnssHandler_;
  MonitorHardwareHandler               monHardwareHandler_;
  MonitorExtHardwareHandler            monExtHardwareHandler_;
  MonitorVersionHandler                monVersionHandler_;
  MonSpanHandler                       monSpanHandler_;
  MonRfHandler                         monRfHandler_;

  //----------------------------------------------------------------------------
  // Incoming data buffers
  //----------------------------------------------------------------------------
  // Serial port read buffer
  uint8_t buffer_[MAX_UBX_SIZE];
  // Buffer containing partial messages read from the serial port
  unsigned char dataBuffer_[MAX_UBX_SIZE];
  // bytes remaining to be read in the current message
  size_t bytesRemaining_;
  // Index into dataBuffer_
  size_t bufferIndex_;
  // time stamp when last serial port read completed
  SysClkTimePoint readTimestamp_;
  // last parsed message ID
  unsigned short msgId_;

  ublox::PortIdentifier connectedPortId_;
  // uint32_t baudrate_;

  //----------------------------------------------------------------------------
  // Conversion Utilitites
  //----------------------------------------------------------------------------
  inline int convertNavClass(const std::string& classIn)
  {
    if (!classIn.compare("ACK"))
    {
      return (MSG_CLASS_ACK);
    }
    else if (!classIn.compare("AID"))
    {
      return (MSG_CLASS_AID);
    }
    else if (!classIn.compare("CFG"))
    {
      return (MSG_CLASS_CFG);
    }
    else if (!classIn.compare("ESF"))
    {
      return (MSG_CLASS_ESF);
    }
    else if (!classIn.compare("INF"))
    {
      return (MSG_CLASS_INF);
    }
    else if (!classIn.compare("MON"))
    {
      return (MSG_CLASS_MON);
    }
    else if (!classIn.compare("NAV"))
    {
      return (MSG_CLASS_NAV);
    }
    else if (!classIn.compare("RXM"))
    {
      return (MSG_CLASS_RXM);
    }
    else if (!classIn.compare("TIM"))
    {
      return (MSG_CLASS_TIM);
    }
    else
    {
      std::ostringstream output;
      output << "Unsupported Navigation Class " << classIn;
      log_(output.str(), logutils::LogLevel::Error);
      return (-1);
    }
  }

  inline int convertMsgId(const std::string& idIn)
  {
    if (!idIn.compare("ACK_ACK"))
    {
      return (MSG_ID_ACK_ACK);
    }
    else if (!idIn.compare("ACK_NAK"))
    {
      return (MSG_ID_ACK_NAK);
    }
    else if (!idIn.compare("AID_ALM"))
    {
      return (MSG_ID_AID_ALM);
    }
    else if (!idIn.compare("AID_ALPSRV"))
    {
      return (MSG_ID_AID_ALPSRV);
    }
    else if (!idIn.compare("AID_ALP"))
    {
      return (MSG_ID_AID_ALP);
    }
    else if (!idIn.compare("AID_AOP"))
    {
      return (MSG_ID_AID_AOP);
    }
    else if (!idIn.compare("AID_DATA"))
    {
      return (MSG_ID_AID_DATA);
    }
    else if (!idIn.compare("AID_EPH"))
    {
      return (MSG_ID_AID_EPH);
    }
    else if (!idIn.compare("AID_HUI"))
    {
      return (MSG_ID_AID_HUI);
    }
    else if (!idIn.compare("AID_INI"))
    {
      return (MSG_ID_AID_INI);
    }
    else if (!idIn.compare("AID_REQ"))
    {
      return (MSG_ID_AID_REQ);
    }
    else if (!idIn.compare("CFG_ANT"))
    {
      return (MSG_ID_CFG_ANT);
    }
    else if (!idIn.compare("CFG_CNFGR"))
    {
      return (MSG_ID_CFG_CNFGR);
    }
    else if (!idIn.compare("CFG_DAT"))
    {
      return (MSG_ID_CFG_DAT);
    }
    else if (!idIn.compare("CFG_EKF"))
    {
      return (MSG_ID_CFG_EKF);
    }
    else if (!idIn.compare("CFG_ESFGWT"))
    {
      return (MSG_ID_CFG_ESFGWT);
    }
    else if (!idIn.compare("CFG_FXN"))
    {
      return (MSG_ID_CFG_FXN);
    }
    else if (!idIn.compare("CFG_ITFM"))
    {
      return (MSG_ID_CFG_ITFM);
    }
    else if (!idIn.compare("CFG_MSG"))
    {
      return (MSG_ID_CFG_MSG);
    }
    else if (!idIn.compare("CFG_NAV5"))
    {
      return (MSG_ID_CFG_NAV5);
    }
    else if (!idIn.compare("CFG_NAVX5"))
    {
      return (MSG_ID_CFG_NAVX5);
    }
    else if (!idIn.compare("CFG_NMEA"))
    {
      return (MSG_ID_CFG_NMEA);
    }
    else if (!idIn.compare("CFG_NVS"))
    {
      return (MSG_ID_CFG_NVS);
    }
    else if (!idIn.compare("CFG_PM2"))
    {
      return (MSG_ID_CFG_PM2);
    }
    else if (!idIn.compare("CFG_PM"))
    {
      return (MSG_ID_CFG_PM);
    }
    else if (!idIn.compare("CFG_PRT"))
    {
      return (MSG_ID_CFG_PRT);
    }
    else if (!idIn.compare("CFG_RATE"))
    {
      return (MSG_ID_CFG_RATE);
    }
    else if (!idIn.compare("CFG_RINV"))
    {
      return (MSG_ID_CFG_RINV);
    }
    else if (!idIn.compare("CFG_RST"))
    {
      return (MSG_ID_CFG_RST);
    }
    else if (!idIn.compare("CFG_RXM"))
    {
      return (MSG_ID_CFG_RXM);
    }
    else if (!idIn.compare("CFG_SBAS"))
    {
      return (MSG_ID_CFG_SBAS);
    }
    else if (!idIn.compare("CFG_TMODE2"))
    {
      return (MSG_ID_CFG_TMODE2);
    }
    else if (!idIn.compare("CFG_TMODE"))
    {
      return (MSG_ID_CFG_TMODE);
    }
    else if (!idIn.compare("CFG_TP5"))
    {
      return (MSG_ID_CFG_TP5);
    }
    else if (!idIn.compare("CFG_TP"))
    {
      return (MSG_ID_CFG_TP);
    }
    else if (!idIn.compare("CFG_US"))
    {
      return (MSG_ID_CFG_USB);
    }
    else if (!idIn.compare("ESF_MEAS"))
    {
      return (MSG_ID_ESF_MEAS);
    }
    else if (!idIn.compare("ESF_STATUS"))
    {
      return (MSG_ID_ESF_STATUS);
    }
    else if (!idIn.compare("INF_DEBUG"))
    {
      return (MSG_ID_INF_DEBUG);
    }
    else if (!idIn.compare("INF_ERROR"))
    {
      return (MSG_ID_INF_ERROR);
    }
    else if (!idIn.compare("INF_NOTICE"))
    {
      return (MSG_ID_INF_NOTICE);
    }
    else if (!idIn.compare("INF_TEST"))
    {
      return (MSG_ID_INF_TEST);
    }
    else if (!idIn.compare("INF_WARNING"))
    {
      return (MSG_ID_INF_WARNING);
    }
    else if (!idIn.compare("MON_HW2"))
    {
      return (MSG_ID_MON_HW2);
    }
    else if (!idIn.compare("MON_HW"))
    {
      return (MSG_ID_MON_HW);
    }
    else if (!idIn.compare("MON_IO"))
    {
      return (MSG_ID_MON_IO);
    }
    else if (!idIn.compare("MON_MSGPP"))
    {
      return (MSG_ID_MON_MSGPP);
    }
    else if (!idIn.compare("MON_RXBUF"))
    {
      return (MSG_ID_MON_RXBUF);
    }
    else if (!idIn.compare("MON_RXR"))
    {
      return (MSG_ID_MON_RXR);
    }
    else if (!idIn.compare("MON_TXBUF"))
    {
      return (MSG_ID_MON_TXBUF);
    }
    else if (!idIn.compare("MON_VER"))
    {
      return (MSG_ID_MON_VER);
    }
    else if (!idIn.compare("MON_RF"))
    {
      return (MSG_ID_MON_RF);
    }
    else if (!idIn.compare("MON_SPAN"))
    {
      return (MSG_ID_MON_SPAN);
    }
    else if (!idIn.compare("NAV_AOPSTATUS"))
    {
      return (MSG_ID_NAV_AOPSTATUS);
    }
    else if (!idIn.compare("NAV_CLOCK"))
    {
      return (MSG_ID_NAV_CLOCK);
    }
    else if (!idIn.compare("NAV_DGPS"))
    {
      return (MSG_ID_NAV_DGPS);
    }
    else if (!idIn.compare("NAV_DOP"))
    {
      return (MSG_ID_NAV_DOP);
    }
    else if (!idIn.compare("NAV_EKFSTATUS"))
    {
      return (MSG_ID_NAV_EKFSTATUS);
    }
    else if (!idIn.compare("NAV_POSECEF"))
    {
      return (MSG_ID_NAV_POSECEF);
    }
    else if (!idIn.compare("NAV_POSLLH"))
    {
      return (MSG_ID_NAV_POSLLH);
    }
    else if (!idIn.compare("NAV_SBAS"))
    {
      return (MSG_ID_NAV_SBAS);
    }
    else if (!idIn.compare("NAV_SOL"))
    {
      return (MSG_ID_NAV_SOL);
    }
    else if (!idIn.compare("NAV_PVT"))
    {
      return (MSG_ID_NAV_PVT);
    }
    else if (!idIn.compare("NAV_STATUS"))
    {
      return (MSG_ID_NAV_STATUS);
    }
    else if (!idIn.compare("NAV_SVINFO"))
    {
      return (MSG_ID_NAV_SVINFO);
    }
    else if (!idIn.compare("NAV_SAT"))
    {
      return (MSG_ID_NAV_SAT);
    }
    else if (!idIn.compare("NAV_SIG"))
    {
      return (MSG_ID_NAV_SIG);
    }
    else if (!idIn.compare("NAV_TIMEGPS"))
    {
      return (MSG_ID_NAV_TIMEGPS);
    }
    else if (!idIn.compare("NAV_TIMEUTC"))
    {
      return (MSG_ID_NAV_TIMEUTC);
    }
    else if (!idIn.compare("NAV_VELECEF"))
    {
      return (MSG_ID_NAV_VELECEF);
    }
    else if (!idIn.compare("NAV_VELNED"))
    {
      return (MSG_ID_NAV_VELNED);
    }
    else if (!idIn.compare("RXM_ALM"))
    {
      return (MSG_ID_RXM_ALM);
    }
    else if (!idIn.compare("RXM_EPH"))
    {
      return (MSG_ID_RXM_EPH);
    }
    else if (!idIn.compare("RXM_PMREQ"))
    {
      return (MSG_ID_RXM_PMREQ);
    }
    else if (!idIn.compare("RXM_RAW"))
    {
      return (MSG_ID_RXM_RAW);
    }
    else if (!idIn.compare("RXM_RAWX"))
    {
      return (MSG_ID_RXM_RAWX);
    }
    else if (!idIn.compare("RXM_SFRB"))
    {
      return (MSG_ID_RXM_SFRB);
    }
    else if (!idIn.compare("RXM_SFRBX"))
    {
      return (MSG_ID_RXM_SFRBX);
    }
    else if (!idIn.compare("RXM_SVSI"))
    {
      return (MSG_ID_RXM_SVSI);
    }
    else if (!idIn.compare("TIM_SVIN"))
    {
      return (MSG_ID_TIM_SVIN);
    }
    else if (!idIn.compare("TIM_TM2"))
    {
      return (MSG_ID_TIM_TM2);
    }
    else if (!idIn.compare("TIM_TP"))
    {
      return (MSG_ID_TIM_TP);
    }
    else if (!idIn.compare("TIM_VRFY"))
    {
      return (MSG_ID_TIM_VRFY);
    }
    else
    {
      std::ostringstream output;
      output << "Unsupported Message ID " << idIn;
      log_(output.str(), logutils::LogLevel::Error);
      return (-1);
    }
  }
  inline int convertModel(const std::string& modelIn)
  {
    if (!modelIn.compare("portable"))
    {
      return (0);
    }
    else if (!modelIn.compare("stationary"))
    {
      return (2);
    }
    else if (!modelIn.compare("pedestrian"))
    {
      return (3);
    }
    else if (!modelIn.compare("automotive"))
    {
      return (4);
    }
    else if (!modelIn.compare("sea"))
    {
      return (5);
    }
    else if (!modelIn.compare("air1g"))
    {
      return (6);
    }
    else if (!modelIn.compare("air2g"))
    {
      return (7);
    }
    else if (!modelIn.compare("air4g"))
    {
      return (8);
    }
    else
    {
      std::ostringstream out;
      out << "Unsupported model type " << modelIn;
      log_(out.str(), logutils::LogLevel::Error);
      return (-1);
    }
  }

  inline int convertFix(const std::string& fixIn)
  {
    // TODO: replace these with enums
    if (!fixIn.compare("2D"))
    {
      return (1);
    }
    else if (!fixIn.compare("3D"))
    {
      return (2);
    }
    else if (!fixIn.compare("auto"))
    {
      return (3);
    }
    else
    {
      std::ostringstream out;
      out << "Unsupported fix type " << fixIn;
      log_(out.str(), logutils::LogLevel::Error);
      return (-1);
    }
  }
};  // end class Ublox
}  // end namespace ublox
#endif
//------------------------------------------------------------------------------
//                                UNCLASSIFIED
//------------------------------------------------------------------------------