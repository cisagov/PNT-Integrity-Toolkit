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
//
// Class definition for main toolkit application
// Josh Clanton <josh.clanton@is4s.com>
// April 20, 2020
//============================================================================//
#include <unistd.h>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <memory>

#include "integrity_toolkit/ToolkitApplication.hpp"
#include "usrp_utilities/SurrpasUsrpRecord.hpp"

namespace integrity_toolkit
{
ToolkitApplication::ToolkitApplication(int argc, char* argv[])
  : qApp_(argc, argv)
  , log_(logutils::printLogToStdOut)
  , minLogLevel_(logutils::LogLevel::Info)
  , pushMode_(false)
  , checkCount_(0)
  , seqNum_(0)
  , pvtUblox1SeqNum_(0)
  , pvtUblox2SeqNum_(0)
  , rawMeasXUblox1SeqNum_(0)
  , rawMeasXUblox2SeqNum_(0)
  , rxmSubframeUblox1SeqNum_(0)
  , rxmSubframeUblox2SeqNum_(0)
  , monRfUblox1SeqNum_(0)
  , rngPosCheckName_("range_pos_check")
  , rngPosCheckPtr_(nullptr)
  , useStaticMode_(false)
  , rx1rx2Baseline_(100e3)
  , useStaticPosCheck_(false)
  , staticPosCheckName_("static_pos_check")
  , staticPosCheckPtr_(nullptr)
  , useCnoCheck_(false)
  , cnoCheckName_("cno_check")
  , cnoCheckPtr_(nullptr)
  , usePosJumpCheck_(false)
  , posJumpCheckName_("pos_jump_check")
  , usePosVelConsCheck_(false)
  , posVelConsCheckName_("pos_vel_cons_check")
  , posVelConsCheckPtr_(nullptr)
  , useAoaCheck_(false)
  , aoaCheckName_("aoa_check")
  , useAgcCheck_(false)
  , agcCheckName_("agc_check")
  , agcCheckPtr_(nullptr)
  , useClockBiasCheck_(false)
  , clockBiasCheckName_("clock_bias_check")
  , clockBiasCheckPtr_(nullptr)
#ifdef PNT_INTEGRITY_INCLUDES_ACQ_CHECK
  , acqCheckName_("acq_check")
  , acqCheckPtr_(nullptr)
#endif
{
  std::string ublox1CfgFile    = "ublox1.yaml";
  std::string ublox2CfgFile    = "ublox2.yaml";
  std::string usrpCfgFile      = "usrp.yaml";
  std::string integrityCfgFile = "integrity.yaml";
  std::string nmeaPort         = "/dev/ttyUSB99";
  std::string lcdPort          = "/dev/ttyACM0";
  bool        dualRxMode       = false;
  bool        logging          = false;
  bool        usrpMode         = false;
  // Process arguments
  auto print_usage = []() {
    std::cout << "integrity_toolkit usage" << std::endl;
    std::cout << "  integrity_toolkit [OPTION]..." << std::endl;
    std::cout << "    -u FILENAME      Ublox 1 config YAML file" << std::endl;
    std::cout << "    -d               Enables dual Rx Mode" << std::endl;
    std::cout << "    -b FILENAME      Ublox 2 config YAML file (Also requires "
                 "-d option)"
              << std::endl;
    std::cout << "    -r FILENAME      USRP config YAML file (if equipped)"
              << std::endl;
    std::cout << "    -i FILENAME      integrity config YAML file (if equipped)"
              << std::endl;
    std::cout << "    -p PORT          serial port for NMEA output"
              << std::endl;
    std::cout << "    -x LCD PORT      serial port for LCD display "
              << std::endl;
    std::cout << "    -l               Enables Logging" << std::endl;
    std::cout << "  if options are not specified, then defaults are:"
              << std::endl;
    std::cout << "    ublox1 config  = ublox1.yaml " << std::endl;
    std::cout << "    ublox2 config  = ublox2.yaml " << std::endl;
    std::cout << "    usrp config  = usrp.yaml " << std::endl;
    std::cout << "    integrity config  = integrity.yaml " << std::endl;
    std::cout << "    lcdPort = /dev/ttyACM0" << std::endl;
    std::cout << "    port = /dev/ttyUSB99" << std::endl << std::endl;
  };

  bool validArgs = true;
  int  opt       = 0;
  while ((opt = getopt(argc, argv, "dlx:u:b:r:i:p:h")) != -1)
  {
    switch (opt)
    {
      case 'd':
        std::cout << "Dual receiver mode" << std::endl;
        dualRxMode = true;
        break;
      case 'l':
        std::cout << "Logging mode" << std::endl;
        logging = true;
        break;
      case 'u':
        std::cout << "Using Ublox config file: ";
        if (optarg)
        {
          ublox1CfgFile = optarg;
          std::cout << ublox1CfgFile;
        }
        std::cout << std::endl;
        break;
      case 'b':
        std::cout << "Using Ublox 2 config file: ";
        if (optarg)
        {
          ublox2CfgFile = optarg;
          std::cout << ublox2CfgFile;
        }
        std::cout << std::endl;
        break;
      case 'r':
        std::cout << "Using USRP config file: ";
        if (optarg)
        {
          usrpMode    = true;
          usrpCfgFile = optarg;
          std::cout << usrpCfgFile;
        }
        std::cout << std::endl;
        break;
      case 'i':
        std::cout << "Using integrity config file: ";
        if (optarg)
        {
          integrityCfgFile = optarg;
          std::cout << integrityCfgFile;
        }
        std::cout << std::endl;
        break;
      case 'x':
        std::cout << "Using lcd serial port: ";
        if (optarg)
        {
          lcdPort = optarg;
          std::cout << "serial port for LCD display = " << lcdPort;
        }
        std::cout << std::endl;
        break;
      case 'p':
        std::cout << "Using NMEA serial port: ";
        if (optarg)
        {
          pushMode_ = true;
          nmeaPort  = optarg;
          std::cout << "serial port for LCD display = " << nmeaPort;
        }
        std::cout << std::endl;
        break;
      default:
        print_usage();
        validArgs = false;
    }
  }

  // configure all components if args are valid
  if (validArgs)
  {
    bool goodToRun = true;

    // configure lcd display
    if (lcdDisplay_.initialize(lcdPort, 115200))
    {
      goodToRun &= true;
    }
    else
    {
      goodToRun &= false;
      log_("LCD not properly configured", logutils::LogLevel::Error);
    }

    // configure ublox 1
    if (configureUblox1(ublox1CfgFile))
    {
      ublox1_.run();
      goodToRun &= true;  // allow running if ubx1 is configured
    }
    else
    {
      goodToRun &= false;
      log_("Ublox 1 not properly configured", logutils::LogLevel::Error);
    }

    // if dual mode, configure ublox 2
    if (dualRxMode)
    {
      if (configureUblox2(ublox2CfgFile))
      {
        ublox2_.run();
        goodToRun &= true;  // allow running if ubx1 is configured
      }
      else
      {
        goodToRun &= false;
        log_("Ublox 2 not properly configured", logutils::LogLevel::Error);
      }
    }

    // configure usrp stream (if specified)
    if (usrpMode)
    {
      if (configureIfDataStream(usrpCfgFile))
      {
        goodToRun &= true;  // allow running if IF configures
      }
      else
      {
        goodToRun &= false;
        log_("IF Data stream not properly configured",
             logutils::LogLevel::Error);
      }
    }

    // configure integrity
    if (configureIntegrity(integrityCfgFile))
    {
      goodToRun &= true;  // allow running if Integrity configures
    }
    else
    {
      goodToRun &= false;
      log_("Integrity monitor not properly configured",
           logutils::LogLevel::Error);
    }

    if (pushMode_)
    {
      if (connectNmeaPort(nmeaPort))
      {
        goodToRun &= true;  // serial port configured successfully
      }
      else
      {
        goodToRun &= false;
        log_("Serial output port not properly configured",
             logutils::LogLevel::Error);
      }
    }

    // if all hardware is configured properly, proceed
    if (goodToRun)
    {
      // Starts creating a csv file for Assurance Reports & Assurance Report
      if (logging)
      {
        csvAssuranceReportWriter_.openFile(".", "Assurance_Report_Log", true);
        csvAssuranceReportsWriter_.openFile(".", "Assurance_Reports_Log", true);

        csvAssuranceReportWriter_.writeHeader(
          csv_write::AssuranceReportHeaderString);

        csvAssuranceReportsWriter_.writeHeader(
          csv_write::AssuranceReportsHeaderString);

        std::stringstream logMsg;
        logMsg << __FUNCTION__ << " Opened csvAssuranceReportWriter_ = "
               << csvAssuranceReportWriter_.getFilename() << std::endl;
        logMsg << __FUNCTION__ << " Opened csvAssuranceReportsWriter_ = "
               << csvAssuranceReportsWriter_.getFilename() << std::endl;
        log_(logMsg.str(), logutils::LogLevel::Info);
      }

      // start the lcd update thread
      lcdThreadptr_ = std::make_shared<std::thread>(std::bind(
        &ToolkitApplication::updateLcd, this, lcdExitSignal_.get_future()));

      // run the application
      run();
    }
  }  // if (validArgs)

  log_("Application terminating...", logutils::LogLevel::Info);
  lcdDisplay_.clearDisplay();
  lcdDisplay_.clearLeds();
}  // namespace integrity_toolkit

///==============================================================//
//----------------------- connectNmeaPort() ----------------------------//
//==============================================================//
bool ToolkitApplication::connectNmeaPort(std::string port, int baudrate)
{
  try
  {
    serialPtr_ = std::make_shared<serial::Serial>(
      port, baudrate, serial::Timeout::simpleTimeout(50));

    if (!serialPtr_->isOpen())
    {
      std::stringstream output;
      output << "Serial port: " << port << " failed to open.";
      log_(output.str(), logutils::LogLevel::Error);
      serialPtr_ = nullptr;
      return false;
    }
    else
    {
      std::stringstream output;
      output << "Serial port: " << port << " opened successfully.";
      log_(output.str(), logutils::LogLevel::Debug);
    }

    serialPtr_->flush();
  }
  catch (std::exception e)
  {
    std::stringstream output;
    output << "Failed to open port " << port << "  Err: " << e.what();
    log_(output.str(), logutils::LogLevel::Debug);
    serialPtr_ = nullptr;
    return false;
  }

  return true;
}

//==============================================================//
//------------------------- run() ------------------------------//
//==============================================================//
int ToolkitApplication::run()
{
  mainWindow_.show();
  return qApp_.exec();
};

//==============================================================//
//----------------------publishAssurance------------------------//
//==============================================================//
void ToolkitApplication::publishAssurance(
  const pnt_integrity::data::PositionVelocity msg)
{
  // Publish Cumulative Assurance Level
  pnt_integrity::data::AssuranceReport report;
  report.header = msg.header;
  report.state.setWithLevel(integrityMonitor_.getAssuranceLevel());
  mainWindow_.sendIntegrityAssuranceReport(report);

  //-------------- Publish Assurance Reports --------------------------
  pnt_integrity::data::AssuranceReports reports =
    integrityMonitor_.getAssuranceReports();
  reports.header = msg.header;
  mainWindow_.sendIntegrityAssuranceReports(reports);

  if (pushMode_ & (serialPtr_ != nullptr))
  {
    if (serialPtr_->isOpen())
    {
      // form the NMEA message for assurance level
      std::stringstream reportStream;
      double            timestamp =
        report.header.timestampValid.sec +
        (double)report.header.timestampValid.nanoseconds / (1e9);
      reportStream << std::setprecision(12);
      reportStream << "$GPSAR," << timestamp << ","
                   << report.state.getIntegerAssuranceValue() << ":";

      uint8_t checksum = computeNmeaChecksum(reportStream.str().c_str());
      reportStream << checksum;
      serialPtr_->write(reportStream.str());
    }
  }
};

uint8_t ToolkitApplication::computeNmeaChecksum(const char* sentence)
{
  // Support senteces with or without the starting dollar sign.
  if (*sentence == '$')
    sentence++;

  uint8_t checksum = 0x00;

  // The optional checksum is an XOR of all bytes between "$" and "*".
  while (*sentence && *sentence != '*')
    checksum ^= *sentence++;

  return checksum;
}

//==============================================================//
//---------------------- configureUblox1() ---------------------//
//==============================================================//
bool ToolkitApplication::configureUblox1(const std::string& yamlFilename)
{
  if (yamlFilename.empty())
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() Ublox config file not provided" << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Error);
    return false;
  }
  /// Bind data handlers to Ublox driver class data callbacks
  bindUblox1Callbacks();

  //****************************************************************************
  // Parse Ublox yaml config file
  try
  {
    yamlParserPtr_ = std::unique_ptr<yaml_parser::YamlParser>(
      new yaml_parser::YamlParser(yamlFilename,
                                  std::bind(&ToolkitApplication::handleLog,
                                            this,
                                            std::placeholders::_1,
                                            std::placeholders::_2)));
  }
  catch (std::exception& e)
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() YamlParser error: " << e.what() << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Error);
    return false;
  }

  std::string port;
  int         baudrate;
  // Ublox connection settings
  if (!yamlParserPtr_->readVariable<std::string>(port, "port") ||
      !yamlParserPtr_->readVariable<int>(baudrate, "baudrate"))
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() " << yamlFilename
            << " no port or baudrate provided.f" << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Error);
    return false;
  }

  ublox1_.setPort(port);
  ublox1_.setBaudrate(baudrate);

  ublox1_.setMonitorConnectionTimeout(
    yamlParserPtr_->readVariable<unsigned int>("connection_timeout", 10));
  ublox1_.setMonitorConnectionPeriod(
    yamlParserPtr_->readVariable<unsigned int>("connect_retry_period", 1));

  ublox1_.setAgpsPollingSettings(
    yamlParserPtr_->readVariable<bool>("poll_for_pvt", false),
    yamlParserPtr_->readVariable<bool>("poll_for_ephemeris", false),
    yamlParserPtr_->readVariable<bool>("poll_for_almanac", false),
    yamlParserPtr_->readVariable<bool>("poll_for_utcIono", false),
    yamlParserPtr_->readVariable<bool>("poll_for_raw", false));

  ublox1_.setProtocolSettings(
    yamlParserPtr_->readVariable<bool>("ublox_input", true),
    yamlParserPtr_->readVariable<bool>("ublox_output", true),
    yamlParserPtr_->readVariable<bool>("nmea_input", false),
    yamlParserPtr_->readVariable<bool>("nmea_output", false));

  ublox1_.setDynamicModel(
    yamlParserPtr_->readVariable<std::string>("dynamic_model", "automotive"));
  ublox1_.setFixMode(
    yamlParserPtr_->readVariable<std::string>("fix_mode", "auto"));

  ublox1_.msgClasses_ =
    yamlParserPtr_->readVariable<std::vector<std::string>>("messages_class");
  ublox1_.msgIds_ =
    yamlParserPtr_->readVariable<std::vector<std::string>>("messages_id");

  std::vector<int> tempV =
    yamlParserPtr_->readVariable<std::vector<int>>("messages_rate");
  std::vector<uint8_t> tempV2(tempV.begin(), tempV.end());
  ublox1_.msgRates_ = tempV2;

  return true;
}

//==============================================================//
//---------------------- bindUblox1Callbacks() -----------------//
//==============================================================//
void ToolkitApplication::bindUblox1Callbacks()
{
  //   set up logging handlers
  ublox1_.setLogCallback(std::bind(&ToolkitApplication::handleLog,
                                   this,
                                   std::placeholders::_1,
                                   std::placeholders::_2));

  ublox1_.set_nav_pvt_Handler(std::bind(&ToolkitApplication::handlePvtUblox1,
                                        this,
                                        std::placeholders::_1,
                                        std::placeholders::_2));

  ublox1_.set_rxm_rawx_Handler(
    std::bind(&ToolkitApplication::handleRawxMeasUblox1,
              this,
              std::placeholders::_1,
              std::placeholders::_2));

  ublox1_.set_nav_clock_Handler(
    std::bind(&ToolkitApplication::handleNavClockUblox1,
              this,
              std::placeholders::_1,
              std::placeholders::_2));

  ublox1_.set_nav_gps_time_Handler(
    std::bind(&ToolkitApplication::handleNavGpsTimeUblox1,
              this,
              std::placeholders::_1,
              std::placeholders::_2));

  ublox1_.set_mon_rf_handler(std::bind(&ToolkitApplication::handleMonRfUblox1,
                                       this,
                                       std::placeholders::_1,
                                       std::placeholders::_2));

  ublox1_.set_rxm_subframe_X_Handler(
    std::bind(&ToolkitApplication::handleRxmSubframeXUblox1,
              this,
              std::placeholders::_1,
              std::placeholders::_2));                                         
}

//==============================================================//
//---------------------- configureUblox2() ---------------------//
//==============================================================//
bool ToolkitApplication::configureUblox2(const std::string& yamlFilename)
{
  std::cout << "config ublox 2" << std::endl;
  if (yamlFilename.empty())
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() Ublox config file not provided" << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Error);
    return false;
  }
  /// Bind data handlers to Ublox driver class data callbacks
  bindUblox2Callbacks();

  //****************************************************************************
  // Parse Ublox yaml config file
  try
  {
    yamlParserPtr_ = std::unique_ptr<yaml_parser::YamlParser>(
      new yaml_parser::YamlParser(yamlFilename,
                                  std::bind(&ToolkitApplication::handleLog,
                                            this,
                                            std::placeholders::_1,
                                            std::placeholders::_2)));
  }
  catch (std::exception& e)
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() YamlParser error: " << e.what() << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Error);
    return false;
  }

  std::string port;
  int         baudrate;
  // Ublox connection settings
  if (!yamlParserPtr_->readVariable<std::string>(port, "port") ||
      !yamlParserPtr_->readVariable<int>(baudrate, "baudrate"))
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() " << yamlFilename
            << " no port or baudrate provided.f" << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Error);
    return false;
  }

  ublox2_.setPort(port);
  ublox2_.setBaudrate(baudrate);

  ublox2_.setMonitorConnectionTimeout(
    yamlParserPtr_->readVariable<unsigned int>("connection_timeout", 10));
  ublox2_.setMonitorConnectionPeriod(
    yamlParserPtr_->readVariable<unsigned int>("connect_retry_period", 1));

  ublox2_.setAgpsPollingSettings(
    yamlParserPtr_->readVariable<bool>("poll_for_pvt", false),
    yamlParserPtr_->readVariable<bool>("poll_for_ephemeris", false),
    yamlParserPtr_->readVariable<bool>("poll_for_almanac", false),
    yamlParserPtr_->readVariable<bool>("poll_for_utcIono", false),
    yamlParserPtr_->readVariable<bool>("poll_for_raw", false));

  ublox2_.setProtocolSettings(
    yamlParserPtr_->readVariable<bool>("ublox_input", true),
    yamlParserPtr_->readVariable<bool>("ublox_output", true),
    yamlParserPtr_->readVariable<bool>("nmea_input", false),
    yamlParserPtr_->readVariable<bool>("nmea_output", false));

  ublox2_.setDynamicModel(
    yamlParserPtr_->readVariable<std::string>("dynamic_model", "automotive"));
  ublox2_.setFixMode(
    yamlParserPtr_->readVariable<std::string>("fix_mode", "auto"));

  ublox2_.msgClasses_ =
    yamlParserPtr_->readVariable<std::vector<std::string>>("messages_class");
  ublox2_.msgIds_ =
    yamlParserPtr_->readVariable<std::vector<std::string>>("messages_id");

  std::vector<int> tempV =
    yamlParserPtr_->readVariable<std::vector<int>>("messages_rate");
  std::vector<uint8_t> tempV2(tempV.begin(), tempV.end());
  ublox2_.msgRates_ = tempV2;

  return true;
}

//==============================================================//
//---------------------- bindUblox2Callbacks() -----------------//
//==============================================================//
void ToolkitApplication::bindUblox2Callbacks()
{
  //   set up logging handlers
  ublox2_.setLogCallback(std::bind(&ToolkitApplication::handleLog,
                                   this,
                                   std::placeholders::_1,
                                   std::placeholders::_2));

  ublox2_.set_nav_pvt_Handler(std::bind(&ToolkitApplication::handlePvtUblox2,
                                        this,
                                        std::placeholders::_1,
                                        std::placeholders::_2));

  ublox2_.set_rxm_rawx_Handler(
    std::bind(&ToolkitApplication::handleRawxMeasUblox2,
              this,
              std::placeholders::_1,
              std::placeholders::_2));

  ublox2_.set_nav_clock_Handler(
    std::bind(&ToolkitApplication::handleNavClockUblox2,
              this,
              std::placeholders::_1,
              std::placeholders::_2));

  ublox2_.set_nav_gps_time_Handler(
    std::bind(&ToolkitApplication::handleNavGpsTimeUblox2,
              this,
              std::placeholders::_1,
              std::placeholders::_2));

  ublox2_.set_rxm_subframe_X_Handler(
    std::bind(&ToolkitApplication::handleRxmSubframeXUblox2,
              this,
              std::placeholders::_1,
              std::placeholders::_2));              
}
//==============================================================//
//-------------- handlePvtUblox1() -----------------//
//==============================================================//
void ToolkitApplication::handlePvtUblox1(
  const ublox::NavPvt&          pvt,
  const ublox::SysClkTimePoint& timeStamp)
{
  // convert to PosVel struct
  pnt_integrity::data::PositionVelocity posVel;
  buildPositionVelocity(pvt, timeStamp, posVel);

  // update GUI with Position
  mainWindow_.sendRcvrPV(posVel);

  // call handlePV in IntegrityMonitor
  posVel.header.deviceId = rx1Name_;
  posVel.header.seq_num = pvtUblox1SeqNum_++;
  integrityMonitor_.handlePositionVelocity(posVel, true);

  integrityMonitor_.determineAssuranceLevels();
  publishAssurance(posVel);

  if (pushMode_)
    publishPvtSerial(pvt);
}

void ToolkitApplication::updateLcd(const std::future<void>& futureObj)
{
  while ((futureObj.wait_for(std::chrono::milliseconds(1000)) ==
          std::future_status::timeout))
  {
    pnt_integrity::data::AssuranceReport report;

    // Retrieve Assurance Level using ToolkitLCDDisplay
    pnt_integrity::data::AssuranceLevel level =
      integrityMonitor_.getAssuranceLevel();

    lcdDisplay_.SetAssuranceLevel(level);  // returns level_

    pnt_integrity::data::AssuranceReports test_report =
      integrityMonitor_.getAssuranceReports();

    // Creating a Timestamp for Arrival Time (valid_time) in the csv logging
    // file for MATLAB use
    auto current_time = std::chrono::system_clock::now();
    auto duration_in_seconds =
      std::chrono::duration<double>(current_time.time_since_epoch());

    double timestamp = duration_in_seconds.count();

    test_report.header.timestampValid.sec = std::floor(timestamp);

    test_report.header.timestampValid.nanoseconds =
      (timestamp - test_report.header.timestampValid.sec) * 1e9;

    test_report.header.timestampArrival = test_report.header.timestampValid;
    report.header                       = test_report.header;

    report.header.seq_num = seqNum_;
    report.state.setWithLevel(level);
    csvAssuranceReportWriter_.logOutput(report);

    // Logging begins for Assurance Reports
    if (test_report.numStates > 0)
    {
      lcdDisplay_.displayCheckState(test_report.states[checkCount_]);
      checkCount_++;
      if (checkCount_ >= test_report.numStates)
        checkCount_ = 0;
    }
    test_report.header.seq_num = seqNum_;
    csvAssuranceReportsWriter_.logOutput(test_report);

    seqNum_++;
  }
}

void ToolkitApplication::publishPvtSerial(const ublox::NavPvt& pvt)
{
  if (serialPtr_ != nullptr)
  {
    if (serialPtr_->isOpen())
    {
      std::stringstream pvtMsg;
      pvtMsg << "$GPGGA,";

      //-----------time: 'HHMMSS.SS'-------------//
      // format the hour into a 'hh' format
      std::stringstream hourStr;
      hourStr << std::setw(2) << std::setfill('0') << (int)pvt.hour;

      // format the mins into a 'MM' format
      std::stringstream minStr;
      minStr << std::setw(2) << std::setfill('0') << (int)pvt.min;

      // format the seconds into a'SS' format
      // first compute the full seconds (sometimes nano is a negative value)
      double            fullSec = (double)pvt.sec + (double)pvt.nano / 1e9;
      std::stringstream secStr;
      secStr << std::setw(2) << std::setfill('0') << (int)fullSec;

      // format the remianing fractional seconds
      double            fracSec = fullSec - (int)fullSec;
      std::stringstream nanoStr;
      // multiply the fractional seconds by 100 to get on the left side of
      // decimal for zero fill i.e. fracSec = 0.abcdefg becomes ab.cdefg and
      // then 'ab' is taken for the fracional amount for the printed string
      nanoStr << std::setw(2) << std::setfill('0') << (int)(fracSec * 100);

      // add the 'HHMMSS.SS' string to the message
      pvtMsg << hourStr.str() << minStr.str() << secStr.str() << "."
             << nanoStr.str();

      //---------latitude in DDMM.MMMMM------------
      // compute the full latitude by removing the scale
      double fullLat = (double)pvt.latitude_scaled * 1e-7;

      // determine if this is north or south by sign
      std::string northSouthStr = "N";
      if (fullLat < 0)
        northSouthStr = "S";

      // assign the degrees value for the latitude
      uint32_t latDeg = (uint32_t)(std::abs(fullLat));

      // compute the remining fractional degrees and convert to minutes
      double fracLat = std::abs(fullLat) - latDeg;
      double latMin  = fracLat * 60;

      // dump the degrees to a string
      std::stringstream latDegStr;
      latDegStr << std::setw(2) << std::setfill('0') << latDeg;

      // dump the minutes to a string
      std::stringstream latMinStr;
      latMinStr << std::fixed << std::setprecision(4) << std::setw(7)
                << std::setfill('0') << latMin;

      pvtMsg << "," << latDegStr.str() << latMinStr.str() << ","
             << northSouthStr;

      //---------longitude in DDMM.MMMMM------------
      // compute the full longitude by removing the scale
      double fullLon = (double)pvt.longitude_scaled * 1e-7;

      // determine if this is north or south by sign
      std::string eastWestStr = "E";
      if (fullLon < 0)
        eastWestStr = "W";

      // assign the degrees value for the longitude
      uint32_t lonDeg = (uint32_t)(std::abs(fullLon));

      // compute the remining fractional degrees and convert to minutes
      double fracLon = std::abs(fullLon) - lonDeg;
      double lonMin  = fracLon * 60;

      // dump the degrees to a string
      std::stringstream lonDegStr;
      lonDegStr << std::setw(2) << std::setfill('0') << lonDeg;

      // dump the minutes to a string
      std::stringstream lonMinStr;
      lonMinStr << std::fixed << std::setprecision(4) << std::setw(7)
                << std::setfill('0') << lonMin;

      pvtMsg << "," << lonDegStr.str() << lonMinStr.str() << "," << eastWestStr;

      //------------fix---------------
      pvtMsg << "," << (int)pvt.gpsFix;

      //------------num sats (XX)----------
      std::stringstream numSatsStr;
      numSatsStr << std::setw(2) << std::setfill('0') << (int)pvt.numSV;
      pvtMsg << "," << numSatsStr.str();

      //--------------HDOP-----------------
      pvtMsg << ",";  // blank

      //--------------Altitude (msl)-----------------
      std::stringstream altStr;
      altStr << std::setprecision(5)
             << (double)pvt.height_mean_sea_level / 1000;
      pvtMsg << "," << altStr.str() << ",M";

      //---------------Height (geoid)----------
      std::stringstream heightStr;
      heightStr << std::setprecision(5) << (double)pvt.height / 1000;
      pvtMsg << "," << heightStr.str() << ",M";

      //---------DGPS--------
      pvtMsg << ",";  // blank

      //----------------------------------------------------
      uint8_t checksum = computeNmeaChecksum(pvtMsg.str().c_str());

      pvtMsg << checksum;

      serialPtr_->write(pvtMsg.str());
    }
  }
}

//==============================================================//
//-------------- handlePvtUblox2() -----------------//
//==============================================================//
void ToolkitApplication::handlePvtUblox2(
  const ublox::NavPvt&          pvt,
  const ublox::SysClkTimePoint& timeStamp)
{
  // convert to PosVel struct
  pnt_integrity::data::PositionVelocity posVel;
  buildPositionVelocity(pvt, timeStamp, posVel);

  // TODO: send to IM

  // update GUI with Position
  mainWindow_.sendRcvr2PV(posVel);

  posVel.header.deviceId = rx2Name_;
  posVel.header.seq_num = pvtUblox2SeqNum_++;
  integrityMonitor_.handlePositionVelocity(posVel, false);
  if (useStaticMode_)
  {
    pnt_integrity::data::MeasuredRange range;
    range.header = posVel.header;

    range.rangeValid      = true;
    range.range           = rx1rx2Baseline_;
    range.variance        = 1e-3;
    range.featurePosition = posVel.position;
    for (auto ii = 0; ii < 3; ++ii)
    {
      for (auto jj = 0; jj < 3; ++jj)
      {
        range.feature_position_covariance_[ii][jj] = posVel.covariance[ii][jj];
      }
    }
    // call IM handle function
    integrityMonitor_.handleMeasuredRange(range, false);
  }

  integrityMonitor_.determineAssuranceLevels();
}

// ==============================================================//
// -------------- handleRawxMeasUblox1() -----------------//
// ==============================================================//
void ToolkitApplication::handleRawxMeasUblox1(
  const ublox::RawMeasX&        rawMeasX,
  const ublox::SysClkTimePoint& timeStamp)
{
  pnt_integrity::data::GNSSObservables gnss;
  buildGnssObservables(rawMeasX, timeStamp, gnss);
  gnss.header.deviceId = rx1Name_;
  gnss.header.seq_num = rawMeasXUblox1SeqNum_++;
  integrityMonitor_.handleGnssObservables(gnss, true);
  mainWindow_.sendRcvrGNSS(gnss);
}

// ==============================================================//
// -------------- handleRawxMeasUblox2() -----------------//
// ==============================================================//
void ToolkitApplication::handleRawxMeasUblox2(
  const ublox::RawMeasX&        rawMeasX,
  const ublox::SysClkTimePoint& timeStamp)
{
  pnt_integrity::data::GNSSObservables gnss;
  buildGnssObservables(rawMeasX, timeStamp, gnss);
  gnss.header.deviceId = rx2Name_;
  gnss.header.seq_num = rawMeasXUblox2SeqNum_++;
  integrityMonitor_.handleGnssObservables(gnss, false);
  mainWindow_.sendRcvr2GNSS(gnss);
}

// ==============================================================//
// -------------- handleRxmSubframeXUblox1() -----------------//
// ==============================================================//
void ToolkitApplication::handleRxmSubframeXUblox1(const ublox::SubframeDataX&       rxmSubframeX,
                                              const ublox::SysClkTimePoint& timeStamp)
{
  pnt_integrity::data::GNSSSubframe gnssSubframe;
  buildGnssSubframe(rxmSubframeX, timeStamp, gnssSubframe);
  gnssSubframe.header.deviceId = rx1Name_;
  gnssSubframe.header.seq_num = rxmSubframeUblox1SeqNum_++;
  integrityMonitor_.handleGnssSubframe(gnssSubframe, true);
  // mainWindow_.sendRcvrGNSS(gnss);
}

// ==============================================================//
// -------------- handleRawxMeasUblox2() -----------------//
// ==============================================================//
void ToolkitApplication::handleRxmSubframeXUblox2(const ublox::SubframeDataX&       rxmSubframeX,
                                              const ublox::SysClkTimePoint& timeStamp)
{
  pnt_integrity::data::GNSSSubframe gnssSubframe;
  buildGnssSubframe(rxmSubframeX, timeStamp, gnssSubframe);
  gnssSubframe.header.deviceId = rx2Name_;
  gnssSubframe.header.seq_num = rxmSubframeUblox2SeqNum_++;
  integrityMonitor_.handleGnssSubframe(gnssSubframe, false);
  // mainWindow_.sendRcvr2GNSS(gnss);
}

// ==============================================================//
// -------------- handleNavClockUblox1() -----------------//
// ==============================================================//
void ToolkitApplication::handleNavClockUblox1(
  const ublox::NavClock&        clock,
  const ublox::SysClkTimePoint& timeStamp)
{
  std::stringstream logStr;
  logStr << __FUNCTION__ << "() msg received";
  handleLog(logStr.str(), logutils::LogLevel::Debug2);

  curNavClockUblox1_.first  = timeStamp;
  curNavClockUblox1_.second = clock;
  checkForCompleteMsgsUblox1();
}

// ==============================================================//
// -------------- handleNavClockUblox2() -----------------//
// ==============================================================//
void ToolkitApplication::handleNavClockUblox2(
  const ublox::NavClock&        clock,
  const ublox::SysClkTimePoint& timeStamp)
{
  std::stringstream logStr;
  logStr << __FUNCTION__ << "() msg received";
  handleLog(logStr.str(), logutils::LogLevel::Debug2);

  curNavClockUblox2_.first  = timeStamp;
  curNavClockUblox2_.second = clock;
  checkForCompleteMsgsUblox2();
}

//==============================================================//
//-------------- handleNavGpsTimeUblox1() -----------------//
//==============================================================//
void ToolkitApplication::handleNavGpsTimeUblox1(
  const ublox::NavGPSTime&      gpsTime,
  const ublox::SysClkTimePoint& timeStamp)
{
  std::stringstream logStr;
  logStr << __FUNCTION__ << "() msg received";
  handleLog(logStr.str(), logutils::LogLevel::Debug2);

  curNavGpsTimeUblox1_.first  = timeStamp;
  curNavGpsTimeUblox1_.second = gpsTime;

  checkForCompleteMsgsUblox1();
}

//==============================================================//
//-------------- handleNavGpsTimeUblox2() -----------------//
//==============================================================//
void ToolkitApplication::handleNavGpsTimeUblox2(
  const ublox::NavGPSTime&      gpsTime,
  const ublox::SysClkTimePoint& timeStamp)
{
  std::stringstream logStr;
  logStr << __FUNCTION__ << "() msg received";
  handleLog(logStr.str(), logutils::LogLevel::Debug2);

  curNavGpsTimeUblox2_.first  = timeStamp;
  curNavGpsTimeUblox2_.second = gpsTime;

  checkForCompleteMsgsUblox2();
}

//==============================================================//
//------------------- handleMonRfUblox1() ----------------------//
//==============================================================//
void ToolkitApplication::handleMonRfUblox1(
  const ublox::MonRf&           monRf,
  const ublox::SysClkTimePoint& timeStamp)
{
  std::stringstream logStr;
  logStr << __FUNCTION__ << "() msg received";
  handleLog(logStr.str(), logutils::LogLevel::Debug2);

  pnt_integrity::data::AgcValue agc;
  if (useAgcCheck_)
  {
    agc.header.deviceId = "integrity";
    agc.header.seq_num  = monRfUblox1SeqNum_++;

    uint64_t nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             timeStamp.time_since_epoch())
                             .count();

    nanoseconds = nanoseconds % (uint64_t)1e9;

    uint64_t seconds = std::chrono::duration_cast<std::chrono::seconds>(
                         timeStamp.time_since_epoch())
                         .count();

    agc.header.timestampArrival.nanoseconds = nanoseconds;
    agc.header.timestampArrival.sec         = seconds;
    agc.header.timestampArrival.timecode    = 0;

    agc.header.timestampValid = agc.header.timestampArrival;

    for (size_t ii = 0; ii < monRf.numBlocks; ++ii)
    {
      // Band 0 is L1, Band 1 is L2
      agc.agcValues[(pnt_integrity::data::FrequencyBand)ii] =
        monRf.rfBlocks[ii].agcCnt;
    }
    integrityMonitor_.handleAGC(agc);
  }
}
//==============================================================//
//-------------- checkForCompleteMsgsUblox1() -----------------//
//==============================================================//
void ToolkitApplication::checkForCompleteMsgsUblox1()
{
  // check member variables for completeness
  std::stringstream logStr;
  logStr << __FUNCTION__ << "() msg received";
  handleLog(logStr.str(), logutils::LogLevel::Debug2);

  if ((curNavClockUblox1_.second.iTOW != ublox::NavClock().iTOW) &&
      (curNavClockUblox1_.second.iTOW == curNavGpsTimeUblox1_.second.iTOW))
  {
    pnt_integrity::data::ClockOffset clockOffset;
    buildClockOffset(curNavGpsTimeUblox1_.second,
                     curNavClockUblox1_.second,
                     curNavGpsTimeUblox1_.first,
                     clockOffset);

    integrityMonitor_.handleClockOffset(clockOffset, true);

    // send time to UI
    double towSec = (curNavGpsTimeUblox1_.second.iTOW / 1e3) +
                    (curNavGpsTimeUblox1_.second.ftow / 1e9);
    mainWindow_.sendRcvrGpsTime(curNavGpsTimeUblox1_.second.week, towSec);
  }
}
//==============================================================//
//-------------- checkForCompleteMsgsUblox2() -----------------//
//==============================================================//
void ToolkitApplication::checkForCompleteMsgsUblox2()
{
  // check member variables for completeness
  std::stringstream logStr;
  logStr << __FUNCTION__ << "() msg received";
  handleLog(logStr.str(), logutils::LogLevel::Debug2);

  if ((curNavClockUblox2_.second.iTOW != ublox::NavClock().iTOW) &&
      (curNavClockUblox2_.second.iTOW == curNavGpsTimeUblox2_.second.iTOW))
  {
    // send time to UI
    double towSec = (curNavGpsTimeUblox2_.second.iTOW / 1e3) +
                    (curNavGpsTimeUblox2_.second.ftow / 1e9);
    mainWindow_.sendRcvr2GpsTime(curNavGpsTimeUblox2_.second.week, towSec);
  }
}

//==============================================================//
//-------------- buildClockOffset() -----------------//
//==============================================================//
void ToolkitApplication::buildClockOffset(
  const ublox::NavGPSTime&          gpsTime,
  const ublox::NavClock&            clock,
  const ublox::SysClkTimePoint&     timeStamp,
  pnt_integrity::data::ClockOffset& clockOffset)
{
  clockOffset.header.deviceId = "integrity";
  clockOffset.header.seq_num  = 0;

  uint64_t nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
                           timeStamp.time_since_epoch())
                           .count();

  nanoseconds = nanoseconds % (uint64_t)1e9;

  uint64_t seconds = std::chrono::duration_cast<std::chrono::seconds>(
                       timeStamp.time_since_epoch())
                       .count();

  clockOffset.header.timestampArrival.sec         = seconds;
  clockOffset.header.timestampArrival.nanoseconds = nanoseconds;
  clockOffset.header.timestampValid = clockOffset.header.timestampArrival;

  clockOffset.timecode1 = 0;
  clockOffset.timecode2 = 0;

  // Exact Rcvr Set Time
  double sec = static_cast<double>(gpsTime.iTOW) * 1e-3 +
               static_cast<double>(gpsTime.ftow) * 1e-9;

  // Truncated Whole Seconds of Rcvr Set Time
  int32_t wholeSec     = static_cast<int32_t>(sec);
  double  leftoverNsec = (sec - static_cast<double>(wholeSec)) * 1e9;

  clockOffset.offset = static_cast<double>(clock.clkbias) * -1e-9;
  clockOffset.drift  = static_cast<double>(clock.clkdrift) * -1e-9;

  clockOffset.covariance[0][0] =
    std::pow(static_cast<double>(clock.tacc) * 1e-9, 2);  // nsec to sec
  clockOffset.covariance[1][1] =
    std::pow(static_cast<double>(clock.facc) * 1e-12, 2);  // picosec to sec
  clockOffset.covariance[0][1] = 0.0;
  clockOffset.covariance[1][0] = 0.0;
}

//==============================================================//
//-------------- buildHeader() -----------------//
//==============================================================//

void ToolkitApplication::buildHeader(const ublox::UbloxHeader&     ubxHeader,
                                     const ublox::SysClkTimePoint& timeStamp,
                                     pnt_integrity::data::Header&  header)
{
  header.deviceId = "ublox";
  header.seq_num  = 0;
  header.timestampArrival.nanoseconds =
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      timeStamp.time_since_epoch())
      .count() %
    (uint64_t)1e9;

  header.timestampArrival.sec =
    std::chrono::duration_cast<std::chrono::seconds>(
      timeStamp.time_since_epoch())
      .count();

  header.timestampValid = header.timestampArrival;
}

//==============================================================//
//-------------- buildPositionVelocity() -----------------//
//==============================================================//
void ToolkitApplication::buildPositionVelocity(
  const ublox::NavPvt&                   pvt,
  const ublox::SysClkTimePoint&          timeStamp,
  pnt_integrity::data::PositionVelocity& posVel)
{
  buildHeader(pvt.header, timeStamp, posVel.header);

  posVel.position.altitude = pvt.height * 1e-3;
  posVel.position.latitude = pvt.latitude_scaled * (double)(1e-7) * deg2rad;

  posVel.position.longitude = pvt.longitude_scaled * (double)(1e-7) * deg2rad;

  posVel.velocity[0] = pvt.velocity_north * 1e-3;  // North, mm/sec to m/sec
  posVel.velocity[1] = pvt.velocity_east * 1e-3;   // East, mm/sec to m/sec
  posVel.velocity[2] = pvt.velocity_down * 1e-3;   // Down, mm/sec to m/sec

  // Position Covariance
  for (auto ii = 0; ii < 6; ++ii)
  {
    for (auto jj = 0; jj < 6; ++jj)
    {
      posVel.covariance[ii][jj] = 0.0;
    }
  }

  posVel.covariance[0][0] =
    std::pow(pvt.horizontal_accuracy * 1e-3, 2);  // North
  posVel.covariance[1][1] =
    std::pow(pvt.horizontal_accuracy * 1e-3, 2);                        // East
  posVel.covariance[2][2] = std::pow(pvt.vertical_accuracy * 1e-3, 2);  // Down

  // Velocity Covariance
  posVel.covariance[3][3] = std::pow(pvt.velocity_north * 1e-3, 2);  // North
  posVel.covariance[4][4] = std::pow(pvt.velocity_east * 1e-3, 2);   // East
  posVel.covariance[5][5] = std::pow(pvt.velocity_down * 1e-3, 2);   // Down
}

//==============================================================//
//------------------- buildGnssObservables() -------------------//
//==============================================================//
void ToolkitApplication::buildGnssObservables(
  const ublox::RawMeasX&                rawMeasX,
  const ublox::SysClkTimePoint&         timeStamp,
  pnt_integrity::data::GNSSObservables& gnss)
{
  buildHeader(rawMeasX.header, timeStamp, gnss.header);

  gnss.gnssTime.secondsOfWeek = rawMeasX.week;
  gnss.gnssTime.weekNumber    = rawMeasX.rcvTow;
  gnss.gnssTime.timeSystem    = pnt_integrity::data::TimeSystem::GPS;

  for (size_t i = 0; i < rawMeasX.numMeas; ++i)
  {
    // If Carrier phase locked for more than 1 sec then haven't lost lock
    bool lossOfLock = true;
    if (rawMeasX.rawxmeasreap[i].locktime > 1000)
    {
      lossOfLock = false;
    }

    // create a new pnt_integrity::data::GNSSObservable to put in the map
    pnt_integrity::data::GNSSObservable newObs(
      getSvID(rawMeasX.rawxmeasreap[i].svid),
      getSatSystem(rawMeasX.rawxmeasreap[i].gnssId),
      getCodeType(rawMeasX.rawxmeasreap[i].gnssId,
                  rawMeasX.rawxmeasreap[i].sigId),
      getBand(rawMeasX.rawxmeasreap[i].gnssId, rawMeasX.rawxmeasreap[i].sigId),
      pnt_integrity::data::AssuranceLevel::Unavailable,
      rawMeasX.rawxmeasreap[i].cno,
      true,
      rawMeasX.rawxmeasreap[i].psuedorange,
      pow(rawMeasX.rawxmeasreap[i].psrStdDev, 2.0),
      true,
      rawMeasX.rawxmeasreap[i].doppler,
      pow(rawMeasX.rawxmeasreap[i].doStdDev, 2.0),
      true,
      rawMeasX.rawxmeasreap[i].carrier_phase,
      pow(rawMeasX.rawxmeasreap[i].cpStdDeve, 2.0),
      lossOfLock);

    // Enter the new observable into the map with the unique id from
    // the new observable
    gnss.observables.insert(std::make_pair(newObs.getUniqueID(), newObs));
  }
}

//==============================================================//
//------------------- buildGnssSubframe() -------------------//
//==============================================================//
void ToolkitApplication::buildGnssSubframe(const ublox::SubframeDataX& rxmSubframeX,
                          const ublox::SysClkTimePoint& timeStamp,
                          pnt_integrity::data::GNSSSubframe& gnssSubframe)
{

  // Populate gnssSubframe.header
  // Convert timeStamp
  uint64_t nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
    timeStamp.time_since_epoch()).count();
  nanoseconds = nanoseconds % (uint64_t)1e9;
  uint64_t seconds = std::chrono::duration_cast<std::chrono::seconds>(
                       timeStamp.time_since_epoch()).count();

  gnssSubframe.header.timestampArrival.nanoseconds = nanoseconds;
  gnssSubframe.header.timestampArrival.sec      = seconds;
  gnssSubframe.header.timestampArrival.timecode = 100;
  gnssSubframe.header.timestampValid = gnssSubframe.header.timestampArrival;
    
  gnssSubframe.prn = getSvID(rxmSubframeX.svID);
  gnssSubframe.satelliteType = getSatSystem((ublox::GnssId)rxmSubframeX.gnssId);
  
  for (uint8_t ii=0; ii<rxmSubframeX.numWords; ii++)
  {
    // remove parity (last 6 bits LSBs)
    uint32_t wordNoParity;
    wordNoParity = ((rxmSubframeX.dwrd[ii]>>6) & 0xFFFFFF);
    // only 24 bits of data are valid after subframe is removed 
    // (30 bit data - 6 bit subframe).  Store in byte array
    gnssSubframe.subframeData.push_back((wordNoParity >> 16) & 0xff);
    gnssSubframe.subframeData.push_back((wordNoParity >> 8) & 0xff);
    gnssSubframe.subframeData.push_back((wordNoParity & 0xff));
  }
}

//===============================================================
//------------------------- getCodeType ------------------------
//===============================================================
pnt_integrity::data::CodeType ToolkitApplication::getCodeType(
  const ublox::GnssId& gnssIde,
  const uint8_t&       sigId)
{
  uint8_t gnssId = static_cast<uint8_t>(gnssIde);

  if (gnssId == 0)  // GPS
  {
    if (sigId == 0)  // C/A
      return pnt_integrity::data::CodeType::SigC;
    if (sigId == 3)  // L2 CL
      return pnt_integrity::data::CodeType::SigD;
    if (sigId == 4)  // L2 CM
      return pnt_integrity::data::CodeType::SigD;
  }
  else if (gnssId == 2)  // Galileo
  {
    if (sigId == 0)  // E1 C
      return pnt_integrity::data::CodeType::SigC;
    if (sigId == 1)  // E1 B
      return pnt_integrity::data::CodeType::SigB;
    if (sigId == 5)  // E5 bI
      return pnt_integrity::data::CodeType::SigI;
    if (sigId == 6)  // E5 bQ
      return pnt_integrity::data::CodeType::SigQ;
  }
  else if (gnssId == 3)  // BeiDou
  {
    if (sigId == 0)  // B1l D1
      return pnt_integrity::data::CodeType::SigX;
    if (sigId == 1)  // B1l D2
      return pnt_integrity::data::CodeType::SigX;
    if (sigId == 2)  // B2l D1
      return pnt_integrity::data::CodeType::SigX;
    if (sigId == 3)  // B2l D2
      return pnt_integrity::data::CodeType::SigX;
  }
  else if (gnssId == 5)  // QZSS
  {
    if (sigId == 0)  // L1C/A
      return pnt_integrity::data::CodeType::SigC;
    if (sigId == 4)  // L2 CM
      return pnt_integrity::data::CodeType::SigS;
    if (sigId == 5)  // L2 CL
      return pnt_integrity::data::CodeType::SigL;
  }
  else if (gnssId == 6)  // GLONASS
  {
    if (sigId == 0)  // L1 OF
      return pnt_integrity::data::CodeType::SigC;
    if (sigId == 2)  // L2 OF
      return pnt_integrity::data::CodeType::SigC;
  }

  return pnt_integrity::data::CodeType::SigBLANK;
}

//===============================================================
//------------------------- getSatSystem ------------------------
//===============================================================
/// \brief  Convert UBX GNSS ID to pnt_integrity::data::SatelliteSystem
pnt_integrity::data::SatelliteSystem ToolkitApplication::getSatSystem(
  const ublox::GnssId& gnssIde)
{
  uint8_t gnssId = static_cast<uint8_t>(gnssIde);

  if (gnssId == 0)  // GPS
    return pnt_integrity::data::SatelliteSystem::GPS;
  else if (gnssId == 1)  // SBAS
    return pnt_integrity::data::SatelliteSystem::SBAS;
  else if (gnssId == 2)  // Galileo
    return pnt_integrity::data::SatelliteSystem::Galileo;
  else if (gnssId == 3)
    return pnt_integrity::data::SatelliteSystem::BeiDou;
  else if (gnssId == 5)  // QZSS
    return pnt_integrity::data::SatelliteSystem::QZSS;
  else if (gnssId == 6)  // GLONASS
    return pnt_integrity::data::SatelliteSystem::Glonass;
  else  // Reserved/Other
    return pnt_integrity::data::SatelliteSystem::Other;
}

//===============================================================
//------------------------- getSvID -----------------------------
//===============================================================
uint32_t ToolkitApplication::getSvID(const uint8_t& svid)
{
  if ((1 <= svid) && (svid <= 32))  // GPS
  {
    return svid;
  }
  else if ((120 <= svid) && (svid <= 158))  // SBAS
  {
    return svid;
  }
  else if ((211 <= svid) && (svid <= 246))  // Galileo
  {
    return svid - 210;
  }
  else if ((159 <= svid) && (svid <= 163))  // BeiDou 1-5
  {
    return svid - 158;
  }
  else if ((33 <= svid) && (svid <= 64))  // BeiDou 6-37
  {
    return svid - 32 + 5;
  }
  else if ((193 <= svid) && (svid <= 202))  // QZSS
  {
    return svid - 192;
  }
  else if ((65 <= svid) && (svid <= 96))  // GLONASS
  {
    return svid - 64;
  }
  else  // (svid == 255) // GLONASS, or nothing
  {
    return 0;
  }
}
//===============================================================
//------------------------- getBand------------------------------
//===============================================================
/// \brief  Convert UBX GNSSID & SIGID to pnt_integrity::data::FrequencyBand
pnt_integrity::data::FrequencyBand ToolkitApplication::getBand(
  const ublox::GnssId& gnssIde,
  const uint8_t&       sigId)
{
  uint8_t gnssId = static_cast<uint8_t>(gnssIde);

  if (sigId == 0)
  {
    if (gnssId == 0)  // GPS L1
      return pnt_integrity::data::FrequencyBand::Band1;
    else if (gnssId == 2)  // Galileo E1
      return pnt_integrity::data::FrequencyBand::Band1;
    else if (gnssId == 3)  // BeiDou B1l D1
      return pnt_integrity::data::FrequencyBand::Band2;
    else if (gnssId == 5)  // QZSS L1C/A
      return pnt_integrity::data::FrequencyBand::Band1;
    else if (gnssId == 6)  // GLONASS L1 OF
      return pnt_integrity::data::FrequencyBand::Band1;
  }
  else if (sigId == 1)
  {
    if (gnssId == 2)  // Galileo E1 B
      return pnt_integrity::data::FrequencyBand::Band1;
    else if (gnssId == 3)  // BeiDou B1l D2
      return pnt_integrity::data::FrequencyBand::Band2;
  }
  else if (sigId == 3)
  {
    if (gnssId == 0)  // GPS L2 CL
      return pnt_integrity::data::FrequencyBand::Band2;
    else if (gnssId == 3)  // BeiDou B2I D2
      return pnt_integrity::data::FrequencyBand::Band7;
  }
  else if (sigId == 4)
  {
    if (gnssId == 0)  // GPS L2 CM
      return pnt_integrity::data::FrequencyBand::Band2;
    else if (gnssId == 5)  // QZSS L2 CM
      return pnt_integrity::data::FrequencyBand::Band2;
  }
  else if (sigId == 5)
  {
    if (gnssId == 2)  // Galileo E5 bI
      return pnt_integrity::data::FrequencyBand::Band7;
    else if (gnssId == 5)  // QZSS L2 CL
      return pnt_integrity::data::FrequencyBand::Band2;
  }
  else if (sigId == 6)
  {
    if (gnssId == 2)  // Galileo E5 bQ
      return pnt_integrity::data::FrequencyBand::Band7;
  }
  return pnt_integrity::data::FrequencyBand::Band0;
}

//==============================================================================
//------------------------- configure-------------------------------
//==============================================================================
bool ToolkitApplication::configureIfDataStream(const std::string& yamlFilename)
{
  if (yamlFilename.empty())
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() Config file not provided" << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Error);
    return false;
  }

  try
  {
    yamlParserPtr_ = std::unique_ptr<yaml_parser::YamlParser>(
      new yaml_parser::YamlParser(yamlFilename,
                                  std::bind(&ToolkitApplication::handleLog,
                                            this,
                                            std::placeholders::_1,
                                            std::placeholders::_2)));
  }
  catch (std::exception& e)
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() YamlParser error: " << e.what() << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Error);
    return false;
  }

  //****************************************************************************
  // Setup IF data stream

  // create an IF sample header for convenience based on input parameters
  double samplingFreq;
  int    sampleType;
  if (!yamlParserPtr_->readVariable<double>(samplingFreq, "sampling_frequency"))
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() " << yamlFilename
            << " 'sampling_frequency' not provided." << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Error);
    return false;
  }
  if (!yamlParserPtr_->readVariable<int>(sampleType, "if_sample_type"))
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() " << yamlFilename
            << " 'if_sample_type' not provided." << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Error);
    return false;
  }
  if_data_utils::IFSampleType ifSampleType =
    static_cast<if_data_utils::IFSampleType>(sampleType);

  samplesPerMessage_ =
    yamlParserPtr_->readVariable<int>("samples_per_message", 10e3);

  // build a sample header to use for convenience
  if_data_utils::IFSampleHeader ifSampleHeader(
    samplesPerMessage_, ifSampleType, 0.0, samplingFreq);

  // spawn a thread around the liveStream function which
  // creates the recorder object in publishing mode,
  // connects the recorder object's publishing signal to the local
  // publishing function, and monitors the buffer status
  publishThreadPtr_ =
    std::make_shared<std::thread>(std::bind(&ToolkitApplication::liveStream,
                                            this,
                                            ifDataExitSignal_.get_future(),
                                            ifSampleHeader));
  //**********************************************//
  return true;
}

//==============================================================================
//-------------------------------
// liveStream------------------------------------
//==============================================================================
void ToolkitApplication::liveStream(const std::future<void>& futureObj,
                                    const if_data_utils::IFSampleHeader& header)
{
  double nanDefault = std::numeric_limits<double>::quiet_NaN();

  double freq, gain, bandwidth, totalTime;
  freq = yamlParserPtr_->readVariable<double>("center_frequency", 1575.42e6);
  gain = yamlParserPtr_->readVariable<double>("gain", 0.0);
  bandwidth = yamlParserPtr_->readVariable<double>("bandwidth", nanDefault);
  totalTime = yamlParserPtr_->readVariable<double>("total_time", 3600);

  std::string deviceArgs, antenna, subDev, clockRef, wireFmt;
  deviceArgs = yamlParserPtr_->readVariable<std::string>("device_args",
                                                         "addr=192.168.5.1");
  antenna    = yamlParserPtr_->readVariable<std::string>("antenna", "RX2");
  subDev     = yamlParserPtr_->readVariable<std::string>("subdev", "A:A");
  clockRef = yamlParserPtr_->readVariable<std::string>("clock_ref", "internal");
  wireFmt  = yamlParserPtr_->readVariable<std::string>("wirefmt", "sc8");

  bool qcal = yamlParserPtr_->readVariable<bool>("qcal", false);

  int cbCapacity =
    yamlParserPtr_->readVariable<int>("circ_buff_capacity", 100000);

  usrp_utilities::SurrpasUsrpArgs recordArgs(
    deviceArgs,
    usrp_utilities::UsrpDeviceMode::RECEIVE,
    totalTime,
    {freq},
    {header.fs_},
    {gain},
    {antenna},
    {bandwidth},
    {usrp_utilities::UsrpClockRef(clockRef)},
    {subDev},
    {"usrp_samples.dat"},
    usrp_utilities::UsrpSampleType(header.sampleType_, log_),
    usrp_utilities::UsrpWireFormat(wireFmt),
    cbCapacity,
    "0",     // start time
    false,   // provide bw summary
    1.0,     // setup time
    0,       // total samps
    qcal,    // qcal
    true,    // we always want publish mode true here
    false,   // checkLo
    false,   // intn tuning
    false,   // status
    false,   // size map
    false,   // null
    false);  // continue on back packet

  usrp_utilities::SurrpasUsrpRecord publisher(recordArgs, log_);

  // connect the publish function inside the publisher object to the local
  // publishing function
  publisher.setPublishSamples(
    std::bind(&ToolkitApplication::handleIfDataFromStream,
              this,
              std::placeholders::_1,
              std::placeholders::_2,
              std::placeholders::_3));

  // std::signal(SIGINT, &sigintHandler);
  handleLog("Press Ctrl + C to stop streaming...", logutils::LogLevel::Info);

  std::vector<size_t> bufferStatus;
  publisher.startRecordThread();

  while ((futureObj.wait_for(std::chrono::milliseconds(10)) ==
          std::future_status::timeout) and
         (publisher.checkRecordThreadStatus()))
  {
    if (qcal)
    {
      publisher.getBufferStatus(bufferStatus);
      std::stringstream qcalStr;
      for (size_t ii = 0; ii < recordArgs.numChannels_; ++ii)
      {
        qcalStr << "Channel [" << ii << "] :";
        qcalStr << " MaxBufferValue = " << publisher.getQcalVals(ii) << ", ";
        qcalStr << " Buffer size = " << bufferStatus[ii] << ", ";
      }
      handleLog(qcalStr.str(), logutils::LogLevel::Info);
    }
    else  // recording, not qcal-ing
    {
      publisher.getBufferStatus(bufferStatus);
      std::stringstream bufferMsg;
      bufferMsg << "Buffer Status: ";
      for (size_t ii = 0; ii < bufferStatus.size(); ++ii)
      {
        bufferMsg << "Channel " << ii << ": " << bufferStatus[ii] << ", ";
      }
      handleLog(bufferMsg.str(), logutils::LogLevel::Debug);
    }
  }
  publisher.killRecording();

  handleLog("Live IF Stream Terminated", logutils::LogLevel::Info);
}

//==============================================================================
//-------------------------
// publishDataFromStream-------------------------------
//==============================================================================
size_t ToolkitApplication::handleIfDataFromStream(
  uhd::transport::bounded_buffer<usrp_utilities::circbuff_element_t>& buffer,
  const usrp_utilities::SurrpasUsrpArgs&                              args,
  const size_t& sampsPerElement)
{
  // compute  how many buffer grabs are required to retrieve the desired
  // number of samples, then set the sample message size big enough to all
  // samples of each buffer grab
  size_t buffsToRead =
    (size_t)(ceil((double)samplesPerMessage_ / (double)sampsPerElement));
  size_t samplesInMessage = buffsToRead * sampsPerElement;

  // set timestamp and sequence number
  int64_t sec        = 0;
  int32_t nsec       = 0;
  double  sampleTime = getTime(sec, nsec);

  if_data_utils::IFSampleType sampleType = args.sampleType_.type_;
  switch (sampleType)
  {
    case if_data_utils::IFSampleType::SC8: {
      // Build a sample header based on provided data
      if_data_utils::IFSampleHeader header(
        samplesInMessage, sampleType, 0.0, args.rate_[0]);

      // Build the data structure, which allocates the memory for the samples
      if_data_utils::IFSampleData<if_data_utils::IFSampleSC8> sampleData(
        header);

      // define a pointer to the allocated data for direct read
      auto sampleDataBufferPtr =
        (usrp_utilities::circbuff_element_t*)sampleData.getBufferPtr();

      for (size_t nb = 0; nb < buffsToRead; ++nb)
      {
        // pop the data into the data structure
        buffer.pop_with_timed_wait(*sampleDataBufferPtr, 1.0);
        // incremement the buffer pointer for the next buffer read
        sampleDataBufferPtr++;
      }

      integrityMonitor_.handleIfSampleData(sampleTime, sampleData);

      break;
    }
    case if_data_utils::IFSampleType::SC16: {
      // Build a sample header based on provided data
      if_data_utils::IFSampleHeader header(
        samplesInMessage, sampleType, 0.0, args.rate_[0]);

      // Build the data structure, which allocates the memory for the samples
      if_data_utils::IFSampleData<if_data_utils::IFSampleSC16> sampleData(
        header);

      // define a pointer to the allocated data for direct read
      auto sampleDataBufferPtr =
        (usrp_utilities::circbuff_element_t*)sampleData.getBufferPtr();

      for (size_t nb = 0; nb < buffsToRead; ++nb)
      {
        // pop the data into the data structure
        buffer.pop_with_timed_wait(*sampleDataBufferPtr, 1.0);
        // incremement the buffer pointer for the next buffer read
        sampleDataBufferPtr++;
      }
      integrityMonitor_.handleIfSampleData(sampleTime, sampleData);
      break;
    }
    default: {
      handleLog("Sample type not yet supported", logutils::LogLevel::Error);
    }
  }

  return buffsToRead;
}

bool ToolkitApplication::configureIntegrity(const std::string& yamlFilename)
{
  std::stringstream log_str;
  log_str << __FUNCTION__ << "() Configure integrity" << std::endl;
  handleLog(log_str.str(), logutils::LogLevel::Info);

  //--------------------------------------------------------------------------
  if (yamlFilename.empty())
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() Integrity Monitor yaml file not provided"
            << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Error);
    return false;
  }

  try
  {
    yamlParserPtr_ = std::unique_ptr<yaml_parser::YamlParser>(
      new yaml_parser::YamlParser(yamlFilename,
                                  std::bind(&ToolkitApplication::handleLog,
                                            this,
                                            std::placeholders::_1,
                                            std::placeholders::_2)));
  }
  catch (std::exception& e)
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() YamlParser error: " << e.what() << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Error);
    return false;
  }

  rx1Name_ = yamlParserPtr_->readVariable<std::string>("rx1_name");
  rx2Name_ = yamlParserPtr_->readVariable<std::string>("rx2_name");

  integrityMonitor_.setLogMessageHandler(
    std::bind(&ToolkitApplication::handleLog,
              this,
              std::placeholders::_1,
              std::placeholders::_2));

  // TODO: should these return a bool indicating success?
  initializeRangePosCheck();
  initializeStaticPosCheck();
  initializeCnoCheck();
  initializePosJumpCheck();
  initializePosVelConsCheck();
  initializeAoaCheck();
  initializeAgcCheck();
  initializeClockBiasCheck();
  initializeNavDataCheck();
#ifdef PNT_INTEGRITY_INCLUDES_ACQ_CHECK
  initializeAcquisitionCheck();
#endif
  return true;
}
//==================================================================
//-----------------Range position check functions-------------------
//==================================================================
void ToolkitApplication::initializeRangePosCheck()
{
  useRngPosCheck_ =
    yamlParserPtr_->readVariable<bool>("enable_rng_pos_check", false);

  if (useRngPosCheck_)
  {
    rngPosCheckName_ = yamlParserPtr_->readVariable<std::string>(
      "rng_pos_check_name", "rng_pos_check");

    rngPosCheckPtr_ = std::make_shared<pnt_integrity::RangePositionCheck>(
      rngPosCheckName_, log_);

    rngPosCheckPtr_->setAssuranceLevelPeriod(
      yamlParserPtr_->readVariable<double>("rng_pos_check_level_period", 2.0));

    rngPosCheckPtr_->setWeight(
      yamlParserPtr_->readVariable<double>("rng_pos_check_weight", 1.0));

    rngPosCheckPtr_->setPublishDiagnostics(
      std::bind(&ToolkitApplication::publishRangePosCheckDiagnostics,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    integrityMonitor_.registerCheck(rngPosCheckName_, rngPosCheckPtr_.get());

    useStaticMode_ =
      yamlParserPtr_->readVariable<bool>("enable_static_mode", false);
    rx1rx2Baseline_ =
      yamlParserPtr_->readVariable<double>("rx1_rx2_baseline", 100e3);
  }
}
//-------------------------- publish diagnostics
//------------------------------
void ToolkitApplication::publishRangePosCheckDiagnostics(
  const double&                                time,
  const pnt_integrity::RngPosCheckDiagnostics& checkData)
{
  Diagnostics diagnostics;
  diagnostics.header.deviceId = "integrity";
  diagnostics.header.seq_num  = 0;  // doesn't matter for toolkit app

  diagnostics.header.timestampArrival.nanoseconds =
    (int)(time * 1e9) % 1000000000;
  diagnostics.header.timestampArrival.sec      = time;
  diagnostics.header.timestampArrival.timecode = 100;
  diagnostics.header.timestampValid = diagnostics.header.timestampArrival;

  diagnostics.level      = LevelEnum::DIAG_OK;
  diagnostics.name       = pnt_integrity::INTEGRITY_RNG_POS_DIAGNOSTICS;
  diagnostics.message    = "Rng-Pos check data";
  diagnostics.hardwareId = "integrity_monitor";
  diagnostics.numValues  = 0;

  auto nodeIt = checkData.begin();
  for (; nodeIt != checkData.end(); ++nodeIt)
  {
    KeyValue maxCalcKv;
    maxCalcKv.key =
      nodeIt->first + "_" + pnt_integrity::INTEGRITY_RNG_POS_DIAG_MAX_CALC;
    maxCalcKv.value = std::to_string(nodeIt->second.maxCalculatedRange);
    diagnostics.numValues++;
    diagnostics.values.push_back(maxCalcKv);

    KeyValue minCalcKv;
    minCalcKv.key =
      nodeIt->first + "_" + pnt_integrity::INTEGRITY_RNG_POS_DIAG_MIN_CALC;
    minCalcKv.value = std::to_string(nodeIt->second.minCalculatedRange);
    diagnostics.numValues++;
    diagnostics.values.push_back(minCalcKv);

    KeyValue maxMeasKv;
    maxMeasKv.key =
      nodeIt->first + "_" + pnt_integrity::INTEGRITY_RNG_POS_DIAG_MAX_MEAS;
    maxMeasKv.value = std::to_string(nodeIt->second.maxMeasRange);
    diagnostics.numValues++;
    diagnostics.values.push_back(maxMeasKv);

    KeyValue minMeasKv;
    minMeasKv.key =
      nodeIt->first + "_" + pnt_integrity::INTEGRITY_RNG_POS_DIAG_MIN_MEAS;
    minMeasKv.value = std::to_string(nodeIt->second.minMeasRange);
    diagnostics.numValues++;
    diagnostics.values.push_back(minMeasKv);
  }
  mainWindow_.sendDiagnostics(diagnostics);
}
//==============================================================================
//---------------------- Static-pos check
// functions-----------------------------
//==============================================================================
void ToolkitApplication::initializeStaticPosCheck()
{
  useStaticPosCheck_ =
    yamlParserPtr_->readVariable<bool>("enable_static_pos_check", false);

  if (useStaticPosCheck_)
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() Initialize static position check."
            << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Info);

    staticPosCheckName_ = yamlParserPtr_->readVariable<std::string>(
      "static_pos_check_name", "static_pos_check");

    int numPosForInit;
    numPosForInit = yamlParserPtr_->readVariable<int>("num_pos_for_init", 60);

    int checkWindowSize;
    checkWindowSize =
      yamlParserPtr_->readVariable<int>("static_pos_check_window_size", 10);

    double posChangeThresh;
    posChangeThresh =
      yamlParserPtr_->readVariable<double>("pos_change_thresh", 5.0);

    staticPosCheckPtr_ =
      std::make_shared<pnt_integrity::StaticPositionCheck>(staticPosCheckName_,
                                                           numPosForInit,
                                                           checkWindowSize,
                                                           posChangeThresh,
                                                           log_);

    staticPosCheckPtr_->setPublishDiagnostics(
      std::bind(&ToolkitApplication::publishStaticPosCheckDiagnostics,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    staticPosCheckPtr_->setAssuranceLevelPeriod(
      yamlParserPtr_->readVariable<double>("static_pos_check_level_period",
                                           60.0));

    staticPosCheckPtr_->setWeight(
      yamlParserPtr_->readVariable<double>("static_pos_check_weight", 1.0));

    integrityMonitor_.registerCheck(staticPosCheckName_,
                                    staticPosCheckPtr_.get());

    if (!yamlParserPtr_->readVariable<bool>("survey_init", true))
    {
      double staticLatDeg;
      staticLatDeg =
        yamlParserPtr_->readVariable<double>("static_pos_lat_deg", 5.0);

      double staticLonDeg;
      staticLonDeg =
        yamlParserPtr_->readVariable<double>("static_pos_lon_deg", 5.0);

      double staticAltM;
      staticAltM =
        yamlParserPtr_->readVariable<double>("static_pos_alt_m", 5.0);

      double deg2rad = 3.1415926535898 / 180.0;

      double staticLatRad = staticLatDeg * deg2rad;
      double staticLonRad = staticLonDeg * deg2rad;

      staticPosCheckPtr_->setStaticPosition(
        pnt_integrity::data::GeodeticPosition3d(
          staticLatRad, staticLonRad, staticAltM));
    }

    double staticPosCheckInconsistentThresh;
    staticPosCheckInconsistentThresh = yamlParserPtr_->readVariable<double>(
      "static_pos_check_inconsistent_thresh", 0.3);

    double staticPosCheckUnassuredThresh;
    staticPosCheckUnassuredThresh = yamlParserPtr_->readVariable<double>(
      "static_pos_check_unassured_thresh", 0.7);

    staticPosCheckPtr_->setAssuranceThresholds(staticPosCheckInconsistentThresh,
                                               staticPosCheckUnassuredThresh);
  }
  else
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() Static position check not enabled."
            << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Debug);
  }
}
//----------------------------------------------------------------------------//
void ToolkitApplication::publishStaticPosCheckDiagnostics(
  const double&                                   time,
  const pnt_integrity::StaticPosCheckDiagnostics& checkData)
{
  Diagnostics staticPosDiagMsg;
  staticPosDiagMsg.header.deviceId = "integrity";
  staticPosDiagMsg.header.seq_num  = 0;  // doesn't matter for toolkit app

  staticPosDiagMsg.header.timestampArrival.nanoseconds =
    (int)(time * 1e9) % 1000000000;
  staticPosDiagMsg.header.timestampArrival.sec      = time;
  staticPosDiagMsg.header.timestampArrival.timecode = 100;
  staticPosDiagMsg.header.timestampValid =
    staticPosDiagMsg.header.timestampArrival;

  staticPosDiagMsg.level      = LevelEnum::DIAG_OK;
  staticPosDiagMsg.name       = pnt_integrity::INTEGRITY_STATIC_POS_DIAGNOSTICS;
  staticPosDiagMsg.message    = "Static-Pos check data";
  staticPosDiagMsg.hardwareId = "integrity_monitor";
  staticPosDiagMsg.numValues  = 0;

  KeyValue latKV;
  latKV.key   = pnt_integrity::INTEGRITY_STAIC_POS_DIAG_POS_LAT;
  latKV.value = std::to_string(checkData.staticPosition.latitude);
  staticPosDiagMsg.numValues++;
  staticPosDiagMsg.values.push_back(latKV);

  KeyValue lonKV;
  lonKV.key   = pnt_integrity::INTEGRITY_STAIC_POS_DIAG_POS_LON;
  lonKV.value = std::to_string(checkData.staticPosition.longitude);
  staticPosDiagMsg.numValues++;
  staticPosDiagMsg.values.push_back(lonKV);

  KeyValue altKV;
  altKV.key   = pnt_integrity::INTEGRITY_STAIC_POS_DIAG_POS_ALT;
  altKV.value = std::to_string(checkData.staticPosition.altitude);
  staticPosDiagMsg.numValues++;
  staticPosDiagMsg.values.push_back(altKV);

  KeyValue posChgKV;
  posChgKV.key   = pnt_integrity::INTEGRITY_STAIC_POS_DIAG_POS_CHNG_THRESH;
  posChgKV.value = std::to_string(checkData.posChangeThresh);
  staticPosDiagMsg.numValues++;
  staticPosDiagMsg.values.push_back(posChgKV);

  KeyValue perOverKV;
  perOverKV.key   = pnt_integrity::INTEGRITY_STAIC_POS_DIAG_PERCENT_OVER;
  perOverKV.value = std::to_string(checkData.percentOverThresh);
  staticPosDiagMsg.numValues++;
  staticPosDiagMsg.values.push_back(perOverKV);

  KeyValue iThreshKV;
  iThreshKV.key = pnt_integrity::INTEGRITY_STAIC_POS_DIAG_ITHRESH;

  iThreshKV.value = std::to_string(checkData.inconsistentThresh);
  staticPosDiagMsg.numValues++;
  staticPosDiagMsg.values.push_back(iThreshKV);

  KeyValue uThreshKV;
  uThreshKV.key   = pnt_integrity::INTEGRITY_STAIC_POS_DIAG_UTHRESH;
  uThreshKV.value = std::to_string(checkData.unassuredThresh);
  staticPosDiagMsg.numValues++;
  staticPosDiagMsg.values.push_back(uThreshKV);

  mainWindow_.sendDiagnostics(staticPosDiagMsg);
}
//==============================================================================
//---------------------------- Cno check
// functions------------------------------
//==============================================================================
void ToolkitApplication::initializeCnoCheck()
{
  useCnoCheck_ = yamlParserPtr_->readVariable<bool>("enable_cno_check", false);

  if (useCnoCheck_)
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() Initialize C/N0 check." << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Info);

    cnoCheckName_ =
      yamlParserPtr_->readVariable<std::string>("cno_check_name", "cno_check");

    int cnoFilterWindow;
    cnoFilterWindow =
      yamlParserPtr_->readVariable<int>("cno_filter_window", 10);

    cnoCheckPtr_ = std::make_shared<pnt_integrity::CnoCheck>(
      cnoCheckName_, cnoFilterWindow, log_);
    cnoCheckPtr_->setLogMessageHandler(std::bind(&ToolkitApplication::handleLog,
                                                 this,
                                                 std::placeholders::_1,
                                                 std::placeholders::_2));

    cnoCheckPtr_->setPublishDiagnostics(
      std::bind(&ToolkitApplication::publishCnoCheckDiagnostics,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    cnoCheckPtr_->setAssuranceThresholds(
      yamlParserPtr_->readVariable<double>("cno_check_inconsistent_thresh",
                                           4.0),
      yamlParserPtr_->readVariable<double>("cno_check_unassured_thresh", 6.0));

    cnoCheckPtr_->setAssuranceLevelPeriod(
      yamlParserPtr_->readVariable<double>("cno_check_level_period", 5.0));

    double checkWeight =
      yamlParserPtr_->readVariable<double>("cno_check_weight", 1.0);
    cnoCheckPtr_->setWeight(checkWeight);

    integrityMonitor_.registerCheck(cnoCheckName_, cnoCheckPtr_.get());
  }
  else
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() C/N0 check not enabled." << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Debug);
  }
}

//----------------------------------------------------------------------------//
void ToolkitApplication::publishCnoCheckDiagnostics(
  const double&                             time,
  const pnt_integrity::CnoCheckDiagnostics& checkData)
{
  Diagnostics diagnostics;
  diagnostics.header.deviceId = "integrity";
  diagnostics.header.seq_num  = 0;  // doesn't matter for toolkit app

  diagnostics.header.timestampArrival.nanoseconds =
    (int)(time * 1e9) % 1000000000;
  diagnostics.header.timestampArrival.sec      = time;
  diagnostics.header.timestampArrival.timecode = 100;
  diagnostics.header.timestampValid = diagnostics.header.timestampArrival;

  diagnostics.level      = LevelEnum::DIAG_OK;
  diagnostics.name       = pnt_integrity::INTEGRITY_CN0_DIAGNOSTICS;
  diagnostics.message    = "CNO check data";
  diagnostics.hardwareId = "integrity_monitor";
  diagnostics.numValues  = 0;

  KeyValue kvPair;
  kvPair.key   = pnt_integrity::INTEGRITY_CN0_DIAG_AVG_COUNT;
  kvPair.value = std::to_string(checkData.averageCount);
  diagnostics.numValues++;
  diagnostics.values.push_back(kvPair);

  KeyValue iThreshKV;
  iThreshKV.key   = pnt_integrity::INTEGRITY_CN0_DIAG_ITHRESH;
  iThreshKV.value = std::to_string(checkData.inconsistentThresh);
  diagnostics.numValues++;
  diagnostics.values.push_back(iThreshKV);

  KeyValue uThreshKV;
  uThreshKV.key   = pnt_integrity::INTEGRITY_CN0_DIAG_UTHRESH;
  uThreshKV.value = std::to_string(checkData.unassuredThresh);
  diagnostics.numValues++;
  diagnostics.values.push_back(uThreshKV);

  mainWindow_.sendDiagnostics(diagnostics);
}

//==============================================================================
//------------------------ Pos Jump Check functions --------------------------
//==============================================================================
void ToolkitApplication::initializePosJumpCheck()
{
  usePosJumpCheck_ =
    yamlParserPtr_->readVariable<bool>("enable_pos_jump_check");

  if (usePosJumpCheck_)
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() Initialize position jump check."
            << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Info);

    posJumpCheckName_ = yamlParserPtr_->readVariable<std::string>(
      "pos_jump_check_name", "pos_jump_check");
    double minimumBound =
      yamlParserPtr_->readVariable<double>("minimum_bound", 15);
    double maximumVelocity =
      yamlParserPtr_->readVariable<double>("maximum_velocity", 3.0);

    posJumpCheckPtr_ = std::make_shared<pnt_integrity::PositionJumpCheck>(
      posJumpCheckName_,
      minimumBound,
      false,  // not used in toolkit
      false,  // not used in toolkit
      maximumVelocity,
      1.0,  // not used in toolkit
      log_);

    posJumpCheckPtr_->setPublishDiagnostics(
      std::bind(&ToolkitApplication::publishPosJumpCheckDiagnostics,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    posJumpCheckPtr_->setAssuranceLevelPeriod(
      yamlParserPtr_->readVariable<double>("position_jump_check_level_period",
                                           5.0));

    double checkWeight =
      yamlParserPtr_->readVariable<double>("position_jump_check_weight", 1.0);
    posJumpCheckPtr_->setWeight(checkWeight);

    integrityMonitor_.registerCheck(posJumpCheckName_, posJumpCheckPtr_.get());
  }
  else
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() Position jump check not enabled."
            << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Debug);
  }
}
//----------------------------------------------------------------------------//
void ToolkitApplication::publishPosJumpCheckDiagnostics(
  const double&                                 time,
  const pnt_integrity::PosJumpCheckDiagnostics& checkData)
{
  Diagnostics posJumpDiagMsg;
  posJumpDiagMsg.header.deviceId = "integrity";
  posJumpDiagMsg.header.seq_num  = 0;  // doesn't matter for toolkit app

  posJumpDiagMsg.header.timestampArrival.nanoseconds =
    (int)(time * 1e9) % 1000000000;
  posJumpDiagMsg.header.timestampArrival.sec      = time;
  posJumpDiagMsg.header.timestampArrival.timecode = 100;
  posJumpDiagMsg.header.timestampValid = posJumpDiagMsg.header.timestampArrival;

  posJumpDiagMsg.level      = LevelEnum::DIAG_OK;
  posJumpDiagMsg.name       = pnt_integrity::INTEGRITY_POS_JUMP_DIAGNOSTICS;
  posJumpDiagMsg.message    = "Pos-Jump check data";
  posJumpDiagMsg.hardwareId = "integrity_monitor";
  posJumpDiagMsg.numValues  = 0;

  KeyValue distKV;
  distKV.key   = pnt_integrity::INTEGRITY_POS_JUMP_DIAG_DIST;
  distKV.value = std::to_string(checkData.distance);
  posJumpDiagMsg.numValues++;
  posJumpDiagMsg.values.push_back(distKV);

  KeyValue boundKV;
  boundKV.key   = pnt_integrity::INTEGRITY_POS_JUMP_DIAG_BOUND;
  boundKV.value = std::to_string(checkData.bound);
  posJumpDiagMsg.numValues++;
  posJumpDiagMsg.values.push_back(boundKV);

  mainWindow_.sendDiagnostics(posJumpDiagMsg);
}

//==============================================================================
//---------------- Position Velocity Consistency check functions
//---------------
//==============================================================================
void ToolkitApplication::initializePosVelConsCheck()
{
  usePosVelConsCheck_ =
    yamlParserPtr_->readVariable<bool>("enable_pos_vel_cons_check", false);

  if (usePosVelConsCheck_)
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() Initialize pos-vel consistency check."
            << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Info);

    posVelConsCheckName_ = yamlParserPtr_->readVariable<std::string>(
      "pos_vel_cons_check_name", "pos_vel_cons_check");

    double posVelConsSamleWindow =
      yamlParserPtr_->readVariable<double>("pos_vel_cons_sample_window", 5.0);

    double posVelConsErrorThreshSF = yamlParserPtr_->readVariable<double>(
      "pos_vel_cons_error_thresh_scale_factor", 2.0);

    posVelConsCheckPtr_ =
      std::make_shared<pnt_integrity::PositionVelocityConsistencyCheck>(
        posVelConsCheckName_,
        posVelConsSamleWindow,
        posVelConsErrorThreshSF,
        log_);

    posVelConsCheckPtr_->setPublishDiagnostics(
      std::bind(&ToolkitApplication::publishPvcCheckDiagnostics,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    posVelConsCheckPtr_->setAssuranceLevelPeriod(
      yamlParserPtr_->readVariable<double>("pos_vel_cons_check_level_period",
                                           5.0));

    double posVelConsCheckInconsistentThresh =
      yamlParserPtr_->readVariable<double>(
        "pos_vel_cons_check_inconsistent_thresh", 0.3);

    double posVelConsCheckUnassuredThresh =
      yamlParserPtr_->readVariable<double>(
        "pos_vel_cons_check_unassured_thresh", 0.7);

    posVelConsCheckPtr_->setAssuranceThresholds(
      posVelConsCheckInconsistentThresh, posVelConsCheckUnassuredThresh);

    posVelConsCheckPtr_->setWeight(
      yamlParserPtr_->readVariable<double>("pos_vel_cons_check_weight", 1.0));

    integrityMonitor_.registerCheck(posVelConsCheckName_,
                                    posVelConsCheckPtr_.get());
  }
  else
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() Pos-Vel consistency check not enabled."
            << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Debug);
  }
}
//---------------- Publish PVC Check data ---------------
void ToolkitApplication::publishPvcCheckDiagnostics(
  const double&                                    time,
  const pnt_integrity::PosVelConsCheckDiagnostics& checkData)
{
  Diagnostics pvcDiagMsg;
  pvcDiagMsg.header.deviceId = "integrity";
  pvcDiagMsg.header.seq_num  = 0;  // doesn't matter for toolkit app

  pvcDiagMsg.header.timestampArrival.nanoseconds =
    (int)(time * 1e9) % 1000000000;
  pvcDiagMsg.header.timestampArrival.sec      = time;
  pvcDiagMsg.header.timestampArrival.timecode = 100;
  pvcDiagMsg.header.timestampValid = pvcDiagMsg.header.timestampArrival;

  pvcDiagMsg.level      = LevelEnum::DIAG_OK;
  pvcDiagMsg.name       = pnt_integrity::INTEGRITY_PVC_DIAGNOSTICS;
  pvcDiagMsg.message    = "pvc check data";
  pvcDiagMsg.hardwareId = "integrity_monitor";

  KeyValue percentBadKV;
  percentBadKV.key   = pnt_integrity::INTEGRITY_PVC_DIAG_PB;
  percentBadKV.value = std::to_string(checkData.percentBad);
  pvcDiagMsg.numValues++;
  pvcDiagMsg.values.push_back(percentBadKV);

  KeyValue inconsistentThreshKV;
  inconsistentThreshKV.key   = pnt_integrity::INTEGRITY_PVC_DIAG_ITHRESH;
  inconsistentThreshKV.value = std::to_string(checkData.inconsistentThresh);
  pvcDiagMsg.numValues++;
  pvcDiagMsg.values.push_back(inconsistentThreshKV);

  KeyValue assuredThreshKV;
  assuredThreshKV.key   = pnt_integrity::INTEGRITY_PVC_DIAG_UTHRESH;
  assuredThreshKV.value = std::to_string(checkData.unassuredThresh);
  pvcDiagMsg.numValues++;
  pvcDiagMsg.values.push_back(assuredThreshKV);

  for (size_t ii = 0; ii < checkData.errorVals.size(); ++ii)
  {
    KeyValue errorValKV;
    errorValKV.key =
      pnt_integrity::INTEGRITY_PVC_DIAG_ERR_VAL + "_" + std::to_string(ii);
    errorValKV.value = std::to_string(checkData.errorVals[ii]);
    pvcDiagMsg.numValues++;
    pvcDiagMsg.values.push_back(errorValKV);

    KeyValue threshValKV;
    threshValKV.key =
      pnt_integrity::INTEGRITY_PVC_DIAG_ERR_THRESH + "_" + std::to_string(ii);
    threshValKV.value = std::to_string(checkData.errorThresh[ii]);
    pvcDiagMsg.numValues++;
    pvcDiagMsg.values.push_back(threshValKV);
  }

  mainWindow_.sendDiagnostics(pvcDiagMsg);
}

//==============================================================================
//---------------- AGC check functions ---------------
//==============================================================================
void ToolkitApplication::initializeAgcCheck()
{
  useAgcCheck_ = yamlParserPtr_->readVariable<bool>("enable_agc_check", false);

  if (useAgcCheck_)
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() Initialize AGC check." << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Info);

    agcCheckName_ =
      yamlParserPtr_->readVariable<std::string>("agc_check_name", "agc_check");

    double agcMinValue =
      yamlParserPtr_->readVariable<double>("min_agc_value", 0.0);
    double agcMaxValue =
      yamlParserPtr_->readVariable<double>("max_agc_value", 10000);

    agcCheckPtr_ = std::make_shared<pnt_integrity::AgcCheck>(
      agcCheckName_, agcMinValue, agcMaxValue, log_);

    agcCheckPtr_->setPublishDiagnostics(
      std::bind(&ToolkitApplication::publishAgcCheckDiagnostics,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    agcCheckPtr_->setAssuranceLevelPeriod(
      yamlParserPtr_->readVariable<double>("agc_check_level_period", 3.0));

    double agcCheckInconsistentThresh = yamlParserPtr_->readVariable<double>(
      "agc_check_inconsistent_thresh", 0.5);

    double agcCheckUnassuredThresh =
      yamlParserPtr_->readVariable<double>("agc_check_unassured_thresh", 0.7);

    agcCheckPtr_->setAssuranceThresholds(agcCheckInconsistentThresh,
                                         agcCheckUnassuredThresh);

    agcCheckPtr_->setWeight(
      yamlParserPtr_->readVariable<double>("agc_check_weight", 1.0));

    integrityMonitor_.registerCheck(agcCheckName_, agcCheckPtr_.get());
  }
  else
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() AGC check not enabled." << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Debug);
  }
}
//---------------- Publish AGC Check data ---------------

void ToolkitApplication::publishAgcCheckDiagnostics(
  const double&                             time,
  const pnt_integrity::AgcCheckDiagnostics& checkData)
{
  Diagnostics agcDiagMsg;
  agcDiagMsg.header.deviceId = "integrity";
  agcDiagMsg.header.seq_num  = 0;  // doesn't matter for toolkit app

  agcDiagMsg.header.timestampArrival.nanoseconds =
    (int)(time * 1e9) % 1000000000;
  agcDiagMsg.header.timestampArrival.sec      = time;
  agcDiagMsg.header.timestampArrival.timecode = 100;
  agcDiagMsg.header.timestampValid = agcDiagMsg.header.timestampArrival;

  agcDiagMsg.level      = LevelEnum::DIAG_OK;
  agcDiagMsg.name       = pnt_integrity::INTEGRITY_AGC_DIAGNOSTICS;
  agcDiagMsg.message    = "agc check data";
  agcDiagMsg.hardwareId = "integrity_monitor";

  KeyValue iThreshKV;
  iThreshKV.key   = pnt_integrity::INTEGRITY_AGC_DIAG_ITHRESH;
  iThreshKV.value = std::to_string(checkData.inconsistentThresh);
  agcDiagMsg.numValues++;
  agcDiagMsg.values.push_back(iThreshKV);

  auto agcValIt = checkData.values.agcValues.begin();
  for (; agcValIt != checkData.values.agcValues.end(); ++agcValIt)
  {
    KeyValue agcValKV;
    agcValKV.key   = "BAND_ENUM_" + std::to_string((int)agcValIt->first);
    agcValKV.value = std::to_string(agcValIt->second);
    agcDiagMsg.numValues++;
    agcDiagMsg.values.push_back(agcValKV);
  }

  mainWindow_.sendDiagnostics(agcDiagMsg);
}

//==============================================================================
//---------------- AOA check functions ---------------
//==============================================================================
void ToolkitApplication::initializeAoaCheck()
{
  useAoaCheck_ = yamlParserPtr_->readVariable<bool>("enable_aoa_check", false);
  if (useAoaCheck_)
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() Initialize AOA check." << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Info);

    aoaCheckName_ =
      yamlParserPtr_->readVariable<std::string>("aoa_check_name", "aoa_check");

    double singleDiffThresh =
      yamlParserPtr_->readVariable<double>("aoa_check_single_diff_thresh", 5.0);
    double rangeThresh =
      yamlParserPtr_->readVariable<double>("aoa_check_range_thresh", 5.0);
    double prnCountThresh =
      yamlParserPtr_->readVariable<double>("aoa_check_prn_count_thresh", 5.0);

    bool useCarrierPhase =
      yamlParserPtr_->readVariable<bool>("aoa_check_use_carrier_phase", false);

    pnt_integrity::AoaCheckData checkDataType =
      pnt_integrity::AoaCheckData::UsePseudorange;
    if (useCarrierPhase)
    {
      checkDataType = pnt_integrity::AoaCheckData::UseCarrierPhase;
    }
    aoaCheckPtr_ =
      std::make_shared<pnt_integrity::AngleOfArrivalCheck>(aoaCheckName_,
                                                           checkDataType,
                                                           singleDiffThresh,
                                                           prnCountThresh,
                                                           rangeThresh,
                                                           log_);

    aoaCheckPtr_->setPublishDiffData(
      std::bind(&ToolkitApplication::publishAoaCheckDiffs,
                this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3));

    aoaCheckPtr_->setPublishDiagnostics(
      std::bind(&ToolkitApplication::publishAoaCheckDiagnostics,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    aoaCheckPtr_->setAssuranceLevelPeriod(
      yamlParserPtr_->readVariable<double>("aoa_check_level_period", 3.0));

    double aoaCheckInconsistentThresh =
      yamlParserPtr_->readVariable<double>("aoa_check_inconsistent_thresh", 0.3);

    double aoaCheckUnassuredThresh =
      yamlParserPtr_->readVariable<double>("aoa_check_unassured_thresh",0.5);

    double aoaCheckAssuredThresh =
      yamlParserPtr_->readVariable<double>("aoa_check_assured_thresh", 0.75);

    aoaCheckPtr_->setAssuranceThresholds(aoaCheckInconsistentThresh,
                                         aoaCheckUnassuredThresh,
                                         aoaCheckAssuredThresh);

    aoaCheckPtr_->setDifferenceComparisonFailureLimit(
      yamlParserPtr_->readVariable<double>("aoa_check_single_diff_fail_limit", 0.5));

    aoaCheckPtr_->setWeight(
      yamlParserPtr_->readVariable<double>("aoa_check_weight", 1.0));

    integrityMonitor_.registerCheck(aoaCheckName_, aoaCheckPtr_.get());
  }
  else
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() AOA check not enabled." << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Debug);
  }
}
//----------------------------------------------------------------------------//
void ToolkitApplication::publishAoaCheckDiffs(
  const double&                       time,
  const std::string&                  nodeID,
  const pnt_integrity::SingleDiffMap& diffMap)
{
  Diagnostics aoaDiffMsg;
  aoaDiffMsg.header.deviceId = "integrity";
  aoaDiffMsg.header.seq_num  = 0;  // doesn't matter for toolkit app

  aoaDiffMsg.header.timestampArrival.nanoseconds =
    (int)(time * 1e9) % 1000000000;
  aoaDiffMsg.header.timestampArrival.sec      = time;
  aoaDiffMsg.header.timestampArrival.timecode = 100;
  aoaDiffMsg.header.timestampValid = aoaDiffMsg.header.timestampArrival;

  aoaDiffMsg.level      = LevelEnum::DIAG_OK;
  aoaDiffMsg.name       = pnt_integrity::INTEGRITY_AOA_DIFF_DIAGNOSTICS;
  aoaDiffMsg.message    = "aoa check diff data";
  aoaDiffMsg.hardwareId = "integrity_monitor";

  KeyValue nodeIdKV;
  nodeIdKV.key   = pnt_integrity::INTEGRITY_AOA_DIFF_NODE_ID;
  nodeIdKV.value = nodeID;
  aoaDiffMsg.numValues++;
  aoaDiffMsg.values.push_back(nodeIdKV);

  auto   diffIt   = diffMap.begin();
  double baseDiff = diffIt->second;
  for (; diffIt != diffMap.end(); ++diffIt)
  {
    KeyValue diffKV;
    diffKV.key = std::to_string(diffIt->first);
    std::stringstream diffStr;
    diffStr << std::setprecision(10) << (diffIt->second - baseDiff);
    diffKV.value = diffStr.str();
    aoaDiffMsg.numValues++;
    aoaDiffMsg.values.push_back(diffKV);
  }

  mainWindow_.sendDiagnostics(aoaDiffMsg);
}

//----------------------------------------------------------------------------//
void ToolkitApplication::publishAoaCheckDiagnostics(
  const double&                             time,
  const pnt_integrity::AoaCheckDiagnostics& checkData)
{
  Diagnostics aoaDiagMsg;
  aoaDiagMsg.header.deviceId = "integrity";
  aoaDiagMsg.header.seq_num  = 0;  // doesn't matter for toolkit app

  aoaDiagMsg.header.timestampArrival.nanoseconds =
    (int)(time * 1e9) % 1000000000;
  aoaDiagMsg.header.timestampArrival.sec      = time;
  aoaDiagMsg.header.timestampArrival.timecode = 100;
  aoaDiagMsg.header.timestampValid = aoaDiagMsg.header.timestampArrival;

  aoaDiagMsg.level      = LevelEnum::DIAG_OK;
  aoaDiagMsg.name       = pnt_integrity::INTEGRITY_AOA_DIAGNOSTICS;
  aoaDiagMsg.message    = "aoa check diff diagnostics";
  aoaDiagMsg.hardwareId = "integrity_monitor";

  KeyValue diffThreshKV;
  diffThreshKV.key   = pnt_integrity::INTEGRITY_AOA_DIAG_DIFF_THRESH;
  diffThreshKV.value = std::to_string(checkData.singleDiffThresh);
  aoaDiagMsg.numValues++;
  aoaDiagMsg.values.push_back(diffThreshKV);

  // Suspect Percent
  KeyValue prnCountKV;
  prnCountKV.key   = pnt_integrity::INTEGRITY_AOA_DIAG_SUSPECT_PRN_PERCENT;
  prnCountKV.value = std::to_string(checkData.suspectPrnPercent);
  aoaDiagMsg.numValues++;
  aoaDiagMsg.values.push_back(prnCountKV);

  // Assured Percent
  KeyValue assuredPrnCountKV;
  assuredPrnCountKV.key   = pnt_integrity::INTEGRITY_AOA_DIAG_ASSURED_PRN_PERCENT;
  assuredPrnCountKV.value = std::to_string(checkData.assuredPrnPercent);
  aoaDiagMsg.numValues++;
  aoaDiagMsg.values.push_back(assuredPrnCountKV);

  // Unavailable Percent
  KeyValue unavailablePrnCountKV;
  unavailablePrnCountKV.key   = pnt_integrity::INTEGRITY_AOA_DIAG_UNAVAILABLE_PRN_PERCENT;
  unavailablePrnCountKV.value = std::to_string(checkData.unavailablePrnPercent);
  aoaDiagMsg.numValues++;
  aoaDiagMsg.values.push_back(unavailablePrnCountKV);

  // Inconsistent Percent Threshold
  KeyValue iThreshKV;
  iThreshKV.key   = pnt_integrity::INTEGRITY_AOA_DIAG_ITHRESH;
  iThreshKV.value = std::to_string(checkData.inconsistentThresh);
  aoaDiagMsg.numValues++;
  aoaDiagMsg.values.push_back(iThreshKV);

  // Unassured Percent Threshold
  KeyValue uThreshKV;
  uThreshKV.key   = pnt_integrity::INTEGRITY_AOA_DIAG_UTHRESH;
  uThreshKV.value = std::to_string(checkData.unassuredThresh);
  aoaDiagMsg.numValues++;
  aoaDiagMsg.values.push_back(uThreshKV);

  // Assured Percent Threshold
  KeyValue aThreshKV;
  aThreshKV.key   = pnt_integrity::INTEGRITY_AOA_DIAG_ATHRESH;
  aThreshKV.value = std::to_string(checkData.assuredThresh);
  aoaDiagMsg.numValues++;
  aoaDiagMsg.values.push_back(aThreshKV);

  mainWindow_.sendDiagnostics(aoaDiagMsg);
}

//==============================================================================
//---------------- Clock bias check functions ---------------
//==============================================================================
void ToolkitApplication::initializeClockBiasCheck()
{
  useClockBiasCheck_ =
    yamlParserPtr_->readVariable<bool>("enable_clock_bias_check", false);

  if (useClockBiasCheck_)
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() Initialize Clock bias check." << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Info);

    clockBiasCheckName_ = yamlParserPtr_->readVariable<std::string>(
      "clock_bias_check_name", "clock_bias_check");

    uint32_t minNumSamples = yamlParserPtr_->readVariable<uint32_t>(
      "clock_bias_check_min_samples", 10);

    uint32_t maxNumSamples = yamlParserPtr_->readVariable<uint32_t>(
      "clock_bias_check_max_samples", 30);

    uint32_t sampleWindow = yamlParserPtr_->readVariable<uint32_t>(
      "clock_bias_check_sample_window", 10);

    double driftRateBound = yamlParserPtr_->readVariable<double>(
      "clock_bias_check_drift_rate_bound", 0.0000005);

    double driftRateVarBound = yamlParserPtr_->readVariable<double>(
      "clock_bias_check_drift_rate_var_bound", 0.000001);

    clockBiasCheckPtr_ =
      std::make_shared<pnt_integrity::ClockBiasCheck>(clockBiasCheckName_,
                                                      minNumSamples,
                                                      maxNumSamples,
                                                      sampleWindow,
                                                      driftRateBound,
                                                      driftRateVarBound,
                                                      log_);

    clockBiasCheckPtr_->setPublishDiagnostics(
      std::bind(&ToolkitApplication::publishClockBiasCheckDiagnostics,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    // // currently, check thresholds are not used

    clockBiasCheckPtr_->setWeight(
      yamlParserPtr_->readVariable<double>("clock_bias_check_weight", 1.0));

    integrityMonitor_.registerCheck(clockBiasCheckName_,
                                    clockBiasCheckPtr_.get());
  }
  else
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() Clock bias check not enabled." << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Debug);
  }
}

//---------------- Publish Clock Bias Check data ---------------

void ToolkitApplication::publishClockBiasCheckDiagnostics(
  const double&                                   time,
  const pnt_integrity::ClockBiasCheckDiagnostics& checkData)
{
  Diagnostics cbDiagMsg;
  cbDiagMsg.header.deviceId = "integrity";
  cbDiagMsg.header.seq_num  = 0;  // doesn't matter for toolkit app

  cbDiagMsg.header.timestampArrival.nanoseconds =
    (int)(time * 1e9) % 1000000000;
  cbDiagMsg.header.timestampArrival.sec      = time;
  cbDiagMsg.header.timestampArrival.timecode = 100;
  cbDiagMsg.header.timestampValid = cbDiagMsg.header.timestampArrival;

  cbDiagMsg.level      = LevelEnum::DIAG_OK;
  cbDiagMsg.name       = pnt_integrity::INTEGRITY_CLOCK_BIAS_DIAGNOSTICS;
  cbDiagMsg.message    = "clock bias check data";
  cbDiagMsg.hardwareId = "integrity_monitor";

  std::stringstream             driftStr;
  KeyValue expDriftKV;
  expDriftKV.key = pnt_integrity::INTEGRITY_CLOCK_BIAS_DIAG_EXP_DRIFT;
  driftStr << std::setprecision(10) << checkData.expectedDrift;
  expDriftKV.value = driftStr.str();
  cbDiagMsg.numValues++;
  cbDiagMsg.values.push_back(expDriftKV);

  std::stringstream             driftVarStr;
  KeyValue expDrifVartKV;
  expDrifVartKV.key = pnt_integrity::INTEGRITY_CLOCK_BIAS_DIAG_EXP_DRIFT_VAR;
  driftVarStr << checkData.expectedDriftVar;
  expDrifVartKV.value = driftVarStr.str();
  cbDiagMsg.numValues++;
  cbDiagMsg.values.push_back(expDrifVartKV);

  std::stringstream             offsetStr;
  KeyValue propOffsetKV;
  propOffsetKV.key = pnt_integrity::INTEGRITY_CLOCK_BIAS_DIAG_PROP_OFFSET;
  offsetStr << std::setprecision(10) << checkData.propagatedOffset;
  propOffsetKV.value = offsetStr.str();
  cbDiagMsg.numValues++;
  cbDiagMsg.values.push_back(propOffsetKV);

  std::stringstream             actOffsetStr;
  KeyValue actualOffsetKV;
  actualOffsetKV.key = pnt_integrity::INTEGRITY_CLOCK_BIAS_DIAG_ACTUAL_OFFSET;
  actOffsetStr << std::setprecision(10) << checkData.actualOffset;
  actualOffsetKV.value = actOffsetStr.str();
  cbDiagMsg.numValues++;
  cbDiagMsg.values.push_back(actualOffsetKV);

  std::stringstream             offsetErrStr;
  KeyValue offsetErrKV;
  offsetErrKV.key = pnt_integrity::INTEGRITY_CLOCK_BIAS_DIAG_OFFSET_ERROR;
  offsetErrStr << std::setprecision(10) << checkData.offsetError;
  offsetErrKV.value = offsetErrStr.str();
  cbDiagMsg.numValues++;
  cbDiagMsg.values.push_back(offsetErrKV);

  std::stringstream             driftRateStr;
  KeyValue driftRateBoundKV;
  driftRateBoundKV.key =
    pnt_integrity::INTEGRITY_CLOCK_BIAS_DIAG_DRIFT_RATE_BOUND;
  driftRateStr << std::setprecision(10) << checkData.driftRateBound;
  driftRateBoundKV.value = driftRateStr.str();
  cbDiagMsg.numValues++;
  cbDiagMsg.values.push_back(driftRateBoundKV);

  std::stringstream             driftRateBoundStr;
  KeyValue driftRateVarBoundKV;
  driftRateVarBoundKV.key =
    pnt_integrity::INTEGRITY_CLOCK_BIAS_DIAG_DRIFT_RATE_VAR_BOUND;
  driftRateBoundStr << std::setprecision(10) << checkData.driftRateVarBound;
  driftRateVarBoundKV.value = driftRateBoundStr.str();
  cbDiagMsg.numValues++;
  cbDiagMsg.values.push_back(driftRateVarBoundKV);

  mainWindow_.sendDiagnostics(cbDiagMsg);
}

//==============================================================================
//---------------- Navigation Data check functions ---------------
//==============================================================================
void ToolkitApplication::initializeNavDataCheck()
{
  useNavDataCheck_ =
    yamlParserPtr_->readVariable<bool>("enable_nav_data_check", false);

  if (useNavDataCheck_)
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() Initialize Navigation Data Check." << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Info);

    navDataCheckName_ = yamlParserPtr_->readVariable<std::string>(
      "nav_data_check_name", "nav_data_check");

    navDataCheckPtr_ =
      std::make_shared<pnt_integrity::NavigationDataCheck>(navDataCheckName_,
                                                      log_);

    navDataCheckPtr_->setPublishDiagnostics(
      std::bind(&ToolkitApplication::publishNavDataCheckDiagnostics,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    // // currently, check thresholds are not used

    navDataCheckPtr_->setWeight(
      yamlParserPtr_->readVariable<double>("nav_data_check_weight", 0.0));

    integrityMonitor_.registerCheck(navDataCheckName_,
                                    navDataCheckPtr_.get());
  }
  else
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() Clock bias check not enabled." << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Debug);
  }
}

//---------------- Publish Navigation Data Check data ---------------

void ToolkitApplication::publishNavDataCheckDiagnostics(
  const double&                                   time,
  const pnt_integrity::NavDataCheckDiagnostics& checkData)
{
  Diagnostics navDataDiagMsg;
  navDataDiagMsg.header.deviceId = "integrity";
  navDataDiagMsg.header.seq_num  = 0;  // doesn't matter for toolkit app

  navDataDiagMsg.header.timestampArrival.nanoseconds =
    (int)(time * 1e9) % 1000000000;
  navDataDiagMsg.header.timestampArrival.sec      = time;
  navDataDiagMsg.header.timestampArrival.timecode = 100;
  navDataDiagMsg.header.timestampValid = navDataDiagMsg.header.timestampArrival;

  navDataDiagMsg.level       = LevelEnum::DIAG_OK;
  navDataDiagMsg.name        = pnt_integrity::INTEGRITY_NAV_DATA_DIAGNOSTICS;
  navDataDiagMsg.message     = "nav check data";
  navDataDiagMsg.hardwareId = "integrity_monitor";

  KeyValue dataValidKV;
  dataValidKV.key = pnt_integrity::INTEGRITY_NAV_DATA_VALID;
  dataValidKV.value = std::to_string(checkData.dataValid);
  navDataDiagMsg.numValues++;
  navDataDiagMsg.values.push_back(dataValidKV);

  KeyValue dataValidMsgKV;
  dataValidMsgKV.key   = pnt_integrity::INTEGRITY_NAV_DATA_VALID_MSG;
  dataValidMsgKV.value = checkData.dataValidMsg;
  navDataDiagMsg.numValues++;
  navDataDiagMsg.values.push_back(dataValidMsgKV);

  KeyValue towValidKV;
  towValidKV.key = pnt_integrity::INTEGRITY_NAV_DATA_TOW_VALID;
  towValidKV.value = std::to_string(checkData.towValid);
  navDataDiagMsg.numValues++;
  navDataDiagMsg.values.push_back(towValidKV);

  KeyValue towValidMsgKV;
  towValidMsgKV.key   = pnt_integrity::INTEGRITY_NAV_DATA_TOW_VALID_MSG;
  towValidMsgKV.value = checkData.towValidMsg;
  navDataDiagMsg.numValues++;
  navDataDiagMsg.values.push_back(towValidMsgKV);

  KeyValue wnValidKV;
  wnValidKV.key = pnt_integrity::INTEGRITY_NAV_DATA_WN_VALID;
  wnValidKV.value = std::to_string(checkData.wnValid);
  navDataDiagMsg.numValues++;
  navDataDiagMsg.values.push_back(wnValidKV);

  KeyValue wmValidMsgKV;
  wmValidMsgKV.key   = pnt_integrity::INTEGRITY_NAV_DATA_WN_VALID_MSG;
  wmValidMsgKV.value = checkData.wnValidMsg;
  navDataDiagMsg.numValues++;
  navDataDiagMsg.values.push_back(wmValidMsgKV);
  
  mainWindow_.sendDiagnostics(navDataDiagMsg);
}

//==============================================================================
//---------------- Acquisition check functions ---------------
//==============================================================================
void ToolkitApplication::initializeAcquisitionCheck()
{
#ifdef PNT_INTEGRITY_INCLUDES_ACQ_CHECK
  useAcquisitionCheck_ =
    yamlParserPtr_->readVariable<bool>("enable_acquisition_check", false);
  if (useAcquisitionCheck_)
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() Initialize acquisition check." << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Info);

    acqCheckName_ =
      yamlParserPtr_->readVariable<std::string>("acq_check_name", "acq_check");

    double samplingRate, highPowerThreshold, peakRatioThreshold,
      acquisitionThreshold, intermediateFreq, codeFreqBasis, searchBand,
      stepSize, intPeriod;
    int codeLength;

    // read in acquisition check parameters
    samplingRate =
      yamlParserPtr_->readVariable<double>("if_data_sampling_rate", 5000000.0);

    intermediateFreq =
      yamlParserPtr_->readVariable<double>("if_data_intermediate_freq", 0.0);

    searchBand =
      yamlParserPtr_->readVariable<double>("acq_check_search_band", 10000.0);

    stepSize =
      yamlParserPtr_->readVariable<double>("acq_check_search_step_size", 500.0);

    intPeriod = yamlParserPtr_->readVariable<double>(
      "acq_check_integration_period", 0.001);

    codeFreqBasis = yamlParserPtr_->readVariable<double>(
      "acq_check_code_freq_basis", 1023000.0);

    codeLength =
      yamlParserPtr_->readVariable<int>("acq_check_code_length", 1023);

    highPowerThreshold =
      yamlParserPtr_->readVariable<double>("high_power_threshold", 25000000);
    peakRatioThreshold =
      yamlParserPtr_->readVariable<double>("peak_ratio_threshold", 3.0);
    acquisitionThreshold =
      yamlParserPtr_->readVariable<double>("acquisition_threshold", 3000000);

    acqCheckPtr_ =
      std::make_shared<pnt_integrity::AcquisitionCheck>(acqCheckName_,
                                                        highPowerThreshold,
                                                        peakRatioThreshold,
                                                        acquisitionThreshold,
                                                        samplingRate,
                                                        intermediateFreq,
                                                        searchBand,
                                                        stepSize,
                                                        intPeriod,
                                                        codeFreqBasis,
                                                        codeLength,
                                                        log_);

    acqCheckPtr_->setPublishPeakData(
      std::bind(&ToolkitApplication::publishAcqCheckPeakData,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    acqCheckPtr_->setPublishDiagnostics(
      std::bind(&ToolkitApplication::publishAcqDiagnostics,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    double acqCheckUnknownThresh, acqCheckUnusableThresh;
    acqCheckUnknownThresh = yamlParserPtr_->readVariable<double>(
      "acq_check_inconsistent_thresh", 2.0);
    acqCheckUnusableThresh =
      yamlParserPtr_->readVariable<double>("acq_check_unassured_thresh", 4.0);

    acqCheckPtr_->setAssuranceThresholds(acqCheckUnknownThresh,
                                         acqCheckUnusableThresh);

    acqCheckPtr_->setAssuranceLevelPeriod(
      yamlParserPtr_->readVariable<double>("acq_check_level_period", 5.0));

    acqCheckPtr_->setWeight(
      yamlParserPtr_->readVariable<double>("acq_check_weight", 1.0));

    integrityMonitor_.registerCheck(acqCheckName_, acqCheckPtr_.get());
  }
  else
  {
    std::stringstream log_str;
    log_str << __FUNCTION__ << "() Acquisition check not enabled." << std::endl;
    handleLog(log_str.str(), logutils::LogLevel::Debug);
  }
#else
  useAcquisitionCheck_ = false;
#endif
}

//---------------- Publish acquisition Check data ---------------
#ifdef PNT_INTEGRITY_INCLUDES_ACQ_CHECK
void ToolkitApplication::publishAcqCheckPeakData(
  const double&                        time,
  const pnt_integrity::PeakResultsMap& peakData)
{
  Diagnostics acqPeakDataMsg;
  acqPeakDataMsg.header.deviceId = "integrity";
  acqPeakDataMsg.header.seq_num  = 0;  // doesn't matter for toolkit app

  acqPeakDataMsg.header.timestampArrival.nanoseconds =
    (int)(time * 1e9) % 1000000000;
  acqPeakDataMsg.header.timestampArrival.sec      = time;
  acqPeakDataMsg.header.timestampArrival.timecode = 100;
  acqPeakDataMsg.header.timestampValid = acqPeakDataMsg.header.timestampArrival;

  acqPeakDataMsg.level      = LevelEnum::DIAG_OK;
  acqPeakDataMsg.name       = pnt_integrity::INTEGRITY_ACQ_PEAK_VALS;
  acqPeakDataMsg.message    = "acquisition check peak data";
  acqPeakDataMsg.hardwareId = "integrity_monitor";

  for (auto peakIt = peakData.begin(); peakIt != peakData.end(); ++peakIt)
  {
    KeyValue firstPeakKv;
    firstPeakKv.key = pnt_integrity::INTEGRITY_ACQ_PEAK1_KEY +
                      std::to_string((int)peakIt->first);
    firstPeakKv.value = std::to_string(peakIt->second.first);
    acqPeakDataMsg.numValues++;
    acqPeakDataMsg.values.push_back(firstPeakKv);

    KeyValue secondPeakKv;
    secondPeakKv.key = pnt_integrity::INTEGRITY_ACQ_PEAK2_KEY +
                       std::to_string((int)peakIt->first);
    secondPeakKv.value = std::to_string(peakIt->second.second);
    acqPeakDataMsg.numValues++;
    acqPeakDataMsg.values.push_back(secondPeakKv);
  }

  mainWindow_.sendDiagnostics(acqPeakDataMsg);
}

void ToolkitApplication::publishAcqDiagnostics(
  const double&                             time,
  const pnt_integrity::AcqCheckDiagnostics& diagnostics)
{
  Diagnostics acqDiagDataMsg;
  acqDiagDataMsg.header.deviceId = "integrity";
  acqDiagDataMsg.header.seq_num  = 0;  // doesn't matter for toolkit app

  acqDiagDataMsg.header.timestampArrival.nanoseconds =
    (int)(time * 1e9) % 1000000000;
  acqDiagDataMsg.header.timestampArrival.sec      = time;
  acqDiagDataMsg.header.timestampArrival.timecode = 100;
  acqDiagDataMsg.header.timestampValid = acqDiagDataMsg.header.timestampArrival;

  acqDiagDataMsg.level      = LevelEnum::DIAG_OK;
  acqDiagDataMsg.name       = pnt_integrity::INTEGRITY_ACQ_DIAGNOSTICS;
  acqDiagDataMsg.message    = "acquisition check diagnostics";
  acqDiagDataMsg.hardwareId = "integrity_monitor";

  KeyValue hiPwrThreshKv;
  hiPwrThreshKv.key   = pnt_integrity::INT_ACQ_DIAG_HI_PWR_THRESH;
  hiPwrThreshKv.value = std::to_string(diagnostics.highPowerThresh);
  acqDiagDataMsg.numValues++;
  acqDiagDataMsg.values.push_back(hiPwrThreshKv);

  KeyValue peakRatioThreshKv;
  peakRatioThreshKv.key   = pnt_integrity::INT_ACQ_DIAG_PEAK_RATIO_THRESH;
  peakRatioThreshKv.value = std::to_string(diagnostics.peakRatioThresh);
  acqDiagDataMsg.numValues++;
  acqDiagDataMsg.values.push_back(peakRatioThreshKv);

  KeyValue acqThreshKv;
  acqThreshKv.key   = pnt_integrity::INT_ACQ_DIAG_ACQ_THRESH;
  acqThreshKv.value = std::to_string(diagnostics.acquisitionThresh);
  acqDiagDataMsg.numValues++;
  acqDiagDataMsg.values.push_back(acqThreshKv);

  KeyValue iThreshKv;
  iThreshKv.key   = pnt_integrity::INT_ACQ_DIAG_ITHRESH;
  iThreshKv.value = std::to_string(diagnostics.inconsistentThresh);
  acqDiagDataMsg.numValues++;
  acqDiagDataMsg.values.push_back(iThreshKv);

  KeyValue uThreshKv;
  uThreshKv.key   = pnt_integrity::INT_ACQ_DIAG_UTHRESH;
  uThreshKv.value = std::to_string(diagnostics.unassuredThresh);
  acqDiagDataMsg.numValues++;
  acqDiagDataMsg.values.push_back(uThreshKv);

  KeyValue iCountKv;
  iCountKv.key   = pnt_integrity::INT_ACQ_DIAG_ICOUNT;
  iCountKv.value = std::to_string(diagnostics.inconsistentCount);
  acqDiagDataMsg.numValues++;
  acqDiagDataMsg.values.push_back(iCountKv);

  KeyValue uCountKv;
  uCountKv.key   = pnt_integrity::INT_ACQ_DIAG_UCOUNT;
  uCountKv.value = std::to_string(diagnostics.unassuredCount);
  acqDiagDataMsg.numValues++;
  acqDiagDataMsg.values.push_back(uCountKv);

  for (auto ratioIt = diagnostics.ratioMap.begin();
       ratioIt != diagnostics.ratioMap.end();
       ++ratioIt)
  {
    KeyValue peakRatioKv;
    peakRatioKv.key = pnt_integrity::INT_ACQ_DIAG_PEAK_RATIO_KEY +
                      std::to_string((int)ratioIt->first);
    peakRatioKv.value = std::to_string(ratioIt->second);
    acqDiagDataMsg.numValues++;
    acqDiagDataMsg.values.push_back(peakRatioKv);
  }

  mainWindow_.sendDiagnostics(acqDiagDataMsg);
}
#endif
}  // namespace integrity_toolkit