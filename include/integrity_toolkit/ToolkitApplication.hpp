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
/// \date     April 20, 2020
//============================================================================//
#ifndef INTEGRITY_TOOLKIT__TOOLKIT_APPLICAION_HPP
#define INTEGRITY_TOOLKIT__TOOLKIT_APPLICAION_HPP

#include <QApplication>
#include <chrono>
#include <fstream>
#include <future>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <uhd/transport/bounded_buffer.hpp>
#include <utility>  // std::pair, std::make_pair

#include "if_data_utils/IFDataFileReader.hpp"
#include "if_data_utils/IFSampleData.hpp"
#include "integrity_toolkit/ToolkitLCDDisplay.hpp"
#include "integrity_ui_base/mainwindow.h"
#include "logutils/logutils.hpp"
#include "pnt_integrity/AgcCheck.hpp"
#include "pnt_integrity/AngleOfArrivalCheck.hpp"
#include "pnt_integrity/ClockBiasCheck.hpp"
#include "pnt_integrity/CnoCheck.hpp"
#include "pnt_integrity/IntegrityData.hpp"
#include "pnt_integrity/IntegrityMonitor.hpp"
#include "pnt_integrity/NavigationDataCheck.hpp"
#include "pnt_integrity/PositionJumpCheck.hpp"
#include "pnt_integrity/PositionVelocityConsistencyCheck.hpp"
#include "pnt_integrity/RangePositionCheck.hpp"
#include "pnt_integrity/StaticPositionCheck.hpp"
#include "serial/serial.h"
#include "ublox/ublox.h"
#include "usrp_utilities/SurrpasUsrpDevice.hpp"
#include "yaml_parser/yaml_parser.hpp"

#include "integrity_toolkit/CsvWrite.hpp"

#ifdef PNT_INTEGRITY_INCLUDES_ACQ_CHECK
#include "pnt_integrity/AcquisitionCheck.hpp"
#endif

/// \brief Namespace for the toolkit application
namespace integrity_toolkit
{
/// \brief Pi (pi), dimensionless
const double pi = 3.1415926535898;

/// \brief pi/2, dimensionless
const double piOver2 = 0.5 * pi;

/// \brief 2*pi, dimensionless
const double piTimes2 = 2.0 * pi;

/// \brief Ratio of degrees to radians, dimensionless
const double deg2rad = pi / 180.0;

/// \brief Ratio of degrees to radians squared, dimensionless
const double deg2radSquared = deg2rad * deg2rad;

/// \brief Ratio of radians to degrees, dimensionless
const double rad2deg = 1.0 / deg2rad;

/// \brief Ratio of radians to degrees squared, dimensionless
const double rad2degSquared = rad2deg * rad2deg;

/// \brief Container class for all toolkit functionality
///
/// The ToolkitApplication class is a wrapper container for all
/// of the hardware drivers, libraries, and user interface objects
/// needed for the toolkit application
class ToolkitApplication
{
public:
  /// \brief Constructor for the application object
  ToolkitApplication(int argc, char* argv[]);

  /// \brief Default constructor for the application object
  ~ToolkitApplication()
  {
    lcdExitSignal_.set_value();
    ifDataExitSignal_.set_value();
    csvAssuranceReportWriter_.closeFile();
  };

  /// \brief Launches the application
  int run();

private:
  QApplication qApp_;
  MainWindow   mainWindow_;

  std::unique_ptr<yaml_parser::YamlParser> yamlParserPtr_;

  logutils::LogCallback log_;
  logutils::LogLevel    minLogLevel_;
  void handleLog(const std::string& msg, const logutils::LogLevel& logLevel)
  {
    if (logLevel <= minLogLevel_)
    {
      log_(msg, logLevel);
    }
  }

  bool                            pushMode_;
  std::shared_ptr<serial::Serial> serialPtr_;
  bool    connectNmeaPort(std::string port, int desiredBaud = 9600);
  uint8_t computeNmeaChecksum(const char* sentence);

  //==================================================================
  //---------------------------LCD Display----------------------------
  //==================================================================

  integrity_toolkit::ToolkitLCDDisplay lcdDisplay_;
  std::shared_ptr<std::thread>         lcdThreadptr_;
  void               updateLcd(const std::future<void>& futureObj);
  int                checkCount_;
  std::promise<void> lcdExitSignal_;

  //==================================================================
  //---------------------------CSV Write----------------------------
  //==================================================================

  csv_write::MsgCsvWrite csvAssuranceReportWriter_;
  csv_write::MsgCsvWrite csvAssuranceReportsWriter_;
  int                    seqNum_;

  //==================================================================
  //------------------------------Ublox-------------------------------
  //==================================================================

  ublox::Ublox ublox1_;
  ublox::Ublox ublox2_;

  void bindUblox1Callbacks();
  void bindUblox2Callbacks();

  bool configureUblox1(const std::string& yamlFilename);
  bool configureUblox2(const std::string& yamlFilename);

  void handlePvtUblox1(const ublox::NavPvt&          pvt,
                       const ublox::SysClkTimePoint& timeStamp);
  long pvtUblox1SeqNum_;
  void publishPvtSerial(const ublox::NavPvt& pvt);
  void handlePvtUblox2(const ublox::NavPvt&          pvt,
                       const ublox::SysClkTimePoint& timeStamp);
  long pvtUblox2SeqNum_;

  void handleRawxMeasUblox1(const ublox::RawMeasX&        rawMeasX,
                            const ublox::SysClkTimePoint& timeStamp);
  long rawMeasXUblox1SeqNum_;
  void handleRawxMeasUblox2(const ublox::RawMeasX&        rawMeasX,
                            const ublox::SysClkTimePoint& timeStamp);
  long rawMeasXUblox2SeqNum_;

  void handleRxmSubframeXUblox1(const ublox::SubframeDataX& rawMeasX,
                            const ublox::SysClkTimePoint& timeStamp);
  long rxmSubframeUblox1SeqNum_;

  void handleRxmSubframeXUblox2(const ublox::SubframeDataX& rawMeasX,
                            const ublox::SysClkTimePoint& timeStamp);
  long rxmSubframeUblox2SeqNum_;

  std::pair<ublox::SysClkTimePoint, ublox::NavClock> curNavClockUblox1_;
  std::pair<ublox::SysClkTimePoint, ublox::NavGPSTime> curNavGpsTimeUblox1_;

  std::pair<ublox::SysClkTimePoint, ublox::NavClock>   curNavClockUblox2_;
  std::pair<ublox::SysClkTimePoint, ublox::NavGPSTime> curNavGpsTimeUblox2_;

  void handleNavClockUblox1(const ublox::NavClock&        clock,
                            const ublox::SysClkTimePoint& timeStamp);

  void handleNavClockUblox2(const ublox::NavClock&        clock,
                            const ublox::SysClkTimePoint& timeStamp);

  void handleNavGpsTimeUblox1(const ublox::NavGPSTime&      gpsTime,
                              const ublox::SysClkTimePoint& timeStamp);

  void handleNavGpsTimeUblox2(const ublox::NavGPSTime&      gpsTime,
                              const ublox::SysClkTimePoint& timeStamp);

  void handleMonRfUblox1(const ublox::MonRf&           monRf,
                         const ublox::SysClkTimePoint& timeStamp);
  long monRfUblox1SeqNum_;

  void checkForCompleteMsgsUblox1();
  void checkForCompleteMsgsUblox2();

  void buildHeader(const ublox::UbloxHeader&     ubxHeader,
                   const ublox::SysClkTimePoint& timeStamp,
                   pnt_integrity::data::Header&  header);

  void buildClockOffset(const ublox::NavGPSTime&          gpsTime,
                        const ublox::NavClock&            clock,
                        const ublox::SysClkTimePoint&     timeStamp,
                        pnt_integrity::data::ClockOffset& clockOffset);

  void buildPositionVelocity(const ublox::NavPvt&                   pvt,
                             const ublox::SysClkTimePoint&          timeStamp,
                             pnt_integrity::data::PositionVelocity& posVel);

  void buildGnssObservables(const ublox::RawMeasX&                rawMeasX,
                            const ublox::SysClkTimePoint&         timeStamp,
                            pnt_integrity::data::GNSSObservables& gnss);


  void buildGnssSubframe(const ublox::SubframeDataX& rxmSubframeX,
                            const ublox::SysClkTimePoint& timeStamp,
                            pnt_integrity::data::GNSSSubframe& gnssSubframe);

  pnt_integrity::data::CodeType getCodeType(const ublox::GnssId& gnssIde,
                                            const uint8_t&       sigId);

  pnt_integrity::data::SatelliteSystem getSatSystem(
    const ublox::GnssId& gnssIde);

  uint32_t getSvID(const uint8_t& svid);

  pnt_integrity::data::FrequencyBand getBand(const ublox::GnssId& gnssIde,
                                             const uint8_t&       sigId);

  //==================================================================
  //-----------------------Rx IF Data Strem (USRP)--------------------
  //==================================================================
  bool configureIfDataStream(const std::string& yamlFilename);

  int samplesPerMessage_;

  std::promise<void> ifDataExitSignal_;

  std::shared_ptr<std::thread> publishThreadPtr_;

  void liveStream(const std::future<void>&             futureObj,
                  const if_data_utils::IFSampleHeader& header);

  size_t handleIfDataFromStream(
    uhd::transport::bounded_buffer<usrp_utilities::circbuff_element_t>&,
    const usrp_utilities::SurrpasUsrpArgs&,
    const size_t&);

  double getTime(int64_t& sec, int32_t& nsec) noexcept
  {
    // Time since epoch in nanoseconds
    auto total_nanosec = std::chrono::steady_clock::now().time_since_epoch();
    sec                = total_nanosec.count() / 1000000000;
    nsec               = total_nanosec.count() % 1000000000;

    return (double)(sec) + ((double)(nsec) / 1e9);
  };  // get_sec_nsec()

  //==================================================================
  //----------------------------Integrity-----------------------------
  //==================================================================
  pnt_integrity::IntegrityMonitor integrityMonitor_;
  std::string                     rx1Name_;
  std::string                     rx2Name_;
  bool configureIntegrity(const std::string& yamlFilename);

  void publishAssurance(const pnt_integrity::data::PositionVelocity msg);

  //-------------------Range Position check--------------------------
  bool                                               useRngPosCheck_;
  std::string                                        rngPosCheckName_;
  std::shared_ptr<pnt_integrity::RangePositionCheck> rngPosCheckPtr_;
  void                                               initializeRangePosCheck();

  bool   useStaticMode_;
  double rx1rx2Baseline_;

  void publishRangePosCheckDiagnostics(
    const double&                                time,
    const pnt_integrity::RngPosCheckDiagnostics& checkData);

  //-------------------Static Position check--------------------------
  bool                                                useStaticPosCheck_;
  std::string                                         staticPosCheckName_;
  std::shared_ptr<pnt_integrity::StaticPositionCheck> staticPosCheckPtr_;
  void initializeStaticPosCheck();

  void publishStaticPosCheckDiagnostics(
    const double&                                   time,
    const pnt_integrity::StaticPosCheckDiagnostics& checkData);

  //---------------------------- Cno Check ---------------------------
  bool                                     useCnoCheck_;
  std::string                              cnoCheckName_;
  std::shared_ptr<pnt_integrity::CnoCheck> cnoCheckPtr_;
  void                                     initializeCnoCheck();

  void publishCnoCheckDiagnostics(
    const double&                             time,
    const pnt_integrity::CnoCheckDiagnostics& checkData);

  //---------------------- Position Jump Check -----------------------
  bool                                              usePosJumpCheck_;
  std::string                                       posJumpCheckName_;
  std::shared_ptr<pnt_integrity::PositionJumpCheck> posJumpCheckPtr_;
  void                                              initializePosJumpCheck();

  void publishPosJumpCheckDiagnostics(
    const double&                                 time,
    const pnt_integrity::PosJumpCheckDiagnostics& checkData);

  //--------------Position Velocity Consistency check-----------------
  bool        usePosVelConsCheck_;
  std::string posVelConsCheckName_;
  std::shared_ptr<pnt_integrity::PositionVelocityConsistencyCheck>
    posVelConsCheckPtr_;

  void initializePosVelConsCheck();

  void publishPvcCheckDiagnostics(
    const double&                                    time,
    const pnt_integrity::PosVelConsCheckDiagnostics& checkData);

  //---------------------------AOA check------------------------------
  bool                                                useAoaCheck_;
  std::string                                         aoaCheckName_;
  std::shared_ptr<pnt_integrity::AngleOfArrivalCheck> aoaCheckPtr_;
  void                                                initializeAoaCheck();

  void publishAoaCheckDiffs(const double&                       time,
                            const std::string&                  nodeID,
                            const pnt_integrity::SingleDiffMap& diffMap);

  void publishAoaCheckDiagnostics(
    const double&                             time,
    const pnt_integrity::AoaCheckDiagnostics& checkData);

  //---------------------------AGC check------------------------------
  bool                                     useAgcCheck_;
  std::string                              agcCheckName_;
  std::shared_ptr<pnt_integrity::AgcCheck> agcCheckPtr_;
  void                                     initializeAgcCheck();

  void publishAgcCheckDiagnostics(
    const double&                             time,
    const pnt_integrity::AgcCheckDiagnostics& checkData);

  //----------------------Clock Bias check----------------------------
  bool                                           useClockBiasCheck_;
  std::string                                    clockBiasCheckName_;
  std::shared_ptr<pnt_integrity::ClockBiasCheck> clockBiasCheckPtr_;
  void                                           initializeClockBiasCheck();

  void publishClockBiasCheckDiagnostics(
    const double&                                   time,
    const pnt_integrity::ClockBiasCheckDiagnostics& checkData);

  //---------------------- Navigation Data Check -----------------------
  bool useNavDataCheck_;
  std::string navDataCheckName_;
  std::shared_ptr<pnt_integrity::NavigationDataCheck> navDataCheckPtr_;
  void initializeNavDataCheck();

  void publishNavDataCheckDiagnostics(
    const double& time,
    const pnt_integrity::NavDataCheckDiagnostics& checkData);

  //----------------------Acquisition check----------------------------
  bool useAcquisitionCheck_;

  std::string acqCheckName_;
#ifdef PNT_INTEGRITY_INCLUDES_ACQ_CHECK
  std::shared_ptr<pnt_integrity::AcquisitionCheck> acqCheckPtr_;
  void publishAcqCheckPeakData(const double&                        time,
                               const pnt_integrity::PeakResultsMap& peakData);

  void publishAcqDiagnostics(const double&,
                             const pnt_integrity::AcqCheckDiagnostics&);

#endif
  void initializeAcquisitionCheck();
};

}  // namespace integrity_toolkit

#endif
