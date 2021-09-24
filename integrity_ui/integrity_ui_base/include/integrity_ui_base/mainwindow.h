//============================================================================//
//---------------------- integrity_ui_/mainwindow.h ------------*- C++ -*-----//
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
/// \brief    The integrity_ui library
/// \author   Josh Clanton <josh.clanton@is4s.com>
/// \date     April 13, 2020
//============================================================================//
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

#include <map>
#include <memory>
#include <mutex>
#include "integrity_ui_base/UpdatingQtChart.hpp"
#include "integrity_ui_base/mapform.hpp"
#include "integrity_ui_base/qcustomplot.h"
#include "pnt_integrity/IntegrityData.hpp"

// Defining draw in /usr/local/include/fastrtps/xmlparser/XMLParserCommon.h
// causes error in MapForm
#undef draw
namespace Ui
{
QT_BEGIN_NAMESPACE
class MainWindow;
}  // namespace Ui
QT_END_NAMESPACE

//==============================================================================
/// \brief An enumeration for diagnostic level
enum class LevelEnum
{
  DIAG_OK = 0,
  DIAG_WARN,
  DIAG_ERROR,
  DIAG_STALE
};

/// \brief A structure for key value pairs
struct KeyValue
{
  /// Label for the value
  std::string key;

  /// Value
  std::string value;
};

//==============================================================================

/// \brief A structure for general diagnostic messages
struct Diagnostics
{
  /// The message header
  pnt_integrity::data::Header header;

  /// Enumeration field to indicate operating level of source application or
  /// component
  LevelEnum level;

  /// The name of the diagnostic
  std::string name;

  /// Detailed description of message
  std::string message;

  /// An identifier for the source of the diagnostic (i.e. hardware serial
  /// number or name of the generating application)
  std::string hardwareId;

  /// The number of key/value pairs in the diagnostic message
  long numValues;

  /// A vector of id / value pairs
  std::vector<KeyValue> values;
};

//==============================================================================
/// \brief An enumeration for event log type
enum class EventLogType
{
  NotSet = 0,
  Debug,
  Info,
  Warning,
  Error,
  Critical
};

/// \brief Structure for event log messages
struct EventLog
{
  /// The message header
  pnt_integrity::data::Header header;

  /// The type of event log
  EventLogType eventLogType;

  /// The log message
  std::string eventLog;
};
//==============================================================================
/// \brief The command / response type enumeration
enum class CommandResponseType
{
  COMMAND = 0,
  RESPONSE
};

/// \brief A structure for command / response messages
struct CommandResponse
{
  /// The header associated with the command / response structure
  pnt_integrity::data::Header header;

  /// The device id of the command target (only used for responses)
  long deviceId;

  /// The identifier for the command
  long commandId;

  /// Tye type of message
  CommandResponseType type;

  /// The command or response message
  std::string message;
};

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

signals:
  void sendEventLog(const EventLog&);
  void sendCommandResponse(CommandResponse&);
  void sendCoordinates(double lat,
                       double lng,
                       int    idx,
                       int    colIdx,
                       double heading = 1000,
                       double error   = 0);
  void sendMapReset();

  void sendRcvrPV(const pnt_integrity::data::PositionVelocity&);
  void sendRcvr2PV(const pnt_integrity::data::PositionVelocity&);
  void sendRcvrGNSS(const pnt_integrity::data::GNSSObservables&);
  void sendRcvr2GNSS(const pnt_integrity::data::GNSSObservables&);
  void sendRcvrGpsTime(const int&, const double&);
  void sendRcvr2GpsTime(const int&, const double&);
  void sendIntegrityAssuranceReport(
    const pnt_integrity::data::AssuranceReport&);
  void sendIntegrityAssuranceReports(
    const pnt_integrity::data::AssuranceReports&);
  void sendDiagnostics(const Diagnostics&);

public slots:
  void chartPointAdded(int index);

private slots:

  void on_chkEventAutoScroll_clicked();
  void on_btnClearEventLog_clicked();
  void updateUi();

  // Toolbar actions
  void on_actionClear_Map_triggered();

  void handleRcvrPV(const pnt_integrity::data::PositionVelocity&);
  void handleRcvr2PV(const pnt_integrity::data::PositionVelocity&);
  void handleRcvrGNSS(const pnt_integrity::data::GNSSObservables&);
  void handleRcvr2GNSS(const pnt_integrity::data::GNSSObservables&);
  void handleRcvrGpsTime(const int&, const double&);
  void handleRcvr2GpsTime(const int&, const double&);

  void handleIntegrityAssuranceReport(
    const pnt_integrity::data::AssuranceReport&);
  void handleIntegrityAssuranceReports(
    const pnt_integrity::data::AssuranceReports&);

  void handleDiagnostics(const Diagnostics&);

  void handleEventLog(const EventLog&);
  void handleCommandResponse(const CommandResponse&);

private:
  void handlePvcCheckDiagnostics(const Diagnostics&);
  void handleClockBiasCheckDiagnostics(const Diagnostics&);
  void handleStaticPosCheckDiagnostics(const Diagnostics&);
  void handleAoaCheckDiffDiagnostics(const Diagnostics&);
  void handleAoaCheckDiagnostics(const Diagnostics&);
  void handleCnoCheckDiagnostics(const Diagnostics&);
  void handleAgcCheckDiagnostics(const Diagnostics&);
  void handleAcqCheckPeakDiagnostics(const Diagnostics&);
  void handleAcqCheckDiagnostics(const Diagnostics&);
  void handleRngPosCheckDiagnostics(const Diagnostics&);
  void handlePosJumpCheckDiagnostics(const Diagnostics&);
  void handleNavDataCheckDiagnostics(const Diagnostics&);

  const double RAD_2_DEG = 180 / M_PI;

  Ui::MainWindow* ui_;
  QTimer*         uiUpdateTimer_;

  MapForm* mapForm_;

  void addEventLog(QString event[4]);
  void addCommandResponse(QString event[6]);

  void addNavDataInfo(std::string infoStr);
  //-------------------------------C/N0 Charts----------------------------------
  static const QColor caColor_;
  static const QColor pyColor_;
  static const QColor mColor_;
  static const QColor galColor_;
  static const QColor otherColor_;

  // Carrier to noise plot data sets
  QCPBars* rcvrCaBars_;     // C/A code bars
  QCPBars* rcvrGalBars_;    // Galileo bars
  QCPBars* rcvrOtherBars_;  // Other bars

  QCPBars* rcvr2CaBars_;     // C/A code bars
  QCPBars* rcvr2GalBars_;    // Galileo bars
  QCPBars* rcvr2OtherBars_;  // Other bars

  //--------------------------Integrity Charts----------------------------------
  static const int integritySampleCount_ = 500;

  // -------Total charts---------
  std::shared_ptr<integrity_ui::UpdatingQtChart> totalLevelChartPtr_;
  std::shared_ptr<integrity_ui::UpdatingQtChart> allLevelsChartPtr_;

  void updateAllLevelsChart(const QString& checkName, const double& data);

  // -------Pos-Jump charts---------
  std::shared_ptr<integrity_ui::UpdatingQtChart> posJumpLevelChartPtr_;
  std::shared_ptr<integrity_ui::UpdatingQtChart> posJumpDiagChartPtr_;

  // -------Clock Jump charts---------
  std::shared_ptr<integrity_ui::UpdatingQtChart> clockJumpLevelChartPtr_;
  std::shared_ptr<integrity_ui::UpdatingQtChart> clockJumpErrChartPtr_;
  std::shared_ptr<integrity_ui::UpdatingQtChart> clockJumpDriftRateChartPtr_;

  // -------Cno charts---------
  std::shared_ptr<integrity_ui::UpdatingQtChart> cnoLevelChartPtr_;
  std::shared_ptr<integrity_ui::UpdatingQtChart> cnoDiagChartPtr_;

  // -------Static-Pos charts---------
  std::shared_ptr<integrity_ui::UpdatingQtChart> staticPosLevelChartPtr_;
  std::shared_ptr<integrity_ui::UpdatingQtChart> staticPosDiagChartPtr_;

  // -------AOA charts---------
  std::shared_ptr<integrity_ui::UpdatingQtChart> aoaLevelChartPtr_;
  std::shared_ptr<integrity_ui::UpdatingQtChart> aoaDiffChartPtr_;
  std::shared_ptr<integrity_ui::UpdatingQtChart> aoaDiagChartPtr_;

  // -------Rng-Pos charts---------
  std::shared_ptr<integrity_ui::UpdatingQtChart> rngPosChartPtr_;
  std::shared_ptr<integrity_ui::UpdatingQtChart> rngPosDiagChartPtr_;

  // -------AGC charts---------
  std::shared_ptr<integrity_ui::UpdatingQtChart> agcLevelChartPtr_;
  std::shared_ptr<integrity_ui::UpdatingQtChart> agcDiagChartPtr_;

  // -------Acq charts---------
  std::shared_ptr<integrity_ui::UpdatingQtChart> acqLevelChartPtr_;
  std::shared_ptr<integrity_ui::UpdatingQtChart> acqPeakRatioChartPtr_;
  std::shared_ptr<integrity_ui::UpdatingQtChart> acqCountChartPtr_;
  std::shared_ptr<integrity_ui::UpdatingQtChart> acqPeak1ChartPtr_;
  std::shared_ptr<integrity_ui::UpdatingQtChart> acqPeak2ChartPtr_;

  // -------PVC charts---------
  std::shared_ptr<integrity_ui::UpdatingQtChart> posVelLevelChartPtr_;
  std::shared_ptr<integrity_ui::UpdatingQtChart> posVelDataChartPtr_;

  // --------Nav Data Charts-----------
  std::shared_ptr<integrity_ui::UpdatingQtChart> navDataLevelChartPtr_;
  std::shared_ptr<integrity_ui::UpdatingQtChart> navDataValidChartPtr_;

  //---------------------------------General------------------------------------
  std::mutex uiUpdateMutex_;

  //---------------------------------Rcvr 1------------------------------------

  pnt_integrity::data::PositionVelocity rcvrPV_;
  int                                   rcvrGpsTimeWeek_;
  double                                rcvrGpsTimeSec_;
  pnt_integrity::data::GNSSObservables  rcvrGNSS_;

  //---------------------------------Rcvr2------------------------------------

  pnt_integrity::data::PositionVelocity rcvr2PV_;
  int                                   rcvr2GpsTimeWeek_;
  double                                rcvr2GpsTimeSec_;
  pnt_integrity::data::GNSSObservables  rcvr2GNSS_;

  //---------------------------Integrity Monitor--------------------------------
  pnt_integrity::data::AssuranceReport  integrityAssuranceReport_;
  pnt_integrity::data::AssuranceReports integrityAssuranceReports_;

  //-------------------------Discovery/General----------------------------------
};

#endif  // MAINWINDOW_H
