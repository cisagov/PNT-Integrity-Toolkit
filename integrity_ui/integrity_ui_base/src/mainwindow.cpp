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
//
// The integrity_ui library
// Josh Clanton <josh.clanton@is4s.com>
// April 13, 2020
//============================================================================//
#include "integrity_ui_base/mainwindow.h"
#include <iomanip>
#include <sstream>
#include "integrity_ui_base/UpdatingQtChart.hpp"
#include "ui_mainwindow.h"

#include "pnt_integrity/AcquisitionCheck.hpp"
#include "pnt_integrity/AgcCheck.hpp"
#include "pnt_integrity/AngleOfArrivalCheck.hpp"
#include "pnt_integrity/ClockBiasCheck.hpp"
#include "pnt_integrity/CnoCheck.hpp"
#include "pnt_integrity/NavigationDataCheck.hpp"
#include "pnt_integrity/PositionJumpCheck.hpp"
#include "pnt_integrity/PositionVelocityConsistencyCheck.hpp"
#include "pnt_integrity/RangePositionCheck.hpp"
#include "pnt_integrity/StaticPositionCheck.hpp"

//==============================================================================
//-------------------------------- UI Constatns --------------------------------
//==============================================================================

const QColor MainWindow::caColor_    = QColor("#FF6633");
const QColor MainWindow::pyColor_    = QColor("#1AB399");
const QColor MainWindow::mColor_     = QColor("#4DB3FF");
const QColor MainWindow::galColor_   = QColor("#FFFF99");
const QColor MainWindow::otherColor_ = QColor("#3366E6");

const std::string intTotalLevelStr = "total_level";

//-------clock bias check diagnostic plot strings ---------
const std::string intClockJumpCheckName      = "clock_bias_check";
const QString     clockJumpOffsetErrPlotStr  = "clock_bias_offset_err";
const QString     clockJumpDriftBoundPlotStr = "clock_bias_drift_bound";
const QString     clockJumpDriftRatePlotStr  = "clock_bias_exp_drift_rate";
const QString clockJumpDriftRateVarPlotStr = "clock_bias_drift_rate_var_bound";

//-------pos-vel consistency diagnostic plot strings ---------
const std::string intPosVelConsCheckName       = "pos_vel_cons_check";
const QString     pvcPercentBadPlotStr         = "pvc_percent_bad";
const QString     pvcInconsistentThreshPlotStr = "pvc_inconsistent_thresh";
const QString     pvcUnassuredThreshPlotStr    = "pvc_unassured_thresh";

const std::string intStaticPosCheckName        = "static_pos_check";
const QString     staticPosChangeThreshPlotStr = "static_pos_change_thresh";
const QString     staticPosPercentOverPlotStr  = "static_pos_percent_over";
const QString     staticPosIconsistentThreshPlotStr =
  "static_pos_inconsistent_thresh";
const QString staticPosUnassuredThreshPlotStr = "static_pos_unassered_thresh";

//--------aoa check diagnostics plot strings----------
const std::string intAoaCheckName             = "aoa_check";
const QString     aoaSingDiffThreshPosPlotStr = "aoa_sing_diff_thresh(pos)";
const QString     aoaSingDiffThreshNegPlotStr = "aoa_sing_diff_thresh(neg)";
const QString     aoaPrnCountPlotStr          = "aoa_suspect_prn_percent";
const QString     aoaIconsistentThreshPlotStr = "aoa_inconsistent_thresh";
const QString     aoaUnassuredThreshPlotStr   = "aoa_unassured_thresh";

//--------cno check diagnostics plot strings----------
const std::string intCN0CheckName             = "cno_check";
const QString     cnoAvgCountPlotStr          = "cno_avg_count";
const QString     cnoIconsistentThreshPlotStr = "cno_inconsistent_thresh";
const QString     cnoUnassuredThreshPlotStr   = "cno_unassured_thresh";

//--------agc check diagnostics plot strings----------
const std::string intAgcCheckName              = "agc_check";
const QString     agcInconsistentThreshPlotStr = "agc_inconsistent_thresh";

//--------acq check diagnostics plot strings----------
const std::string intAcqCheckName              = "acq_check";
const QString     acqInconsistentThreshPlotStr = "acq_inconsistent_thresh";
const QString     acqUnassuredThreshPlotStr    = "acq_unassured_thresh";
const QString     acqInconsistentCountPlotStr  = "acq_inconsistent_count";
const QString     acqUnassuredCountPlotStr     = "acq_unassured_count";
const QString     acqCountPlotStr              = "acq_thresh_count";
const QString     acqHiPwrThreshPlotStr        = "acq_hi_pwr_thresh";
const QString     acqPeakRatioThreshPlotStr    = "acq_peak_ratio_thresh";
const QString     acqThreshPlotStr             = "acq_thresh";

//--------rng-pos check diagnostics plot strings----------
const std::string intRngPosCheckName       = "rng_pos_check";
const QString     rngPosDiagMaxCalcPlotStr = "rp_max_calc_range";
const QString     rngPosDiagMinCalcPlotStr = "rp_min_calc_range";
const QString     rngPosDiagMaxMeasPlotStr = "rp_max_meas_range";
const QString     rngPosDiagMinMeasPlotStr = "rp_min_meas_range";

//--------pos-jump check diagnostics plot strings----------
const std::string intPosJumpCheckName     = "pos_jump_check";
const QString     posJumpDiagBoundPlotStr = "pos_jump_bound";
const QString     posJumpDiagDistPlotStr  = "pos_jump_distance";

//-------nav-data check diagnostics plot strings-----------
const std::string intNavDatacheckName          = "nav_data_check";
const QString     navDataCheckDataValidPlotStr = "nav_data_check_data_valid";
const QString     navDataCheckTowValidPlotStr  = "nav_data_check_tow_valid";
const QString     navDataCheckWnValidPlotStr   = "nav_data_check_wn_valid";

//--------map / track indeces--------
// using the same map / color index for these
const int rcvr1Idx     = 9;   // green #80B300
const int rcvr2Idx     = 1;   // blue #229EEE
const int surveyPosIdx = 15;  // neion yellow / green (tennis ball) #CCFF1A

//==============================================================================
//------------------------------ UI Constructor --------------------------------
//==============================================================================

MainWindow::MainWindow(QWidget* parent)
  : QMainWindow(parent), ui_(new Ui::MainWindow), uiUpdateTimer_(nullptr)
{
  Q_INIT_RESOURCE(resources);

  ui_->setupUi(this);

  // place the Map dock on top of the GNSS dock
  QMainWindow::tabifyDockWidget(ui_->dockGNSS, ui_->dockMap);

  ui_->actionEnable_Map->setChecked(true);
  mapForm_ = new MapForm(this, ui_->webView);

  qRegisterMetaType<QVector<int>>("QVector<int>");
  qRegisterMetaType<pnt_integrity::data::PositionVelocity>(
    "pnt_integrity::data::PositionVelocity");
  qRegisterMetaType<pnt_integrity::data::GNSSObservables>(
    "pnt_integrity::data::GNSSObservables");
  qRegisterMetaType<pnt_integrity::data::AssuranceReport>(
    "pnt_integrity::data::AssuranceReport");
  qRegisterMetaType<pnt_integrity::data::AssuranceReports>(
    "pnt_integrity::data::AssuranceReports");
  qRegisterMetaType<Diagnostics>("Diagnostics");
  qRegisterMetaType<EventLog>("EventLog");
  qRegisterMetaType<CommandResponse>("CommandResponse");

  connect(this,
          SIGNAL(sendCoordinates(double, double, int, int, double, double)),
          mapForm_,
          SLOT(receiveCoordinates(double, double, int, int, double, double)));

  connect(this, SIGNAL(sendMapReset()), mapForm_, SLOT(resetMarkers()));

  uiUpdateTimer_ = new QTimer(this);
  connect(uiUpdateTimer_, SIGNAL(timeout()), this, SLOT(updateUi()));
  uiUpdateTimer_->start(1000);

  connect(this,
          SIGNAL(sendRcvrPV(const pnt_integrity::data::PositionVelocity&)),
          this,
          SLOT(handleRcvrPV(const pnt_integrity::data::PositionVelocity&)));

  connect(this,
          SIGNAL(sendRcvr2PV(const pnt_integrity::data::PositionVelocity&)),
          this,
          SLOT(handleRcvr2PV(const pnt_integrity::data::PositionVelocity&)));

  connect(this,
          SIGNAL(sendRcvrGNSS(const pnt_integrity::data::GNSSObservables&)),
          this,
          SLOT(handleRcvrGNSS(const pnt_integrity::data::GNSSObservables&)));

  connect(this,
          SIGNAL(sendRcvr2GNSS(const pnt_integrity::data::GNSSObservables&)),
          this,
          SLOT(handleRcvr2GNSS(const pnt_integrity::data::GNSSObservables&)));

  connect(this,
          SIGNAL(sendRcvrGpsTime(const int&, const double&)),
          this,
          SLOT(handleRcvrGpsTime(const int&, const double&)));

  connect(this,
          SIGNAL(sendRcvr2GpsTime(const int&, const double&)),
          this,
          SLOT(handleRcvr2GpsTime(const int&, const double&)));

  connect(this,
          SIGNAL(sendIntegrityAssuranceReport(
            const pnt_integrity::data::AssuranceReport&)),
          this,
          SLOT(handleIntegrityAssuranceReport(
            const pnt_integrity::data::AssuranceReport&)));

  connect(this,
          SIGNAL(sendIntegrityAssuranceReports(
            const pnt_integrity::data::AssuranceReports&)),
          this,
          SLOT(handleIntegrityAssuranceReports(
            const pnt_integrity::data::AssuranceReports&)));

  connect(this,
          SIGNAL(sendDiagnostics(const Diagnostics&)),
          this,
          SLOT(handleDiagnostics(const Diagnostics&)));

  connect(this,
          SIGNAL(sendEventLog(const EventLog&)),
          this,
          SLOT(handleEventLog(const EventLog&)));

  connect(this,
          SIGNAL(sendCommandResponse(const CommandResponse&)),
          this,
          SLOT(handleCommandResponse(const CommandResponse&)));

  //---------------------Setup Integrity Charts---------------------------------

  totalLevelChartPtr_ = std::make_shared<integrity_ui::UpdatingQtChart>(
    ui_->chartIntTotal,
    QString("Total"),
    (size_t)integritySampleCount_,
    false,
    false,
    0,
    4);

  totalLevelChartPtr_->addSeries(QString::fromStdString(intTotalLevelStr));

  allLevelsChartPtr_ = std::make_shared<integrity_ui::UpdatingQtChart>(
    ui_->chartIntAll,
    QString("All Levels"),
    (size_t)integritySampleCount_,
    false,
    false,
    0,
    4);

  //--------Setup Pos-Jump Charts-------
  posJumpLevelChartPtr_ = std::make_shared<integrity_ui::UpdatingQtChart>(
    ui_->chartIntPosJump,
    QString("Pos-Jump"),
    (size_t)integritySampleCount_,
    false,
    false,
    0,
    4);

  posJumpLevelChartPtr_->addSeries(QString::fromStdString(intPosJumpCheckName));

  posJumpDiagChartPtr_ = std::make_shared<integrity_ui::UpdatingQtChart>(
    ui_->chartIntPosJumpDiag,
    QString("Pos-Jump diagnostics"),
    (size_t)integritySampleCount_,
    true,
    true);

  posJumpDiagChartPtr_->addSeries(posJumpDiagBoundPlotStr);
  posJumpDiagChartPtr_->addSeries(posJumpDiagDistPlotStr);

  //--------Setup Clock-Jump Charts-------
  clockJumpLevelChartPtr_ = std::make_shared<integrity_ui::UpdatingQtChart>(
    ui_->chartIntClockJump,
    QString("Clock-Jump"),
    (size_t)integritySampleCount_,
    false,
    false,
    0,
    4);

  clockJumpLevelChartPtr_->addSeries(
    QString::fromStdString(intClockJumpCheckName));

  clockJumpErrChartPtr_ = std::make_shared<integrity_ui::UpdatingQtChart>(
    ui_->chartIntClockJumpOffset,
    QString("Offset"),
    (size_t)integritySampleCount_,
    true,
    true);

  clockJumpErrChartPtr_->addSeries(clockJumpOffsetErrPlotStr);
  clockJumpErrChartPtr_->addSeries(clockJumpDriftBoundPlotStr);

  clockJumpDriftRateChartPtr_ = std::make_shared<integrity_ui::UpdatingQtChart>(
    ui_->chartIntClockJumpDriftRate,
    QString("Drift Rate / Var"),
    (size_t)integritySampleCount_,
    true,
    true);

  clockJumpDriftRateChartPtr_->addSeries(clockJumpDriftRatePlotStr);
  clockJumpDriftRateChartPtr_->addSeries(clockJumpDriftRateVarPlotStr);

  //--------Setup Cno Charts-------
  cnoLevelChartPtr_ = std::make_shared<integrity_ui::UpdatingQtChart>(
    ui_->chartIntCN0,
    QString("CN0"),
    (size_t)integritySampleCount_,
    false,
    false,
    0,
    4);

  cnoLevelChartPtr_->addSeries(QString::fromStdString(intCN0CheckName));

  cnoDiagChartPtr_ = std::make_shared<integrity_ui::UpdatingQtChart>(
    ui_->chartIntCN0diag,
    QString("CN0 diagnostics"),
    (size_t)integritySampleCount_,
    true,
    true);

  cnoDiagChartPtr_->addSeries(cnoAvgCountPlotStr);
  cnoDiagChartPtr_->addSeries(cnoIconsistentThreshPlotStr);
  cnoDiagChartPtr_->addSeries(cnoUnassuredThreshPlotStr);

  //--------Setup Static-Pos Charts-------
  staticPosLevelChartPtr_ = std::make_shared<integrity_ui::UpdatingQtChart>(
    ui_->chartIntStaticPos,
    QString("Static-Pos"),
    (size_t)integritySampleCount_,
    false,
    false,
    0,
    4);

  staticPosLevelChartPtr_->addSeries(
    QString::fromStdString(intStaticPosCheckName));

  staticPosDiagChartPtr_ = std::make_shared<integrity_ui::UpdatingQtChart>(
    ui_->chartIntStaticPosDiag,
    QString("Static-Pos Diag"),
    (size_t)integritySampleCount_,
    true,
    true);

  staticPosDiagChartPtr_->addSeries(staticPosPercentOverPlotStr);
  staticPosDiagChartPtr_->addSeries(staticPosIconsistentThreshPlotStr);
  staticPosDiagChartPtr_->addSeries(staticPosUnassuredThreshPlotStr);

  //--------Setup AOA Charts-------
  aoaLevelChartPtr_ = std::make_shared<integrity_ui::UpdatingQtChart>(
    ui_->chartIntAOA,
    QString("AOA"),
    (size_t)integritySampleCount_,
    false,
    false,
    0,
    4);

  aoaLevelChartPtr_->addSeries(QString::fromStdString(intAoaCheckName));

  aoaDiffChartPtr_ = std::make_shared<integrity_ui::UpdatingQtChart>(
    ui_->chartIntAOAdiff,
    QString("AOA Diffs"),
    (size_t)integritySampleCount_,
    true,
    true);

  aoaDiffChartPtr_->addSeries(aoaSingDiffThreshPosPlotStr);
  aoaDiffChartPtr_->addSeries(aoaSingDiffThreshNegPlotStr);

  aoaDiagChartPtr_ = std::make_shared<integrity_ui::UpdatingQtChart>(
    ui_->chartIntAOAdiag,
    QString("AOA Diagnostics"),
    (size_t)integritySampleCount_,
    true,
    true);

  aoaDiagChartPtr_->addSeries(aoaPrnCountPlotStr);
  aoaDiagChartPtr_->addSeries(aoaIconsistentThreshPlotStr);
  aoaDiagChartPtr_->addSeries(aoaUnassuredThreshPlotStr);

  //--------Setup Rang-Pos Charts-------
  rngPosChartPtr_ = std::make_shared<integrity_ui::UpdatingQtChart>(
    ui_->chartIntRngPos,
    QString("Rng-Pos"),
    (size_t)integritySampleCount_,
    false,
    false,
    0,
    4);

  rngPosChartPtr_->addSeries(QString::fromStdString(intRngPosCheckName));

  rngPosDiagChartPtr_ = std::make_shared<integrity_ui::UpdatingQtChart>(
    ui_->chartIntRngPosDiag,
    QString("Rng-Pos diagnostics"),
    (size_t)integritySampleCount_,
    true,
    true);

  rngPosDiagChartPtr_->addSeries(rngPosDiagMaxCalcPlotStr);
  rngPosDiagChartPtr_->addSeries(rngPosDiagMinCalcPlotStr);
  rngPosDiagChartPtr_->addSeries(rngPosDiagMaxMeasPlotStr);
  rngPosDiagChartPtr_->addSeries(rngPosDiagMinMeasPlotStr);

  //--------Setup AGC Charts-------
  agcLevelChartPtr_ = std::make_shared<integrity_ui::UpdatingQtChart>(
    ui_->chartIntAGC,
    QString("AGC"),
    (size_t)integritySampleCount_,
    false,
    false,
    0,
    4);

  agcLevelChartPtr_->addSeries(QString::fromStdString(intAgcCheckName));

  agcDiagChartPtr_ = std::make_shared<integrity_ui::UpdatingQtChart>(
    ui_->chartIntAGCdiag,
    QString("AGC diagnostics"),
    (size_t)integritySampleCount_,
    true,
    true);

  agcDiagChartPtr_->addSeries(agcInconsistentThreshPlotStr);

  //--------Setup ACQ Charts-------
  acqLevelChartPtr_ = std::make_shared<integrity_ui::UpdatingQtChart>(
    ui_->chartIntAcq,
    QString("Acq"),
    (size_t)integritySampleCount_,
    false,
    false,
    0,
    4);

  acqLevelChartPtr_->addSeries(QString::fromStdString(intAcqCheckName));

  acqPeakRatioChartPtr_ = std::make_shared<integrity_ui::UpdatingQtChart>(
    ui_->chartIntAcqPeakRatio,
    QString("Acq Peak Ratio"),
    (size_t)integritySampleCount_,
    true,
    true);

  acqPeakRatioChartPtr_->addSeries(acqPeakRatioThreshPlotStr);

  acqCountChartPtr_ = std::make_shared<integrity_ui::UpdatingQtChart>(
    ui_->chartIntAcqCount,
    QString("Acq Peak Ratio"),
    (size_t)integritySampleCount_,
    true,
    true);

  acqCountChartPtr_->addSeries(acqInconsistentThreshPlotStr);
  acqCountChartPtr_->addSeries(acqUnassuredThreshPlotStr);
  acqCountChartPtr_->addSeries(acqInconsistentCountPlotStr);
  acqCountChartPtr_->addSeries(acqUnassuredCountPlotStr);

  acqPeak1ChartPtr_ = std::make_shared<integrity_ui::UpdatingQtChart>(
    ui_->chartIntAcqPeak1,
    QString("Acq Peak 1"),
    (size_t)integritySampleCount_,
    true,
    true);

  acqPeak1ChartPtr_->addSeries(acqHiPwrThreshPlotStr);

  acqPeak2ChartPtr_ = std::make_shared<integrity_ui::UpdatingQtChart>(
    ui_->chartIntAcqPeak2,
    QString("Acq Peak 2"),
    (size_t)integritySampleCount_,
    true,
    true);

  acqPeak2ChartPtr_->addSeries(acqThreshPlotStr);

  //--------Setup PVC Charts-------
  posVelLevelChartPtr_ = std::make_shared<integrity_ui::UpdatingQtChart>(
    ui_->chartIntPosVelCons,
    QString("Pos-Vel Cons"),
    (size_t)integritySampleCount_,
    false,
    false,
    0,
    4);

  posVelLevelChartPtr_->addSeries(
    QString::fromStdString(intPosVelConsCheckName));

  posVelDataChartPtr_ = std::make_shared<integrity_ui::UpdatingQtChart>(
    ui_->chartIntPosVelCons_Data,
    QString("PVC Data"),
    (size_t)integritySampleCount_,
    true,
    true);

  posVelDataChartPtr_->addSeries(pvcPercentBadPlotStr);
  posVelDataChartPtr_->addSeries(pvcInconsistentThreshPlotStr);
  posVelDataChartPtr_->addSeries(pvcUnassuredThreshPlotStr);

  //--------Setup Nav Data Charts-------
  navDataLevelChartPtr_ = std::make_shared<integrity_ui::UpdatingQtChart>(
    ui_->chartIntNavData,
    QString("Nav Data"),
    (size_t)integritySampleCount_,
    false,
    false,
    0,
    4);

  navDataLevelChartPtr_->addSeries(QString::fromStdString(intNavDatacheckName));

  navDataValidChartPtr_ = std::make_shared<integrity_ui::UpdatingQtChart>(
    ui_->chartIntNavDataValid,
    QString("Nav Data Check"),
    (size_t)integritySampleCount_,
    true,
    true);

  navDataValidChartPtr_->addSeries(navDataCheckDataValidPlotStr);
  navDataValidChartPtr_->addSeries(navDataCheckTowValidPlotStr);
  navDataValidChartPtr_->addSeries(navDataCheckWnValidPlotStr);

  //-------------------------Setup C/N0 Plots-----------------------------------
  rcvrCaBars_ = new QCPBars(ui_->plotRcvrCN0->xAxis, ui_->plotRcvrCN0->yAxis);
  rcvrCaBars_->setAntialiased(false);
  rcvrCaBars_->setName("C/NO");
  rcvrCaBars_->setPen(QPen(caColor_));
  rcvrCaBars_->setBrush(caColor_);

  rcvrGalBars_ = new QCPBars(ui_->plotRcvrCN0->xAxis, ui_->plotRcvrCN0->yAxis);
  rcvrGalBars_->setAntialiased(false);
  rcvrGalBars_->setPen(QPen(galColor_));
  rcvrGalBars_->setBrush(galColor_);

  rcvrOtherBars_ =
    new QCPBars(ui_->plotRcvrCN0->xAxis, ui_->plotRcvrCN0->yAxis);
  rcvrOtherBars_->setAntialiased(false);
  rcvrOtherBars_->setPen(QPen(otherColor_));
  rcvrOtherBars_->setBrush(otherColor_);

  rcvr2CaBars_ =
    new QCPBars(ui_->plotRcvr2CN0->xAxis, ui_->plotRcvr2CN0->yAxis);
  rcvr2CaBars_->setAntialiased(false);
  rcvr2CaBars_->setName("C/NO");
  rcvr2CaBars_->setPen(QPen(caColor_));
  rcvr2CaBars_->setBrush(caColor_);

  rcvr2GalBars_ =
    new QCPBars(ui_->plotRcvrCN0->xAxis, ui_->plotRcvr2CN0->yAxis);
  rcvr2GalBars_->setAntialiased(false);
  rcvr2GalBars_->setPen(QPen(galColor_));
  rcvr2GalBars_->setBrush(galColor_);

  rcvr2OtherBars_ =
    new QCPBars(ui_->plotRcvr2CN0->xAxis, ui_->plotRcvr2CN0->yAxis);
  rcvr2OtherBars_->setAntialiased(false);
  rcvr2OtherBars_->setPen(QPen(otherColor_));
  rcvr2OtherBars_->setBrush(otherColor_);

  // set dark background gradient:
  QLinearGradient gradient(0, 0, 0, 400);
  gradient.setColorAt(0, QColor(90, 90, 90));
  gradient.setColorAt(0.38, QColor(105, 105, 105));
  gradient.setColorAt(1, QColor(70, 70, 70));
  ui_->plotRcvr2CN0->setBackground(QBrush(gradient));
  ui_->plotRcvrCN0->setBackground(QBrush(gradient));
}

//----------------------------------------------------------------------------//

MainWindow::~MainWindow()
{
  delete ui_;
}

//==============================================================================
//-------------------------------- Update Funcs -------------------------------
//==============================================================================

void MainWindow::chartPointAdded(int index)
{
  std::cout << "idx: " << index << std::endl;
}

//----------------------------------------------------------------------------//

void MainWindow::addEventLog(QString event[4])
{
  QTreeWidgetItem* treeItem = new QTreeWidgetItem(ui_->treeEventLog);
  treeItem->setText(0, event[0]);
  treeItem->setText(1, event[1]);
  treeItem->setText(2, event[2]);
  treeItem->setText(3, event[3]);
}

//----------------------------------------------------------------------------//

void MainWindow::addCommandResponse(QString event[6])
{
  QTreeWidgetItem* treeItem = new QTreeWidgetItem(ui_->treeCommandResponse);
  treeItem->setText(0, event[0]);
  treeItem->setText(1, event[1]);
  treeItem->setText(2, event[2]);
  treeItem->setText(3, event[3]);
  treeItem->setText(4, event[4]);
  treeItem->setText(5, event[5]);
}

//----------------------------------------------------------------------------//

void MainWindow::on_chkEventAutoScroll_clicked()
{
  ui_->treeEventLog->setAutoScroll(ui_->chkEventAutoScroll->isChecked());
}

//----------------------------------------------------------------------------//

void MainWindow::on_btnClearEventLog_clicked()
{
  ui_->treeEventLog->clear();
}

//----------------------------------------------------------------------------//

void MainWindow::updateUi()
{
  std::lock_guard<std::mutex> guard(uiUpdateMutex_);

  //============================================================================
  ui_->txtRcvrLat->setText(
    QString::number(rcvrPV_.position.latitude * RAD_2_DEG, 'f', 6));
  ui_->txtRcvrLon->setText(
    QString::number(rcvrPV_.position.longitude * RAD_2_DEG, 'f', 6));
  ui_->txtRcvrAlt->setText(QString::number(rcvrPV_.position.altitude));
  ui_->txtRcvrVn->setText(QString::number(rcvrPV_.velocity[0]));
  ui_->txtRcvrVe->setText(QString::number(rcvrPV_.velocity[1]));
  ui_->txtRcvrVd->setText(QString::number(rcvrPV_.velocity[2]));
  ui_->txtRcvrNErr->setText(QString::number(sqrt(rcvrPV_.covariance[0][0])));
  ui_->txtRcvrEErr->setText(QString::number(sqrt(rcvrPV_.covariance[1][1])));
  ui_->txtRcvrDErr->setText(QString::number(sqrt(rcvrPV_.covariance[2][2])));
  ui_->txtRcvrVnErr->setText(QString::number(sqrt(rcvrPV_.covariance[3][3])));
  ui_->txtRcvrVeErr->setText(QString::number(sqrt(rcvrPV_.covariance[4][4])));
  ui_->txtRcvrVdErr->setText(QString::number(sqrt(rcvrPV_.covariance[5][5])));
  ui_->txtRcvrWeek->setText(QString::number(rcvrGpsTimeWeek_));
  ui_->txtRcvrSec->setText(QString::number(rcvrGpsTimeSec_));

  //============================================================================
  ui_->txtRcvr2Lat->setText(
    QString::number(rcvr2PV_.position.latitude * RAD_2_DEG, 'f', 6));
  ui_->txtRcvr2Lon->setText(
    QString::number(rcvr2PV_.position.longitude * RAD_2_DEG, 'f', 6));
  ui_->txtRcvr2Alt->setText(QString::number(rcvr2PV_.position.altitude));
  ui_->txtRcvr2Vn->setText(QString::number(rcvr2PV_.velocity[0]));
  ui_->txtRcvr2Ve->setText(QString::number(rcvr2PV_.velocity[1]));
  ui_->txtRcvr2Vd->setText(QString::number(rcvr2PV_.velocity[2]));
  ui_->txtRcvr2NErr->setText(QString::number(sqrt(rcvr2PV_.covariance[0][0])));
  ui_->txtRcvr2EErr->setText(QString::number(sqrt(rcvr2PV_.covariance[1][1])));
  ui_->txtRcvr2DErr->setText(QString::number(sqrt(rcvr2PV_.covariance[2][2])));
  ui_->txtRcvr2VnErr->setText(QString::number(sqrt(rcvr2PV_.covariance[3][3])));
  ui_->txtRcvr2VeErr->setText(QString::number(sqrt(rcvr2PV_.covariance[4][4])));
  ui_->txtRcvr2VdErr->setText(QString::number(sqrt(rcvr2PV_.covariance[5][5])));
  ui_->txtRcvr2Week->setText(QString::number(rcvr2GpsTimeWeek_));
  ui_->txtRcvr2Sec->setText(QString::number(rcvr2GpsTimeSec_));

  //============================================================================
  pnt_integrity::data::GNSSObservableMap rcvrObservables =
    rcvrGNSS_.observables;

  ui_->treeRcvrGNSS->clear();
  QVector<double>  rcvrCaBarData;
  QVector<double>  rcvrGalBarData;
  QVector<double>  rcvrOtherBarData;
  QVector<double>  rcvrChannels;
  QVector<QString> rcvrLabels;
  double           rcvrChanNum = 1;
  for (auto obs : rcvrObservables)
  {
    QTreeWidgetItem* treeItem = new QTreeWidgetItem(ui_->treeRcvrGNSS);
    treeItem->setText(0, QString::number(obs.second.prn));
    treeItem->setText(1, QString::number((int)obs.second.satelliteType));
    treeItem->setText(2, QString::number((int)obs.second.frequencyType));
    treeItem->setText(3, QString::number((int)obs.second.codeType));
    treeItem->setText(4, QString::number(obs.second.carrierToNoise));
    treeItem->setText(5, QString::number(obs.second.pseudorange));
    treeItem->setText(6, QString::number(obs.second.carrierPhase));
    treeItem->setText(7, QString::number(obs.second.doppler));

    // std::cout << "Chn: " << rcvrChanNum << " Prn: " << obs.second.prn
    //           << " CNO: " << obs.second.carrierToNoise << " psr: " <<
    //           obs.second.pseudorange
    //           << " code: " << (int)obs.second.codeType
    //           << " Freq: " << (int)obs.second.frequencyType
    //           << " sys: " << (int)obs.second.satelliteType << std::endl;
    if (obs.second.carrierToNoise > 0)
    {
      rcvrChannels.push_back(rcvrChanNum);
      rcvrLabels.push_back(QString::number(rcvrChanNum++));

      if ((obs.second.codeType == pnt_integrity::data::CodeType::SigC) &&
          (obs.second.satelliteType ==
           pnt_integrity::data::SatelliteSystem::GPS))
      {
        rcvrCaBarData.push_back(obs.second.carrierToNoise);
        rcvrGalBarData.push_back(0);
        rcvrOtherBarData.push_back(0);
      }
      else if ((obs.second.codeType == pnt_integrity::data::CodeType::SigC) &&
               (obs.second.satelliteType ==
                pnt_integrity::data::SatelliteSystem::Galileo))
      {
        rcvrCaBarData.push_back(0);
        rcvrGalBarData.push_back(obs.second.carrierToNoise);
        rcvrOtherBarData.push_back(0);
      }
      else
      {
        rcvrCaBarData.push_back(0);
        rcvrGalBarData.push_back(0);
        rcvrOtherBarData.push_back(obs.second.carrierToNoise);
      }
    }
  }

  // TODO: Move most of this to on creation
  ui_->plotRcvrCN0->xAxis->setAutoTicks(false);
  ui_->plotRcvrCN0->xAxis->setAutoTickLabels(false);
  ui_->plotRcvrCN0->xAxis->setLabel("Channel #");
  ui_->plotRcvrCN0->xAxis->setRange(0, rcvrChanNum);
  ui_->plotRcvrCN0->xAxis->setTickVector(rcvrChannels);
  ui_->plotRcvrCN0->xAxis->setTickVectorLabels(rcvrLabels);
  ui_->plotRcvrCN0->xAxis->setSubTickCount(0);
  ui_->plotRcvrCN0->xAxis->setBasePen(QPen(Qt::white));
  ui_->plotRcvrCN0->xAxis->setTickPen(QPen(Qt::white));
  ui_->plotRcvrCN0->xAxis->grid()->setVisible(false);
  ui_->plotRcvrCN0->xAxis->grid()->setPen(
    QPen(QColor(130, 130, 130), 0, Qt::DotLine));
  ui_->plotRcvrCN0->xAxis->setTickLabelColor(Qt::white);
  ui_->plotRcvrCN0->xAxis->setLabelColor(Qt::white);
  // prepare y axis:
  ui_->plotRcvrCN0->yAxis->setRange(0, 65);
  ui_->plotRcvrCN0->yAxis->setPadding(5);
  ui_->plotRcvrCN0->yAxis->setBasePen(QPen(Qt::white));
  ui_->plotRcvrCN0->yAxis->setTickPen(QPen(Qt::white));
  ui_->plotRcvrCN0->yAxis->setSubTickPen(QPen(Qt::white));
  ui_->plotRcvrCN0->yAxis->grid()->setSubGridVisible(true);
  ui_->plotRcvrCN0->yAxis->setTickLabelColor(Qt::white);
  ui_->plotRcvrCN0->yAxis->setLabelColor(Qt::white);
  ui_->plotRcvrCN0->yAxis->grid()->setPen(
    QPen(QColor(130, 130, 130), 0, Qt::SolidLine));
  ui_->plotRcvrCN0->yAxis->grid()->setSubGridPen(
    QPen(QColor(130, 130, 130), 0, Qt::DotLine));

  rcvrCaBars_->setData(rcvrChannels, rcvrCaBarData);
  rcvrGalBars_->setData(rcvrChannels, rcvrGalBarData);
  rcvrOtherBars_->setData(rcvrChannels, rcvrOtherBarData);
  ui_->plotRcvrCN0->replot();

  //============================================================================
  pnt_integrity::data::GNSSObservableMap rcvr2Observables =
    rcvr2GNSS_.observables;

  ui_->treeRcvr2GNSS->clear();
  QVector<double>  rcvr2CaBarData;
  QVector<double>  rcvr2GalBarData;
  QVector<double>  rcvr2OtherBarData;
  QVector<double>  rcvr2Channels;
  QVector<QString> rcvr2Labels;

  double rcvr2ChanNum = 1;
  for (auto obs : rcvr2Observables)
  {
    QTreeWidgetItem* treeItem = new QTreeWidgetItem(ui_->treeRcvr2GNSS);
    treeItem->setText(0, QString::number(obs.second.prn));
    treeItem->setText(1, QString::number((int)obs.second.satelliteType));
    treeItem->setText(2, QString::number((int)obs.second.frequencyType));
    treeItem->setText(3, QString::number((int)obs.second.codeType));
    treeItem->setText(4, QString::number(obs.second.carrierToNoise));
    treeItem->setText(5, QString::number(obs.second.pseudorange));
    treeItem->setText(6, QString::number(obs.second.carrierPhase));
    treeItem->setText(7, QString::number(obs.second.doppler));

    // std::cout << "Chn: " << rcvrChanNum << " Prn: " << obs.second.prn
    //           << " CNO: " << obs.second.carrierToNoise << " psr: " <<
    //           obs.second.pseudorange
    //           << " code: " << (int)obs.second.codeType
    //           << " Freq: " << (int)obs.second.frequencyType
    //           << " sys: " << (int)obs.second.satelliteType << std::endl;

    if (obs.second.carrierToNoise > 0)
    {
      rcvr2Channels.push_back(rcvr2ChanNum);
      rcvr2Labels.push_back(QString::number(rcvr2ChanNum++));

      if ((obs.second.codeType == pnt_integrity::data::CodeType::SigC) &&
          (obs.second.satelliteType ==
           pnt_integrity::data::SatelliteSystem::GPS))
      {
        rcvr2CaBarData.push_back(obs.second.carrierToNoise);
        rcvr2GalBarData.push_back(0);
        rcvr2OtherBarData.push_back(0);
      }
      else if ((obs.second.codeType == pnt_integrity::data::CodeType::SigC) &&
               (obs.second.satelliteType ==
                pnt_integrity::data::SatelliteSystem::Galileo))
      {
        rcvr2CaBarData.push_back(0);
        rcvr2GalBarData.push_back(obs.second.carrierToNoise);
        rcvr2OtherBarData.push_back(0);
      }
      else
      {
        rcvr2CaBarData.push_back(0);
        rcvr2GalBarData.push_back(0);
        rcvr2OtherBarData.push_back(obs.second.carrierToNoise);
      }
    }
  }

  // TODO: Move most of this to on creation
  ui_->plotRcvr2CN0->xAxis->setAutoTicks(false);
  ui_->plotRcvr2CN0->xAxis->setAutoTickLabels(false);
  ui_->plotRcvr2CN0->xAxis->setLabel("Channel #");
  ui_->plotRcvr2CN0->xAxis->setRange(0, rcvrChanNum);
  ui_->plotRcvr2CN0->xAxis->setTickVector(rcvrChannels);
  ui_->plotRcvr2CN0->xAxis->setTickVectorLabels(rcvrLabels);
  ui_->plotRcvr2CN0->xAxis->setSubTickCount(0);
  ui_->plotRcvr2CN0->xAxis->setBasePen(QPen(Qt::white));
  ui_->plotRcvr2CN0->xAxis->setTickPen(QPen(Qt::white));
  ui_->plotRcvr2CN0->xAxis->grid()->setVisible(false);
  ui_->plotRcvr2CN0->xAxis->grid()->setPen(
    QPen(QColor(130, 130, 130), 0, Qt::DotLine));
  ui_->plotRcvr2CN0->xAxis->setTickLabelColor(Qt::white);
  ui_->plotRcvr2CN0->xAxis->setLabelColor(Qt::white);
  // prepare y axis:
  ui_->plotRcvr2CN0->yAxis->setRange(0, 65);
  ui_->plotRcvr2CN0->yAxis->setPadding(5);
  ui_->plotRcvr2CN0->yAxis->setBasePen(QPen(Qt::white));
  ui_->plotRcvr2CN0->yAxis->setTickPen(QPen(Qt::white));
  ui_->plotRcvr2CN0->yAxis->setSubTickPen(QPen(Qt::white));
  ui_->plotRcvr2CN0->yAxis->grid()->setSubGridVisible(true);
  ui_->plotRcvr2CN0->yAxis->setTickLabelColor(Qt::white);
  ui_->plotRcvr2CN0->yAxis->setLabelColor(Qt::white);
  ui_->plotRcvr2CN0->yAxis->grid()->setPen(
    QPen(QColor(130, 130, 130), 0, Qt::SolidLine));
  ui_->plotRcvr2CN0->yAxis->grid()->setSubGridPen(
    QPen(QColor(130, 130, 130), 0, Qt::DotLine));

  rcvr2CaBars_->setData(rcvr2Channels, rcvr2CaBarData);
  rcvr2GalBars_->setData(rcvr2Channels, rcvr2GalBarData);
  rcvr2OtherBars_->setData(rcvr2Channels, rcvr2OtherBarData);
  ui_->plotRcvr2CN0->replot();

  //============================================================================
  // Integrity Charts
  totalLevelChartPtr_->updateAll();
  allLevelsChartPtr_->updateAll();
  posJumpLevelChartPtr_->updateAll();
  posJumpDiagChartPtr_->updateAll();
  clockJumpLevelChartPtr_->updateAll();
  clockJumpErrChartPtr_->updateAll();
  clockJumpDriftRateChartPtr_->updateAll();
  cnoLevelChartPtr_->updateAll();
  cnoDiagChartPtr_->updateAll();
  staticPosLevelChartPtr_->updateAll();
  staticPosDiagChartPtr_->updateAll();
  aoaLevelChartPtr_->updateAll();
  aoaDiffChartPtr_->updateAll();
  aoaDiagChartPtr_->updateAll();
  rngPosChartPtr_->updateAll();
  rngPosDiagChartPtr_->updateAll();
  agcLevelChartPtr_->updateAll();
  agcDiagChartPtr_->updateAll();
  acqLevelChartPtr_->updateAll();
  acqPeak1ChartPtr_->updateAll();
  acqPeak2ChartPtr_->updateAll();
  acqPeakRatioChartPtr_->updateAll();
  acqCountChartPtr_->updateAll();
  posVelLevelChartPtr_->updateAll();
  posVelDataChartPtr_->updateAll();
  navDataLevelChartPtr_->updateAll();
  navDataValidChartPtr_->updateAll();

  ui_->txtAssuranceTimecode->setText(
    QString::number(integrityAssuranceReport_.header.timestampValid.timecode));
  ui_->txtAssuranceTime->setText(
    QString::number(integrityAssuranceReport_.header.timestampValid.sec));
  ui_->txtAssuranceLevel->setText(QString::number(
    integrityAssuranceReport_.state.getIntegerAssuranceValue()));
  ui_->txtAssuranceNumChecks->setText(
    QString::number(integrityAssuranceReports_.numStates));

  for (auto state : integrityAssuranceReports_.states)
  {
    if (state.getName() == intPosJumpCheckName)
    {
      ui_->txtIntPosJumpLevel->setText(
        QString::number(state.getIntegerAssuranceValue()));
      ui_->spinIntPosJumpWeight->setValue(state.getWeight());
    }
    else if (state.getName() == intClockJumpCheckName)
    {
      ui_->txtIntClockJumpLevel->setText(
        QString::number(state.getIntegerAssuranceValue()));
      ui_->spinIntPosJumpWeight->setValue(state.getWeight());
    }
    else if (state.getName() == intCN0CheckName)
    {
      ui_->txtIntCN0Level->setText(
        QString::number(state.getIntegerAssuranceValue()));
      ui_->spinIntCN0Weight->setValue(state.getWeight());
    }
    else if (state.getName() == intPosVelConsCheckName)
    {
      ui_->txtIntPosVelConsLevel->setText(
        QString::number(state.getIntegerAssuranceValue()));
      ui_->spinIntPosVelConsWeight->setValue(state.getWeight());
    }
    else if (state.getName() == intStaticPosCheckName)
    {
      ui_->txtIntStaticPosLevel->setText(
        QString::number(state.getIntegerAssuranceValue()));
      ui_->spinIntStaticPosWeight->setValue(state.getWeight());
    }
    else if (state.getName() == intAoaCheckName)
    {
      ui_->txtIntAOALevel->setText(
        QString::number(state.getIntegerAssuranceValue()));
      ui_->spinIntAOAWeight->setValue(state.getWeight());
    }
    else if (state.getName() == intRngPosCheckName)
    {
      ui_->txtIntRngPosLevel->setText(
        QString::number(state.getIntegerAssuranceValue()));
      ui_->spinIntRngPosWeight->setValue(state.getWeight());
    }
    else if (state.getName() == intAgcCheckName)
    {
      ui_->txtIntAGCLevel->setText(
        QString::number(state.getIntegerAssuranceValue()));
      ui_->spinIntAGCWeight->setValue(state.getWeight());
    }
    else if (state.getName() == intAcqCheckName)
    {
      ui_->txtIntAcqLevel->setText(
        QString::number(state.getIntegerAssuranceValue()));
      ui_->spinIntAcqWeight->setValue(state.getWeight());
    }
    else if (state.getName() == intNavDatacheckName)
    {
      ui_->txtIntNavDataLevel->setText(
        QString::number(state.getIntegerAssuranceValue()));
      ui_->spinIntNavDataWeight->setValue(state.getWeight());
    }
  }
}

//----------------------------------------------------------------------------//

void MainWindow::on_actionClear_Map_triggered()
{
  std::cout << "Clear Map." << std::endl;
  emit sendMapReset();
}

//==============================================================================
//--------------------------- Message Handlers----------------------------------
//==============================================================================

//----------------------------------------------------------------------------//

void MainWindow::handleEventLog(const EventLog& log)
{
  QString event[4];
  event[0] = QString::number(log.header.timestampValid.sec);
  switch (log.eventLogType)
  {
    case EventLogType::NotSet:
      event[1] = "NOT SET";
      break;
    case EventLogType::Debug:
      event[1] = "DEBUG";
      break;
    case EventLogType::Info:
      event[1] = "INFO";
      break;
    case EventLogType::Warning:
      event[1] = "WARNING";
      break;
    case EventLogType::Error:
      event[1] = "ERROR";
      break;
    case EventLogType::Critical:
      event[1] = "CRITICAL";
      break;
  }
  event[2] = QString::fromStdString(log.eventLog);
  event[3] = QString::fromStdString(log.header.deviceId);
  addEventLog(event);
}

//----------------------------------------------------------------------------//

void MainWindow::handleCommandResponse(const CommandResponse& cmd)
{
  QString event[6];
  event[0] = QString::number(cmd.header.timestampValid.sec);
  event[1] = QString::fromStdString(cmd.header.deviceId);
  event[2] = QString::number(cmd.deviceId);
  switch (cmd.type)
  {
    case CommandResponseType::COMMAND:
      event[3] = "COMMAND";
      break;
    case CommandResponseType::RESPONSE:
      event[3] = "RESPONSE";
      break;
  }
  event[4] = QString::number(cmd.commandId);
  event[5] = QString::fromStdString(cmd.message);

  addCommandResponse(event);
}

//----------------------------------------------------------------------------//
void MainWindow::handleRcvrPV(const pnt_integrity::data::PositionVelocity& pv)
{
  std::lock_guard<std::mutex> guard(uiUpdateMutex_);

  if ((rcvrPV_.position.latitude != 0) && (rcvrPV_.position.longitude != 0))
  {
    if (ui_->actionEnable_Map->isChecked())
    {
      emit sendCoordinates(pv.position.latitude * RAD_2_DEG,
                           pv.position.longitude * RAD_2_DEG,
                           rcvr1Idx,
                           rcvr1Idx);
    }
  }
  rcvrPV_ = pv;
}
//----------------------------------------------------------------------------//

void MainWindow::handleRcvrGpsTime(const int& week, const double& tow)
{
  std::lock_guard<std::mutex> guard(uiUpdateMutex_);
  rcvrGpsTimeWeek_ = week;
  rcvrGpsTimeSec_  = tow;
}

//----------------------------------------------------------------------------//

void MainWindow::handleRcvrGNSS(
  const pnt_integrity::data::GNSSObservables& gnss)
{
  std::lock_guard<std::mutex> guard(uiUpdateMutex_);
  rcvrGNSS_ = gnss;
}

//----------------------------------------------------------------------------//
void MainWindow::handleRcvr2PV(const pnt_integrity::data::PositionVelocity& pv)
{
  std::lock_guard<std::mutex> guard(uiUpdateMutex_);

  if ((rcvr2PV_.position.latitude != 0) && (rcvr2PV_.position.longitude != 0))
  {
    if (ui_->actionEnable_Map->isChecked())
    {
      emit sendCoordinates(pv.position.latitude * RAD_2_DEG,
                           pv.position.longitude * RAD_2_DEG,
                           rcvr2Idx,
                           rcvr2Idx);
    }
  }
  rcvr2PV_ = pv;
}

//----------------------------------------------------------------------------//

void MainWindow::handleRcvr2GNSS(
  const pnt_integrity::data::GNSSObservables& gnss)
{
  std::lock_guard<std::mutex> guard(uiUpdateMutex_);
  rcvr2GNSS_ = gnss;
}

//----------------------------------------------------------------------------//

void MainWindow::handleRcvr2GpsTime(const int& week, const double& tow)
{
  std::lock_guard<std::mutex> guard(uiUpdateMutex_);
  rcvr2GpsTimeWeek_ = week;
  rcvr2GpsTimeSec_  = tow;
}

//==============================================================================
//------------------------------ Diagnostics-----------------------------------
//==============================================================================
void MainWindow::handleDiagnostics(const Diagnostics& msg)
{
  //----------- Pos-velocity consistency check diagnostics ------------
  if (msg.name == pnt_integrity::INTEGRITY_PVC_DIAGNOSTICS)
  {
    handlePvcCheckDiagnostics(msg);
  }
  //----------- Clock bias check diagnostics ------------
  else if (msg.name == pnt_integrity::INTEGRITY_CLOCK_BIAS_DIAGNOSTICS)
  {
    handleClockBiasCheckDiagnostics(msg);
  }
  //----------- Static Position check diagnostics ------------
  else if (msg.name == pnt_integrity::INTEGRITY_STATIC_POS_DIAGNOSTICS)
  {
    handleStaticPosCheckDiagnostics(msg);
  }
  //----------- AOA check diff diagnostics ------------
  else if (msg.name == pnt_integrity::INTEGRITY_AOA_DIFF_DIAGNOSTICS)
  {
    handleAoaCheckDiffDiagnostics(msg);
  }
  //----------- AOA check  diagnostics ------------
  else if (msg.name == pnt_integrity::INTEGRITY_AOA_DIAGNOSTICS)
  {
    handleAoaCheckDiagnostics(msg);
  }
  //----------- CN0 check  diagnostics ------------
  else if (msg.name == pnt_integrity::INTEGRITY_CN0_DIAGNOSTICS)
  {
    handleCnoCheckDiagnostics(msg);
  }
  //----------- AGC check  diagnostics ------------
  else if (msg.name == pnt_integrity::INTEGRITY_AGC_DIAGNOSTICS)
  {
    handleAgcCheckDiagnostics(msg);
  }
  //----------- ACQ check  peak data ------------
  else if (msg.name == pnt_integrity::INTEGRITY_ACQ_PEAK_VALS)
  {
    handleAcqCheckPeakDiagnostics(msg);
  }
  //----------- ACQ check  diagnostics ------------
  else if (msg.name == pnt_integrity::INTEGRITY_ACQ_DIAGNOSTICS)
  {
    handleAcqCheckDiagnostics(msg);
  }
  //-----------Rng-Pos Check Diagnostics---------------
  else if (msg.name == pnt_integrity::INTEGRITY_RNG_POS_DIAGNOSTICS)
  {
    handleRngPosCheckDiagnostics(msg);
  }
  //-----------Pos-Jump Check Diagnostics--------------
  else if (msg.name == pnt_integrity::INTEGRITY_POS_JUMP_DIAGNOSTICS)
  {
    handlePosJumpCheckDiagnostics(msg);
  }
  //-----------Nav Data Check Diagnostics--------------
  else if (msg.name == pnt_integrity::INTEGRITY_NAV_DATA_DIAGNOSTICS)
  {
    handleNavDataCheckDiagnostics(msg);
  }
}

//----------------------------------------------------------------------------//

void MainWindow::handlePvcCheckDiagnostics(const Diagnostics& msg)
{
  for (auto valIt = msg.values.begin(); valIt != msg.values.end(); ++valIt)
  {
    if (valIt->key == pnt_integrity::INTEGRITY_PVC_DIAG_PB)
    {
      posVelDataChartPtr_->addData(pvcPercentBadPlotStr,
                                   std::stod(valIt->value));
    }
    else if (valIt->key == pnt_integrity::INTEGRITY_PVC_DIAG_ITHRESH)
    {
      posVelDataChartPtr_->addData(pvcInconsistentThreshPlotStr,
                                   std::stod(valIt->value));
    }
    else if (valIt->key == pnt_integrity::INTEGRITY_PVC_DIAG_UTHRESH)
    {
      posVelDataChartPtr_->addData(pvcUnassuredThreshPlotStr,
                                   std::stod(valIt->value));
    }
  }
}

//----------------------------------------------------------------------------//
void MainWindow::handleNavDataCheckDiagnostics(const Diagnostics& msg)
{
  for (auto valIt = msg.values.begin(); valIt != msg.values.end(); ++valIt)
  {
    if (valIt->key == pnt_integrity::INTEGRITY_NAV_DATA_VALID)
    {
      navDataValidChartPtr_->addData(navDataCheckDataValidPlotStr,
                                     std::stod(valIt->value));
    }
    else if (valIt->key == pnt_integrity::INTEGRITY_NAV_DATA_TOW_VALID)
    {
      navDataValidChartPtr_->addData(navDataCheckTowValidPlotStr,
                                     std::stod(valIt->value));
    }
    else if (valIt->key == pnt_integrity::INTEGRITY_NAV_DATA_WN_VALID)
    {
      navDataValidChartPtr_->addData(navDataCheckWnValidPlotStr,
                                     std::stod(valIt->value));
    }
    else if (valIt->key == pnt_integrity::INTEGRITY_NAV_DATA_VALID_MSG)
    {
      if (valIt->value != "")
      {
        addNavDataInfo(valIt->value);
      }
    }
    else if (valIt->key == pnt_integrity::INTEGRITY_NAV_DATA_TOW_VALID_MSG)
    {
      if (valIt->value != "")
      {
        addNavDataInfo(valIt->value);
      }
    }
    else if (valIt->key == pnt_integrity::INTEGRITY_NAV_DATA_WN_VALID_MSG)
    {
      if (valIt->value != "")
      {
        addNavDataInfo(valIt->value);
      }
    }
  }
}

void MainWindow::addNavDataInfo(std::string infoStr)
{
  auto current_time = std::chrono::system_clock::now();
  auto duration_in_seconds =
    std::chrono::duration<double>(current_time.time_since_epoch());

  std::stringstream newTimestampStr;
  newTimestampStr << std::setprecision(15) << duration_in_seconds.count();

  QTreeWidgetItem* treeItem = new QTreeWidgetItem(ui_->treeNavDataInfo);
  treeItem->setText(0, QString::fromStdString(newTimestampStr.str()));
  treeItem->setText(1, QString::fromStdString(infoStr));
}

//----------------------------------------------------------------------------//
void MainWindow::handleClockBiasCheckDiagnostics(const Diagnostics& msg)
{
  for (auto valIt = msg.values.begin(); valIt != msg.values.end(); ++valIt)
  {
    if (valIt->key == pnt_integrity::INTEGRITY_CLOCK_BIAS_DIAG_OFFSET_ERROR)
    {
      clockJumpErrChartPtr_->addData(clockJumpOffsetErrPlotStr,
                                     std::stod(valIt->value));
    }
    else if (valIt->key ==
             pnt_integrity::INTEGRITY_CLOCK_BIAS_DIAG_DRIFT_RATE_BOUND)
    {
      clockJumpErrChartPtr_->addData(clockJumpDriftBoundPlotStr,
                                     std::stod(valIt->value));
    }
    else if (valIt->key ==
             pnt_integrity::INTEGRITY_CLOCK_BIAS_DIAG_EXP_DRIFT_VAR)
    {
      clockJumpDriftRateChartPtr_->addData(clockJumpDriftRatePlotStr,
                                           std::stod(valIt->value));
    }
    else if (valIt->key ==
             pnt_integrity::INTEGRITY_CLOCK_BIAS_DIAG_DRIFT_RATE_VAR_BOUND)
    {
      clockJumpDriftRateChartPtr_->addData(clockJumpDriftRateVarPlotStr,
                                           std::stod(valIt->value));
    }
  }
}

//----------------------------------------------------------------------------//

void MainWindow::handleStaticPosCheckDiagnostics(const Diagnostics& msg)
{
  bool   gotLat = false;
  bool   gotLon = false;
  bool   gotAlt = false;
  bool   gotErr = false;
  double lat    = 0.0;
  double lon    = 0.0;
  double err    = 0.0;

  for (auto valIt = msg.values.begin(); valIt != msg.values.end(); ++valIt)
  {
    if (valIt->key == pnt_integrity::INTEGRITY_STAIC_POS_DIAG_POS_LAT)
    {
      gotLat = true;
      lat    = std::stod(valIt->value);
    }
    else if (valIt->key == pnt_integrity::INTEGRITY_STAIC_POS_DIAG_POS_LON)
    {
      gotLon = true;
      lon    = std::stod(valIt->value);
    }
    else if (valIt->key == pnt_integrity::INTEGRITY_STAIC_POS_DIAG_POS_ALT)
    {
      gotAlt = true;
    }
    else if (valIt->key ==
             pnt_integrity::INTEGRITY_STAIC_POS_DIAG_POS_CHNG_THRESH)
    {
      gotErr = true;
      err    = std::stod(valIt->value);
    }
    else if (valIt->key == pnt_integrity::INTEGRITY_STAIC_POS_DIAG_PERCENT_OVER)
    {
      staticPosDiagChartPtr_->addData(staticPosPercentOverPlotStr,
                                      std::stod(valIt->value));
    }
    else if (valIt->key == pnt_integrity::INTEGRITY_STAIC_POS_DIAG_ITHRESH)
    {
      staticPosDiagChartPtr_->addData(staticPosIconsistentThreshPlotStr,
                                      std::stod(valIt->value));
    }
    else if (valIt->key == pnt_integrity::INTEGRITY_STAIC_POS_DIAG_UTHRESH)
    {
      staticPosDiagChartPtr_->addData(staticPosUnassuredThreshPlotStr,
                                      std::stod(valIt->value));
    }
  }
  if (gotLat && gotLon && gotAlt && gotErr)
  {
    emit sendCoordinates(
      lat * RAD_2_DEG, lon * RAD_2_DEG, surveyPosIdx, surveyPosIdx, 9999, err);
  }
}

//----------------------------------------------------------------------------//

void MainWindow::handleAoaCheckDiffDiagnostics(const Diagnostics& msg)
{
  std::string           nodeID = "";  // not currently used
  std::map<int, double> diffMap;
  for (auto valIt = msg.values.begin(); valIt != msg.values.end(); ++valIt)
  {
    if (valIt->key == pnt_integrity::INTEGRITY_AOA_DIFF_NODE_ID)
    {
      nodeID = valIt->value;
    }
    else
    {
      diffMap[std::stoi(valIt->key)] = std::stod(valIt->value);
    }
  }

  for (auto diffIt = diffMap.begin(); diffIt != diffMap.end(); ++diffIt)
  {
    // check with chart to see if series exists
    std::stringstream seriesName;
    seriesName << nodeID << "_" << diffIt->first;
    if (!aoaDiffChartPtr_->hasSeries(seriesName.str()))
    {
      aoaDiffChartPtr_->addSeries(seriesName.str());
    }

    aoaDiffChartPtr_->addData(seriesName.str(), diffIt->second);
  }
}

//----------------------------------------------------------------------------//

void MainWindow::handleAoaCheckDiagnostics(const Diagnostics& msg)
{
  for (auto valIt = msg.values.begin(); valIt != msg.values.end(); ++valIt)
  {
    if (valIt->key == pnt_integrity::INTEGRITY_AOA_DIAG_DIFF_THRESH)
    {
      // the diff thresh should be a bound around the plotted double diffs
      double boundVal = std::stod(valIt->value);
      aoaDiffChartPtr_->addData(aoaSingDiffThreshPosPlotStr, 0.5 * boundVal);
      aoaDiffChartPtr_->addData(aoaSingDiffThreshNegPlotStr, -0.5 * boundVal);
    }
    else if (valIt->key ==
             pnt_integrity::INTEGRITY_AOA_DIAG_SUSPECT_PRN_PERCENT)
    {
      aoaDiagChartPtr_->addData(aoaPrnCountPlotStr, std::stod(valIt->value));
    }
    else if (valIt->key == pnt_integrity::INTEGRITY_AOA_DIAG_ITHRESH)
    {
      aoaDiagChartPtr_->addData(aoaIconsistentThreshPlotStr,
                                std::stod(valIt->value));
    }
    else if (valIt->key == pnt_integrity::INTEGRITY_AOA_DIAG_UTHRESH)
    {
      aoaDiagChartPtr_->addData(aoaUnassuredThreshPlotStr,
                                std::stod(valIt->value));
    }
  }
}

//----------------------------------------------------------------------------//

void MainWindow::handleCnoCheckDiagnostics(const Diagnostics& msg)
{
  for (auto valIt = msg.values.begin(); valIt != msg.values.end(); ++valIt)
  {
    if (valIt->key == pnt_integrity::INTEGRITY_CN0_DIAG_AVG_COUNT)
    {
      cnoDiagChartPtr_->addData(cnoAvgCountPlotStr, std::stod(valIt->value));
    }
    else if (valIt->key == pnt_integrity::INTEGRITY_CN0_DIAG_ITHRESH)
    {
      cnoDiagChartPtr_->addData(cnoIconsistentThreshPlotStr,
                                std::stod(valIt->value));
    }
    else if (valIt->key == pnt_integrity::INTEGRITY_CN0_DIAG_UTHRESH)
    {
      cnoDiagChartPtr_->addData(cnoUnassuredThreshPlotStr,
                                std::stod(valIt->value));
    }
  }
}

//----------------------------------------------------------------------------//

void MainWindow::handleAgcCheckDiagnostics(const Diagnostics& msg)
{
  std::map<std::string, double> valMap;
  for (auto valIt = msg.values.begin(); valIt != msg.values.end(); ++valIt)
  {
    if (valIt->key == pnt_integrity::INTEGRITY_AGC_DIAG_ITHRESH)
    {
      agcDiagChartPtr_->addData(agcInconsistentThreshPlotStr,
                                std::stod(valIt->value));
    }
    else
    {
      valMap[valIt->key] = std::stod(valIt->value);
    }
  }

  for (auto valIt = valMap.begin(); valIt != valMap.end(); ++valIt)
  {
    // check with chart to see if series exists
    if (!agcDiagChartPtr_->hasSeries(valIt->first))
    {
      agcDiagChartPtr_->addSeries(valIt->first);
    }
    agcDiagChartPtr_->addData(valIt->first, valIt->second);
  }
}

//----------------------------------------------------------------------------//

void MainWindow::handleAcqCheckPeakDiagnostics(const Diagnostics& msg)
{
  std::map<std::string, double> peak1Map;
  std::map<std::string, double> peak2Map;
  for (auto valIt = msg.values.begin(); valIt != msg.values.end(); ++valIt)
  {
    std::size_t peak1Found =
      valIt->key.find(pnt_integrity::INTEGRITY_ACQ_PEAK1_KEY);
    std::size_t peak2Found =
      valIt->key.find(pnt_integrity::INTEGRITY_ACQ_PEAK2_KEY);
    if (peak1Found != std::string::npos)
    {
      // this is a peak 1 key
      std::string key =
        valIt->key.substr(pnt_integrity::INTEGRITY_ACQ_PEAK1_KEY.length());
      peak1Map[key] = std::stod(valIt->value);
    }
    else if (peak2Found != std::string::npos)
    {
      // this is a peak 1 key
      std::string key =
        valIt->key.substr(pnt_integrity::INTEGRITY_ACQ_PEAK2_KEY.length());
      peak2Map[key] = std::stod(valIt->value);
    }
  }
  // plot the peak1 values
  for (auto valIt = peak1Map.begin(); valIt != peak1Map.end(); ++valIt)
  {
    // check with chart to see if series exists
    if (!acqPeak1ChartPtr_->hasSeries(valIt->first))
    {
      acqPeak1ChartPtr_->addSeries(valIt->first);
    }
    acqPeak1ChartPtr_->addData(valIt->first, valIt->second);
  }
  // plot the peak2 values
  for (auto valIt = peak2Map.begin(); valIt != peak2Map.end(); ++valIt)
  {
    // check with chart to see if series exists
    if (!acqPeak2ChartPtr_->hasSeries(valIt->first))
    {
      acqPeak2ChartPtr_->addSeries(valIt->first);
    }
    acqPeak2ChartPtr_->addData(valIt->first, valIt->second);
  }
}

//----------------------------------------------------------------------------//

void MainWindow::handleAcqCheckDiagnostics(const Diagnostics& msg)
{
  std::map<std::string, double> ratioMap;
  for (auto valIt = msg.values.begin(); valIt != msg.values.end(); ++valIt)
  {
    std::size_t ratioFound =
      valIt->key.find(pnt_integrity::INT_ACQ_DIAG_PEAK_RATIO_KEY);
    if (ratioFound != std::string::npos)
    {
      // tihs is a peak ratio
      // remove the key for readable legend strings
      std::string key =
        valIt->key.substr(pnt_integrity::INT_ACQ_DIAG_PEAK_RATIO_KEY.length());
      ratioMap[key] = std::stod(valIt->value);
    }
    else if (valIt->key == pnt_integrity::INT_ACQ_DIAG_HI_PWR_THRESH)
    {
      // add the hi power threshold to the peak1 chart
      acqPeak1ChartPtr_->addData(acqHiPwrThreshPlotStr,
                                 std::stod(valIt->value));
    }
    else if (valIt->key == pnt_integrity::INT_ACQ_DIAG_PEAK_RATIO_THRESH)
    {
      // add the peak ratio threshold to the peak ratio chart
      acqPeakRatioChartPtr_->addData(acqPeakRatioThreshPlotStr,
                                     std::stod(valIt->value));
    }
    else if (valIt->key == pnt_integrity::INT_ACQ_DIAG_ACQ_THRESH)
    {
      // add the acquisition thresh to the peak2 chart
      acqPeak2ChartPtr_->addData(acqThreshPlotStr, std::stod(valIt->value));
    }
    else if (valIt->key == pnt_integrity::INT_ACQ_DIAG_ITHRESH)
    {
      // add the inconsistent thresh to the count chart
      acqCountChartPtr_->addData(acqInconsistentThreshPlotStr,
                                 std::stod(valIt->value));
    }
    else if (valIt->key == pnt_integrity::INT_ACQ_DIAG_UTHRESH)
    {
      // add the unassured thresh to the count chart
      acqCountChartPtr_->addData(acqUnassuredThreshPlotStr,
                                 std::stod(valIt->value));
    }
    else if (valIt->key == pnt_integrity::INT_ACQ_DIAG_ICOUNT)
    {
      // add the unassured thresh to the count chart
      acqCountChartPtr_->addData(acqInconsistentCountPlotStr,
                                 std::stod(valIt->value));
    }
    else if (valIt->key == pnt_integrity::INT_ACQ_DIAG_UCOUNT)
    {
      // add the unassured thresh to the count chart
      acqCountChartPtr_->addData(acqUnassuredCountPlotStr,
                                 std::stod(valIt->value));
    }
  }
  for (auto valIt = ratioMap.begin(); valIt != ratioMap.end(); ++valIt)
  {
    // check with chart to see if series exists
    if (!acqPeakRatioChartPtr_->hasSeries(valIt->first))
    {
      acqPeakRatioChartPtr_->addSeries(valIt->first);
    }
    acqPeakRatioChartPtr_->addData(valIt->first, valIt->second);
  }
}

//----------------------------------------------------------------------------//

void MainWindow::handleRngPosCheckDiagnostics(const Diagnostics& msg)
{
  for (auto valIt = msg.values.begin(); valIt != msg.values.end(); ++valIt)
  {
    if (valIt->key.find(pnt_integrity::INTEGRITY_RNG_POS_DIAG_MAX_CALC) !=
        std::string::npos)
    {
      rngPosDiagChartPtr_->addData(rngPosDiagMaxCalcPlotStr,
                                   std::stod(valIt->value));
    }
    else if (valIt->key.find(pnt_integrity::INTEGRITY_RNG_POS_DIAG_MIN_CALC) !=
             std::string::npos)
    {
      rngPosDiagChartPtr_->addData(rngPosDiagMinCalcPlotStr,
                                   std::stod(valIt->value));
    }
    else if (valIt->key.find(pnt_integrity::INTEGRITY_RNG_POS_DIAG_MAX_MEAS) !=
             std::string::npos)
    {
      rngPosDiagChartPtr_->addData(rngPosDiagMaxMeasPlotStr,
                                   std::stod(valIt->value));
    }
    else if (valIt->key.find(pnt_integrity::INTEGRITY_RNG_POS_DIAG_MIN_MEAS) !=
             std::string::npos)
    {
      rngPosDiagChartPtr_->addData(rngPosDiagMinMeasPlotStr,
                                   std::stod(valIt->value));
    }
  }
}

//----------------------------------------------------------------------------//

void MainWindow::handlePosJumpCheckDiagnostics(const Diagnostics& msg)
{
  for (auto valIt = msg.values.begin(); valIt != msg.values.end(); ++valIt)
  {
    if (valIt->key.find(pnt_integrity::INTEGRITY_POS_JUMP_DIAG_BOUND) !=
        std::string::npos)
    {
      posJumpDiagChartPtr_->addData(posJumpDiagBoundPlotStr,
                                    std::stod(valIt->value));
    }
    else if (valIt->key.find(pnt_integrity::INTEGRITY_POS_JUMP_DIAG_DIST) !=
             std::string::npos)
    {
      posJumpDiagChartPtr_->addData(posJumpDiagDistPlotStr,
                                    std::stod(valIt->value));
    }
  }
}

//==============================================================================
//------------------------------ Assurance Levels----------------------------
//==============================================================================

void MainWindow::handleIntegrityAssuranceReport(
  const pnt_integrity::data::AssuranceReport& report)
{
  std::lock_guard<std::mutex> guard(uiUpdateMutex_);
  integrityAssuranceReport_ = report;
  totalLevelChartPtr_->addData(QString::fromStdString(intTotalLevelStr),
                               report.state.getIntegerAssuranceValue());
}

//----------------------------------------------------------------------------//

void MainWindow::updateAllLevelsChart(const QString& checkName,
                                      const double&  data)
{
  if (!allLevelsChartPtr_->hasSeries(checkName))
  {
    allLevelsChartPtr_->addSeries(checkName);
  }
  allLevelsChartPtr_->addData(checkName, data);
}

// //----------------------------------------------------------------------------//

void MainWindow::handleIntegrityAssuranceReports(
  const pnt_integrity::data::AssuranceReports& reports)
{
  std::lock_guard<std::mutex> guard(uiUpdateMutex_);
  integrityAssuranceReports_ = reports;

  for (auto state : reports.states)
  {
    if (state.getName() == intPosJumpCheckName)
    {
      QString checkName = QString::fromStdString(intPosJumpCheckName);
      posJumpLevelChartPtr_->addData(checkName,
                                     state.getIntegerAssuranceValue());
      updateAllLevelsChart(checkName, state.getIntegerAssuranceValue());
    }
    else if (state.getName() == intClockJumpCheckName)
    {
      QString checkName = QString::fromStdString(intClockJumpCheckName);
      clockJumpLevelChartPtr_->addData(checkName,
                                       state.getIntegerAssuranceValue());
      updateAllLevelsChart(checkName, state.getIntegerAssuranceValue());
    }
    else if (state.getName() == intCN0CheckName)
    {
      QString checkName = QString::fromStdString(intCN0CheckName);
      cnoLevelChartPtr_->addData(checkName, state.getIntegerAssuranceValue());
      updateAllLevelsChart(checkName, state.getIntegerAssuranceValue());
    }
    else if (state.getName() == intPosVelConsCheckName)
    {
      QString checkName = QString::fromStdString(intPosVelConsCheckName);
      posVelLevelChartPtr_->addData(checkName,
                                    state.getIntegerAssuranceValue());
      updateAllLevelsChart(checkName, state.getIntegerAssuranceValue());
    }
    else if (state.getName() == intStaticPosCheckName)
    {
      QString checkName = QString::fromStdString(intStaticPosCheckName);
      staticPosLevelChartPtr_->addData(checkName,
                                       state.getIntegerAssuranceValue());
      updateAllLevelsChart(checkName, state.getIntegerAssuranceValue());
    }
    else if (state.getName() == intAoaCheckName)
    {
      QString checkName = QString::fromStdString(intAoaCheckName);
      aoaLevelChartPtr_->addData(checkName, state.getIntegerAssuranceValue());
      updateAllLevelsChart(checkName, state.getIntegerAssuranceValue());
    }
    else if (state.getName() == intRngPosCheckName)
    {
      QString checkName = QString::fromStdString(intRngPosCheckName);
      rngPosChartPtr_->addData(checkName, state.getIntegerAssuranceValue());
      updateAllLevelsChart(checkName, state.getIntegerAssuranceValue());
    }
    else if (state.getName() == intAgcCheckName)
    {
      QString checkName = QString::fromStdString(intAgcCheckName);
      agcLevelChartPtr_->addData(checkName, state.getIntegerAssuranceValue());
      updateAllLevelsChart(checkName, state.getIntegerAssuranceValue());
    }
    else if (state.getName() == intAcqCheckName)
    {
      QString checkName = QString::fromStdString(intAcqCheckName);
      acqLevelChartPtr_->addData(checkName, state.getIntegerAssuranceValue());
      updateAllLevelsChart(checkName, state.getIntegerAssuranceValue());
    }
    else if (state.getName() == intNavDatacheckName)
    {
      QString checkName = QString::fromStdString(intNavDatacheckName);
      navDataLevelChartPtr_->addData(checkName,
                                     state.getIntegerAssuranceValue());
      updateAllLevelsChart(checkName, state.getIntegerAssuranceValue());
    }
    else
    {
      std::cout << "Received assurance report with unknown check name: "
                << state.getName() << std::endl;
    }
  }
}