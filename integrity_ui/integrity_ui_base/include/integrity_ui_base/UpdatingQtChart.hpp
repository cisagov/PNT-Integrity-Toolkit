//============================================================================//
//------------------- integrity_ui/UpdatingQtChart.hpp ---------*- C++ -*-----//
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
/// \brief    Updating QT Chart class
/// \author   Josh Clanton <josh.clanton@is4s.com>
/// \date     April 1, 2020
//============================================================================//
#ifndef INTEGRITY_UI__UPDATING_QTCHART_HPP
#define INTEGRITY_UI__UPDATING_QTCHART_HPP

#include <QXYSeries>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

#include <algorithm>
#include <iostream>
/// Namespace for the PNT Integrity User Interface
namespace integrity_ui
{
/// \todo Change error messages to log messages from strings

/// \brief Helper class for implementing an updating QT chart in the main
/// Integrity UI
class UpdatingQtChart
{
public:
  /// \brief Constructor for the chart object
  ///
  /// \param chartViewPtr Pointer to the parent QChartView object
  /// \param yLabel Label for the y-axis on the chart
  /// \param maxSamples The  maximum number of samples to view on the chart
  /// \param verticalZoom Flag to enable vertical zoom (rubber band) on chart
  /// \param autoScale Indicates whether or not the chart object should
  /// auto-scale based on chart data
  /// \param yMin If not auto-scaling, sets the minimum y-axis value
  /// \param yMax If not auto-scaling, sets the maximum y-axis value
  UpdatingQtChart(QtCharts::QChartView* chartViewPtr,
                  const QString&        yLabel,
                  const size_t&         maxSamples,
                  const bool&           verticalZoom = true,
                  const bool&           autoScale    = true,
                  const size_t&         yMin         = 0,
                  const size_t&         yMax         = 4);

  /// \brief Updates the chart with the current buffer
  /// \param seriesName String identifier for the series to update
  void updateWithBuffer(const QString& seriesName);

  /// \brief Adds a new series to the chart
  /// \param seriesName String ID for the series to add
  void addSeries(const std::string& seriesName)
  {
    addSeries(QString::fromStdString(seriesName));
  }

  /// \brief Adds a new series to the chart
  /// \param seriesName String ID for the series to add
  void addSeries(const QString& seriesName);

  /// \brief Adds a data point to the selected series
  /// \param seriesName String ID for series
  /// \param dataPoint Data point to add
  void addData(const std::string& seriesName, const double& dataPoint)
  {
    addData(QString::fromStdString(seriesName), dataPoint);
  }

  /// \brief Adds a data point to the selected series
  /// \param seriesName String ID for series
  /// \param dataPoint Data point to add
  void addData(const QString& seriesName, const double& dataPoint);

  /// \brief Updates all chart sereis with current buffer data
  void updateAll()
  {
    auto seriesIt = seriesMap_.begin();
    for (; seriesIt != seriesMap_.end(); ++seriesIt)
    {
      updateWithBuffer(seriesIt->first);
    }
  }

  /// \brief Checks chart for existence of series
  /// \param seriesName String ID for series
  bool hasSeries(const std::string& seriesName)
  {
    return hasSeries(QString::fromStdString(seriesName));
  }

  /// \brief Checks chart for existence of series
  /// \param seriesName String ID for series
  bool hasSeries(const QString& seriesName)
  {
    return (seriesMap_.find(seriesName) != seriesMap_.end());
  }

private:
  QtCharts::QChart*                       chartPtr_;
  std::map<QString, QtCharts::QXYSeries*> seriesMap_;
  std::map<QString, QVector<QPointF>>     bufferMap_;
  std::map<QString, bool>                 dataAdded_;
  bool                                    autoScale_;
  size_t                                  maxSamples_;

  QtCharts::QValueAxis* axisY_;
  QtCharts::QValueAxis* axisX_;

  double yMin_;
  double yMax_;

  void autoscale(const double& dataPoint);
};

}  // namespace integrity_ui
#endif
