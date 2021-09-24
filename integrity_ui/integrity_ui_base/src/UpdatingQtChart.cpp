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
//
//    Updating QT Chart class
//    Josh Clanton <josh.clanton@is4s.com>
//    April 1, 2020
//============================================================================/
#include "integrity_ui_base/UpdatingQtChart.hpp"

namespace integrity_ui
{
//==============================================================================
//----------------------------- Constructor ------------------------------------
//==============================================================================

UpdatingQtChart::UpdatingQtChart(QtCharts::QChartView* chartViewPtr,
                                 const QString&        yLabel,
                                 const size_t&         maxSamples,
                                 const bool&           verticalZoom,
                                 const bool&           autoScale,
                                 const size_t&         yMin,
                                 const size_t&         yMax)
  : chartPtr_(chartViewPtr->chart())
  , autoScale_(autoScale)
  , maxSamples_(maxSamples)
  , yMin_(std::numeric_limits<double>::max())
  , yMax_(std::numeric_limits<double>::min())
{
  if (verticalZoom)
  {
    chartViewPtr->setRubberBand(QtCharts::QChartView::VerticalRubberBand);
  }

  axisX_ = new QtCharts::QValueAxis;
  axisY_ = new QtCharts::QValueAxis;

  axisX_->setRange(0, maxSamples);
  axisX_->setLabelFormat("%g");

  if (!autoScale)
  {
    axisY_->setRange(yMin, yMax);
  }
  axisY_->setTitleText(yLabel);
  chartPtr_->addAxis(axisX_, Qt::AlignBottom);
  chartPtr_->addAxis(axisY_, Qt::AlignLeft);

  chartPtr_->setMargins({0, 0, 0, 0});
  chartPtr_->setContentsMargins(0, 0, 0, 0);
  chartPtr_->setBackgroundRoundness(0);

  chartPtr_->legend()->setVisible(true);
  chartPtr_->legend()->setAlignment(Qt::AlignBottom);
}

//==============================================================================
//-------------------------- updateWithBuffer ----------------------------------
//==============================================================================
void UpdatingQtChart::updateWithBuffer(const QString& seriesName)
{
  // check to see if data has been added since last update
  // if not, add nan to the series

  auto seriesIt = seriesMap_.find(seriesName);
  auto bufferIt = bufferMap_.find(seriesName);

  if ((seriesIt != seriesMap_.end()) && (bufferIt != bufferMap_.end()))
  {
    // if no data has been added since last update,
    // add nan to the buffer prior to the update
    if (!dataAdded_[seriesName])
    {
      addData(seriesName, std::numeric_limits<double>::quiet_NaN());
    }
    seriesMap_[seriesName]->replace(bufferMap_[seriesName]);

    // set the flag back to false for the next update
    dataAdded_[seriesName] = false;
  }
  else
  {
    std::cout << "Series / buffer not found for: " << seriesName.toStdString()
              << std::endl;
  }
}

//==============================================================================
//--------------------------------- addSeries ----------------------------------
//==============================================================================
void UpdatingQtChart::addSeries(const QString& seriesName)
{
  QtCharts::QXYSeries* newSeries = new QtCharts::QLineSeries();
  newSeries->setName(seriesName);
  chartPtr_->addSeries(newSeries);
  newSeries->attachAxis(axisX_);
  newSeries->attachAxis(axisY_);

  seriesMap_[seriesName] = newSeries;
  bufferMap_[seriesName] = QVector<QPointF>();
  chartPtr_->legend()->update();
}

//==============================================================================
//--------------------------------- autoscale ----------------------------------
//==============================================================================
void UpdatingQtChart::autoscale(const double& dataPoint)
{
  bool minMaxChanged = false;
  if (dataPoint > yMax_)
  {
    yMax_         = dataPoint;
    minMaxChanged = true;
  }
  if (dataPoint < yMin_)
  {
    yMin_         = dataPoint;
    minMaxChanged = true;
  }

  if (yMax_ == yMin_)
  {
    if (yMax_ == 0.0)
    {
      axisY_->setMax(0.1);
      axisY_->setMin(-0.1);
    }
    else
    {
      axisY_->setMax(1.05 * yMax_);
      axisY_->setMin(0.95 * yMin_);
    }
  }
  else if (minMaxChanged)
  {
    axisY_->setMax(yMax_);
    axisY_->setMin(yMin_);
  }
}

//==============================================================================
//----------------------------------- addData ----------------------------------
//==============================================================================
void UpdatingQtChart::addData(const QString& seriesName,
                              const double&  dataPoint)
{
  if (!hasSeries(seriesName))
  {
    std::cout << "UpdatingQtChart::addData() Requested series '"
              << seriesName.toStdString() << "' does not exist" << std::endl;
  }
  else
  {
    dataAdded_[seriesName] = true;
    if (autoScale_)
    {
      autoscale(dataPoint);
    }
    auto buffIt = bufferMap_.find(seriesName);
    if (buffIt != bufferMap_.end())
    {
      if (buffIt->second.isEmpty())
      {
        buffIt->second.reserve(maxSamples_);
        for (size_t ii = 0; ii < maxSamples_; ++ii)
        {
          buffIt->second.append(QPointF(ii, 0));
        }
      }

      for (size_t s = 0; s < maxSamples_ - 1; ++s)
      {
        buffIt->second[s].setY(buffIt->second.at(s + 1).y());
      }

      buffIt->second[maxSamples_ - 1].setY((qreal(dataPoint)));
    }
  }
}
}  // namespace integrity_ui