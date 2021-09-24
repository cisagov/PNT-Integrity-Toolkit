//============================================================================//
//---------------------- integrity_ui_/mapform.hpp -------------*- C++ -*-----//
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
#include "integrity_ui_base/mapform.hpp"
#include <iostream>

MapForm::MapForm(QWidget* parent, QWebEngineView* web)
  : QWidget(parent), webview(web)
{
  QWebEngineSettings::globalSettings()->setAttribute(
    QWebEngineSettings::PluginsEnabled, true);

  // resource was added in qt designer and this property was set there
  // web->setUrl(QUrl("qrc:/resources/html/google_maps.html"));
}

MapForm::~MapForm()
{
}

void MapForm::receiveCoordinates(double lat,
                                 double lng,
                                 int    index,
                                 int    colorIdx,
                                 double heading,
                                 double error)
{
  QString js = QString("updateMap(") + QString::number(lat, 'f', 8) +
               QString(", ") + QString::number(lng, 'f', 8) + QString(", ") +
               QString::number(index, 'f', 8) + QString(", ") +
               QString::number(heading, 'f', 8) + QString(", ") +
               QString::number(error, 'f', 8) + QString(", ") +
               QString::number(colorIdx, 'f', 8) + QString(")");

  webview->page()->runJavaScript(js);
}

void MapForm::resetMarkers()
{
  QString js = QString("resetMarkers();");
  webview->page()->runJavaScript(js);
}
