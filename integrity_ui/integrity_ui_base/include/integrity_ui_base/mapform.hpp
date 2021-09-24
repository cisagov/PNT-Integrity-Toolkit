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
/// \file
/// \brief    The integrity_ui library
/// \author   Josh Clanton <josh.clanton@is4s.com>
/// \date     April 13, 2020
//============================================================================//
#ifndef MAPFORM_H
#define MAPFORM_H

#include <QWidget>
#include <QtWebEngineWidgets/QtWebEngineWidgets>

/**
 * @brief The MapForm class
 * Uses google_maps.html with the api_key to make calls to google services.
 * The services used are handled in javascript. I have written the functions
 * being called in the google_maps.html file for reference.
 *
 * For additional functionality, add slots that call javascript functions that
 * are user defined within the google_maps.html file.
 */

class MapForm : public QWidget
{
  Q_OBJECT

public:
  explicit MapForm(QWidget* parent, QWebEngineView* web);
  ~MapForm();

private slots:
  void resetMarkers();
  // Update coordinates on map. (latitude, longitude, idx, heading, error,
  // colorIdx)
  void receiveCoordinates(double, double, int, int, double, double);

private:
  QWebEngineView* webview;
  double          lastLat_;
  double          lastLong_;
  double          lastAlt_;
  bool            firstPopulation_ = true;
};

#endif  // MAPFORM_H
