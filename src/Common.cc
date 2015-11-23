/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include "swarm/Common.hh"

using namespace swarm;

//////////////////////////////////////////////////
Common::Common()
  : searchMinLatitude(0),
    searchMaxLatitude(0),
    searchMinLongitude(0),
    searchMaxLongitude(0)
{
}

//////////////////////////////////////////////////
bool Common::LoadSearchArea(sdf::ElementPtr _searchAreaSDF)
{
  while (_searchAreaSDF)
  {
    if (_searchAreaSDF->HasElement("min_relative_latitude_deg") &&
        _searchAreaSDF->HasElement("max_relative_latitude_deg") &&
        _searchAreaSDF->HasElement("min_relative_longitude_deg") &&
        _searchAreaSDF->HasElement("max_relative_longitude_deg"))
    {
      this->searchMinLatitude =
        _searchAreaSDF->GetElement("min_relative_latitude_deg")->Get<double>();
      this->searchMaxLatitude =
        _searchAreaSDF->GetElement("max_relative_latitude_deg")->Get<double>();
      this->searchMinLongitude =
        _searchAreaSDF->GetElement("min_relative_longitude_deg")->Get<double>();
      this->searchMaxLongitude =
        _searchAreaSDF->GetElement("max_relative_longitude_deg")->Get<double>();

      return true;
    }
    _searchAreaSDF = _searchAreaSDF->GetNextElement("swarm_search_area");
  }

  return false;
}

/////////////////////////////////////////////////
bool Common::LoadSphericalCoordinates(sdf::ElementPtr _sphericalCoordsSDF)
{
  while (_sphericalCoordsSDF)
  {
    if (_sphericalCoordsSDF->HasElement("latitude_deg") &&
        _sphericalCoordsSDF->HasElement("longitude_deg"))
    {
      // Offset the search borders by the origin.
      this->searchMinLatitude +=
        _sphericalCoordsSDF->GetElement("latitude_deg")->Get<double>();
      this->searchMaxLatitude +=
        _sphericalCoordsSDF->GetElement("latitude_deg")->Get<double>();
      this->searchMinLongitude +=
        _sphericalCoordsSDF->GetElement("longitude_deg")->Get<double>();
      this->searchMaxLongitude +=
        _sphericalCoordsSDF->GetElement("longitude_deg")->Get<double>();

      return true;
    }

    _sphericalCoordsSDF =
      _sphericalCoordsSDF->GetNextElement("spherical_coordinates");
  }

  return false;
}

//////////////////////////////////////////////////
void Common::SearchArea(double &_minLatitude,
                        double &_maxLatitude,
                        double &_minLongitude,
                        double &_maxLongitude)
{
  _minLatitude = this->searchMinLatitude;
  _maxLatitude = this->searchMaxLatitude;
  _minLongitude = this->searchMinLongitude;
  _maxLongitude = this->searchMaxLongitude;
}

//////////////////////////////////////////////////
double Common::SearchMinLatitude() const
{
  return this->searchMinLatitude;
}

//////////////////////////////////////////////////
double Common::SearchMaxLatitude() const
{
  return this->searchMaxLatitude;
}

//////////////////////////////////////////////////
double Common::SearchMinLongitude() const
{
  return this->searchMinLongitude;
}

//////////////////////////////////////////////////
double Common::SearchMaxLongitude() const
{
  return this->searchMaxLongitude;
}

//////////////////////////////////////////////////
void Common::SetSearchMinLatitude(const double _value)
{
  this->searchMinLatitude = _value;
}

//////////////////////////////////////////////////
void Common::SetSearchMaxLatitude(const double _value)
{
  this->searchMaxLatitude = _value;
}

//////////////////////////////////////////////////
void Common::SetSearchMinLongitude(const double _value)
{
  this->searchMinLongitude = _value;
}

//////////////////////////////////////////////////
void Common::SetSearchMaxLongitude(const double _value)
{
  this->searchMaxLongitude = _value;
}
