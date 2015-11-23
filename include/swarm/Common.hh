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

/// \file Common.hh
/// \brief Functions and attributes required by multiple plugins.

#ifndef __SWARM_COMMON__
#define __SWARM_COMMON__

#include <sdf/sdf.hh>

namespace swarm
{
  class Common
  {
    /// \brief Constructor
    public: Common();

    /// \brief Destructor
    public: virtual ~Common() = default;

    /// \brief Get the minimum latitude of the search area.
    /// \return Search area minimum latitude
    public: double SearchMinLatitude() const;

    /// \brief Get the maximum latitude of the search area.
    /// \return Search area maximum latitude
    public: double SearchMaxLatitude() const;

    /// \brief Get the minimum longitude of the search area.
    /// \return Search area minimum longitude
    public: double SearchMinLongitude() const;

    /// \brief Get the maximum longitude of the search area.
    /// \return Search area maximum longitude
    public: double SearchMaxLongitude() const;

    /// \brief set the minimum latitude of the search area.
    /// \param[in] _value Minimum latitude
    public: void SetSearchMinLatitude(const double _value);

    /// \brief set the maximum latitude of the search area.
    /// \param[in] _value Maximum latitude
    public: void SetSearchMaxLatitude(const double _value);

    /// \brief Set the minimum longitude of the search area.
    /// \param[in] _value Minimum longitude
    public: void SetSearchMinLongitude(const double _value);

    /// \brief Set the maximum longitude of the search area.
    /// \param[in] _value Maximum longitude
    public: void SetSearchMaxLongitude(const double _value);

    /// \brief Get the search area, in GPS coordinates.
    ///
    /// \param[out] _minLatitude Minimum latitude will be written here.
    /// \param[out] _maxLatitude Maximum latitude will be written here.
    /// \param[out] _minLongitude Minimum longitude will be written here.
    /// \param[out] _maxLongitude Maximum longitude will be written here.
    public: void SearchArea(double &_minLatitude,
                            double &_maxLatitude,
                            double &_minLongitude,
                            double &_maxLongitude);

    /// \brief Load the search area from SDF
    /// \param[in] _searchAreaElem Pointer to the <search_area> element.
    /// \return True if the search area was found.
    public: bool LoadSearchArea(sdf::ElementPtr _searchAreaSDF);

    /// \brief Load spherical coordinates from SDF
    /// \param[in] _sphereicalCoordsSDF Pointer to the
    /// <spherical_coordinates> sdf element
    /// \return True if the spherical coordinates element was found.
    public: bool LoadSphericalCoordinates(sdf::ElementPtr _sphericalCoordsSDF);

    /// \brief Min/max lat/long of search area.
    private: double searchMinLatitude, searchMaxLatitude,
                    searchMinLongitude, searchMaxLongitude;
  };
}
#endif
