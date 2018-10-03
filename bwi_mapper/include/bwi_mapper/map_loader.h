/**
 * \file  map_loader.h
 * \brief  Simple wrapper around the map_server code to read maps from a 
 * the supplied yaml file. This class itself is based on the map_server node
 * inside the map_server package (written by Brian Gerkey)
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 *
 * Copyright (c) 2013, UT Austin

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the <organization> nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *
 * $ Id: 02/20/2013 03:58:41 PM piyushk $
 *
 **/

#pragma once

#include <string>

#include <nav_msgs/GetMap.h>
#include <nav_msgs/GetMapResponse.h>
#include <yaml-cpp/yaml.h>
#include <opencv/cv.h>

namespace bwi_mapper {

  /**
   * \class MapLoader
   * \brief Base class for reading a standard ROS map from a YAML file and 
   *        writing it to an OpenCV Image
   */
  class MapLoader {

    public:
      
      /**
       * \brief   Constructor. Initializes map_resp_ with the given file
       * \param   fname absolute or relative system file location for the YAML
       *          file
       */
      MapLoader (const std::string& fname);

      /**
       * \brief   Draw map_resp_ onto image starting at (orig_x, orig_y). Uses
       *          the overloaded drawMap internally.
       * \param   image OpenCV image we are writing the map onto 
       */
      void drawMap(cv::Mat &image, uint32_t orig_x = 0, uint32_t orig_y = 0);

      void getMapInfo(nav_msgs::MapMetaData& info) const;
      void getMap(nav_msgs::OccupancyGrid& map) const;

      /**
       * \brief   Draw map onto image starting at (orig_x, orig_y)
       * \param   image OpenCV image we are writing the map onto
       */
      void drawMap(cv::Mat &image, const nav_msgs::OccupancyGrid& map, 
          uint32_t orig_x = 0, uint32_t orig_y = 0);

    protected:

      /** /brief The base map being loaded by MapLoader. GetMap::Response is
       * used to reuse code from map_server */
      nav_msgs::GetMap::Response map_resp_;

  }; /* MapLoader */

} /* bwi_mapper */
