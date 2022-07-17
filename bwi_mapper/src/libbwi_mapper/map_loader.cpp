/**
 * \file  map_loader.cpp
 * \brief  Implementation for map_loader.h
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
 * $ Id: 02/25/2013 05:50:20 PM piyushk $
 *
 **/

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <libgen.h>

#include <map_server/image_loader.h>
#include <bwi_mapper/map_loader.h>
#include <bwi_mapper/map_utils.h>

#ifdef HAVE_NEW_YAMLCPP
namespace YAML {
  // The >> operator disappeared in yaml-cpp 0.5, so this function is
  // added to provide support for code written under the yaml-cpp 0.3 API.
  template<typename T>
  void operator >> (const YAML::Node& node, T& i)
  {
    i = node.as<T>();
  }
}
#endif

namespace bwi_mapper {

  /**
   * \brief   Constructor. Initializes map_resp_ with the given file
   */
  MapLoader::MapLoader (const std::string& fname) {

    std::string mapfname = "";   
    double origin[3];
    int negate;
    double occ_th, free_th;
    double res;

    // open supplied yaml file
    std::ifstream fin(fname.c_str());
    if (fin.fail()) {
      std::cerr << "FATAL: Map_server could not open: " 
        << fname << std::endl;
      exit(-1);
    }

    // Initilize parameters
    YAML::Node doc;
#ifdef HAVE_NEW_YAMLCPP
    doc = YAML::Load(fin);
#else
    YAML::Parser parser(fin);   
    parser.GetNextDocument(doc);
#endif
    try { 
      doc["resolution"] >> res; 
    } catch (YAML::InvalidScalar) { 
      std::cerr << "FATAL: The map does not contain a resolution tag "
        << "or it is invalid." << std::endl;
      exit(-1);
    }
    try { 
      doc["negate"] >> negate; 
    } catch (YAML::InvalidScalar) { 
      std::cerr << "FATAL: The map does not contain a negate tag "
        << "or it is invalid." << std::endl;
      exit(-1);
    }
    try { 
      doc["occupied_thresh"] >> occ_th; 
    } catch (YAML::InvalidScalar) { 
      std::cerr << "FATAL: The map does not contain occupied_thresh tag "
        << "or it is invalid." << std::endl;
      exit(-1);
    }
    try { 
      doc["free_thresh"] >> free_th; 
    } catch (YAML::InvalidScalar) { 
      std::cerr << "FATAL: The map does not contain free_thresh tag "
        << "or it is invalid." << std::endl;
      exit(-1);
    }
    try { 
      doc["origin"][0] >> origin[0]; 
      doc["origin"][1] >> origin[1]; 
      doc["origin"][2] >> origin[2]; 
    } catch (YAML::InvalidScalar) { 
      std::cerr << "FATAL: The map does not contain origin tag "
        << "or it is invalid." << std::endl;
      exit(-1);
    }

    // Get image data
    try { 
      doc["image"] >> mapfname; 
      if(mapfname.size() == 0) {
        std::cerr << "FATAL: The image tag cannot be an empty string."
          << std::endl;
        exit(-1);
      }
      if(mapfname[0] != '/') {
        // dirname can modify what you pass it
        char* fname_copy = strdup(fname.c_str());
        mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
        free(fname_copy);
      }
    } catch (YAML::InvalidScalar) { 
      std::cerr << "FATAL: The map does not contain an image tag "
        << "or it is invalid." << std::endl;
      exit(-1);
    }

    std::cout << "MapLoader: Loading map from image " << mapfname << std::endl;
    map_server::loadMapFromFile(&map_resp_, mapfname.c_str(), res, negate, 
        free_th, free_th, origin);

  }

  /**
   * \brief   Draw map_resp_ onto image starting at (orig_x, orig_y). Uses
   *          the overloaded drawMap internally.
   */
  void MapLoader::drawMap(cv::Mat &image, uint32_t orig_x, uint32_t orig_y) {
    drawMap(image, map_resp_.map, orig_x, orig_y);
  }

  void MapLoader::getMapInfo(nav_msgs::MapMetaData& map) const {
    map = map_resp_.map.info;
  }

  void MapLoader::getMap(nav_msgs::OccupancyGrid &map) const {
    map = map_resp_.map;
  }

  /**
   * \brief   Draw map onto image starting at (orig_x, orig_y)
   */
  void MapLoader::drawMap(cv::Mat &image, const nav_msgs::OccupancyGrid& map, 
      uint32_t orig_x, uint32_t orig_y) {

    // Check if matrix has enough space, otherwise make it larger
    if (image.data == NULL ||
        (uint32_t) image.cols < orig_x + map.info.width ||
        (uint32_t) image.rows < orig_y + map.info.height) {
      cv::Mat old_mat = image.clone();
      image.create(orig_y + map.info.height, orig_x + map.info.width, CV_8UC3);
      for (uint32_t j = 0; j < (uint32_t) old_mat.rows; ++j) {
        const cv::Vec3b* old_row_j = old_mat.ptr<cv::Vec3b>(j);
        cv::Vec3b* row_j = image.ptr<cv::Vec3b>(j);
        for (uint32_t i = 0; i < (uint32_t) old_mat.cols; ++i) {
          row_j[i] = old_row_j[i];
        }
      }
    }

    // Put map onto image
    for (uint32_t j = 0; j < map.info.height; ++j) {
      cv::Vec3b* image_row_j = image.ptr<cv::Vec3b>(j + orig_y);
      for (uint32_t i = 0; i < map.info.width; ++i) {
        uint8_t val = map.data[MAP_IDX(map.info.width, i, j)];
        cv::Vec3b& pixel = image_row_j[i + orig_x];
        switch (val) {
          case 100:
            pixel[0] = pixel[1] = pixel[2] = 0;
            break;
          case 0:
            pixel[0] = pixel[1] = pixel[2] = 255;
            break;
          default:
            pixel[0] = pixel[1] = pixel[2] = 128;
        }
      }
    }
  }

} /* bwi_mapper */


