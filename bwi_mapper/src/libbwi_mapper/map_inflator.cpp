/**
 * \file  map_inflator.cpp
 * \brief  Provides an implementation for map_inflator.h
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
 * $ Id: 02/25/2013 05:24:29 PM piyushk $
 *
 **/

#include <bwi_mapper/map_inflator.h>

namespace bwi_mapper {
  
  /**
   * \brief   A simple utility function that expands the map based on inflation
   *          distance in meters
   */
  void inflateMap(double threshold, const nav_msgs::OccupancyGrid& map, 
      nav_msgs::OccupancyGrid& inflated_map) {

    inflated_map.header = map.header;
    inflated_map.info = map.info;

    // expand the map out based on the circumscribed robot distance
    int expand_pixels = ceil(threshold / map.info.resolution);
    inflated_map.data.resize(map.info.height * map.info.width);
    for (int i = 0; i < (int)map.info.height; ++i) {
      for (int j = 0; j < (int)map.info.width; ++j) {
        int low_i = (i - expand_pixels < 0) ? 0 : i - expand_pixels;
        int high_i = (i + expand_pixels >= (int)map.info.height) ? 
          map.info.height - 1 : i + expand_pixels;
        int max = 0;
        for (int k = low_i; k <= high_i; ++k) {
          int diff_j = floor(sqrtf(expand_pixels * expand_pixels - (i - k) * (i - k)));
          int low_j = (j - diff_j < 0) ? 0 : j - diff_j;
          int high_j = (j + diff_j >= (int)map.info.width) ? 
            map.info.width - 1 : j + diff_j;
          for (int l = low_j; l <= high_j; ++l) {
            if (map.data[k * map.info.width + l] > max) {
              max = map.data[k * map.info.width + l];
            }
          }
        }
        inflated_map.data[i * map.info.width + j] = max;
      }
    }
  }

} /* bwi_mapper */
