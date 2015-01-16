/**
 * \file  path_finder.cpp
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 *
 * Copyright (c) 2015, UT Austin

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
 **/

#include <bwi_mapper/map_utils.h>
#include <bwi_mapper/path_finder.h>

namespace bwi_mapper {

  PathFinder::PathFinder(const nav_msgs::OccupancyGrid& map, const Point2d& start_pt) : width_(map.info.width) {

    search_space_.resize(map.info.height * map.info.width, NOT_CONNECTED);

    // Mark all the obstacles.
    for (int row = 0; row < map.info.height; ++row) {
      for (int col = 0; col < map.info.width; ++col) {
        int map_idx = MAP_IDX(width_, col, row);
        if (map.data[map_idx] == 0) {
          search_space_[map_idx] = OBSTACLE;
        }
      }
    }

    // Lets setup the initial condition.
    int start_idx = MAP_IDX(width_, start_pt.x, start_pt.y);
    if (search_space_[start_idx] != OBSTACLE) {
      search_space_[start_idx] = 0;
    } else {
      // The start point is in an obstacles. Nothing to see here. Move along!
      return;
    }

    bool change = true;
    // Doing in-place changes shouldn't modify the result and may be even faster. The alternative would be to create
    // a copy and update every loop.
    while (change) {
      change = false;
      for (int row = 0; row < map.info.height; ++row) {
        for (int col = 0; col < map.info.width; ++col) {
        int current_idx = MAP_IDX(width_, col, row);

          if (row > 1) {
            int neighbor_idx = MAP_IDX(width_, col, row - 1);
            if ((search_space_[neighbor_idx] != OBSTACLE && search_space_[neighbor_idx] != NOT_CONNECTED) && 
                (search_space_[current_idx] == NOT_CONNECTED || search_space_[current_idx] > search_space_[neighbor_idx] + 1)) {
              search_space_[current_idx] = search_space_[neighbor_idx] + 1; 
              change = true;
            }
          }

          if (col > 1) {
            int neighbor_idx = MAP_IDX(width_, col - 1, row);
            if ((search_space_[neighbor_idx] != OBSTACLE && search_space_[neighbor_idx] != NOT_CONNECTED) && 
                (search_space_[current_idx] == NOT_CONNECTED || search_space_[current_idx] > search_space_[neighbor_idx] + 1)) {
              search_space_[current_idx] = search_space_[neighbor_idx] + 1; 
              change = true;
            }
          }

          if (row < map.info.height - 1) {
            int neighbor_idx = MAP_IDX(width_, col, row + 1);
            if ((search_space_[neighbor_idx] != OBSTACLE && search_space_[neighbor_idx] != NOT_CONNECTED) && 
                (search_space_[current_idx] == NOT_CONNECTED || search_space_[current_idx] > search_space_[neighbor_idx] + 1)) {
              search_space_[current_idx] = search_space_[neighbor_idx] + 1; 
              change = true;
            }
          }

          if (col < map.info.width - 1) {
            int neighbor_idx = MAP_IDX(width_, col + 1, row);
            if ((search_space_[neighbor_idx] != OBSTACLE && search_space_[neighbor_idx] != NOT_CONNECTED) && 
                (search_space_[current_idx] == NOT_CONNECTED || search_space_[current_idx] > search_space_[neighbor_idx] + 1)) {
              search_space_[current_idx] = search_space_[neighbor_idx] + 1; 
              change = true;
            }
          }

        }
      }

    }

  }
      
  bool PathFinder::pathExists(const Point2d& pt) {
    return search_space_[MAP_IDX(width_, pt.x, pt.y)] >= 0;
  }

  int PathFinder::getManhattanDistance(const Point2d& pt) {
    return search_space_[MAP_IDX(width_, pt.x, pt.y)];
  }

} /* bwi_mapper */
