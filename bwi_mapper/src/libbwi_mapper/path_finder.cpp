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
#include <cassert>
#include <fstream>
#include <stdexcept>

namespace bwi_mapper {

  PathFinder::PathFinder(const nav_msgs::OccupancyGrid& map, const Point2d& start_pt) :
    width_(map.info.width), height_(map.info.height) {

    search_space_.resize(map.info.height * map.info.width, NOT_CONNECTED);

    // Mark all the obstacles.
    for (int row = 0; row < map.info.height; ++row) {
      for (int col = 0; col < map.info.width; ++col) {
        int map_idx = MAP_IDX(width_, col, row);
        if (map.data[map_idx] == 100) {
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

    std::set<int> current_points;
    current_points.insert(start_idx);

    while(!current_points.empty()) {
      int current_idx = *(current_points.begin());
      current_points.erase(current_points.begin());
      int row = current_idx / width_;
      int col = current_idx % width_;

      int step_x[] = {0, 1, 0, -1};
      int step_y[] = {1, 0, -1, 0};

      for (int pt = 0; pt < 4; ++pt) {
        int neighbor_x = col + step_x[pt];
        int neighbor_y = row + step_y[pt];
        if (neighbor_x >= 0 && neighbor_x < map.info.width && neighbor_y >= 0 && neighbor_y < map.info.height) {
          int neighbor_idx = MAP_IDX(width_, neighbor_x, neighbor_y);
          if ((search_space_[neighbor_idx] != OBSTACLE) &&
              (search_space_[neighbor_idx] == NOT_CONNECTED || (search_space_[neighbor_idx] > search_space_[current_idx] + 1))) {
            search_space_[neighbor_idx] = search_space_[current_idx] + 1;
            current_points.insert(current_points.end(), neighbor_idx);
          }
        }
      }
    }

    // std::ofstream fout("/home/piyushk/test.txt");
    // for (int row = 0; row < map.info.height; ++row) {
    //   for (int col = 0; col < map.info.width; ++col) {
    //     int map_idx = MAP_IDX(width_, col, row);
    //     fout << std::setfill(' ') << std::setw(5) << search_space_[map_idx];
    //   }
    //   fout << std::endl;
    // }
    // fout.close();
    // throw std::runtime_error("blah!");

  }

  bool PathFinder::pathExists(const Point2d& pt) {
    int pt_idx = MAP_IDX(width_, pt.x, pt.y);
    assert(pt_idx >= 0);
    assert(pt_idx < search_space_.size());
    return search_space_[pt_idx] >= 0;
  }

  int PathFinder::getManhattanDistance(const Point2d& pt) {
    int pt_idx = MAP_IDX(width_, pt.x, pt.y);
    assert(pt_idx >= 0);
    assert(pt_idx < search_space_.size());
    return search_space_[pt_idx];
  }

  bool PathFinder::getNextCloserPointToSearchOrigin(const Point2d& pt, Point2d& next) {
    int pt_idx = MAP_IDX(width_, pt.x, pt.y);
    assert(pt_idx >= 0);
    assert(pt_idx < search_space_.size());
    int pt_val = search_space_[pt_idx];
    if (pt_val == NOT_CONNECTED || pt_val == OBSTACLE || pt_val == 0) {
      return false;
    }

    int step_x[] = {0, 1, 0, -1};
    int step_y[] = {1, 0, -1, 0};

    for (int neighbor_counter = 0; neighbor_counter < 4; ++neighbor_counter) {
      int neighbor_x = pt.x + step_x[neighbor_counter];
      int neighbor_y = pt.y + step_y[neighbor_counter];
      if (neighbor_x >= 0 && neighbor_x < width_ && neighbor_y >= 0 && neighbor_y < height_) {
        int neighbor_idx = MAP_IDX(width_, neighbor_x, neighbor_y);
        assert(neighbor_idx >= 0);
        assert(neighbor_idx < search_space_.size());
        if (search_space_[neighbor_idx] != OBSTACLE &&
            search_space_[neighbor_idx] < pt_val) {
          next.x = neighbor_x;
          next.y = neighbor_y;
          return true;
        }
      }
    }

    // Shouldn't reach here.
    throw std::runtime_error("There's a bug in PathFinder::getNextCloserPointToSearchOrigin()");
  }

} /* bwi_mapper */
