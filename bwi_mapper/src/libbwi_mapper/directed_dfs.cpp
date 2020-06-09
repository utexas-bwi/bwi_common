/**
 * \file  directed_dfs.cpp
 * \brief  Implementation of the Directed DFS class
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
 * $ Id: 02/27/2013 05:02:54 PM piyushk $
 *
 **/

#include <bwi_mapper/directed_dfs.h>
#include <bwi_mapper/map_utils.h>

namespace bwi_mapper {

  /**
   * \brief   Non-recusrive start point for performing DFS
   */
  bool DirectedDFS::searchForPath(const Point2d &start, const Point2d &goal,
      uint32_t depth, bool in_obstacle_space) {

    std::vector<bool> visited(map_.info.height * map_.info.width, false);
    return searchForPath(start, goal, depth, visited, in_obstacle_space);
  }

  /**
   * \brief   Recusrive function performing DFS
   */
  bool DirectedDFS::searchForPath(const Point2d &start, const Point2d &goal, 
      uint32_t depth, std::vector<bool> &visited, bool in_obstacle_space) {

    //std::cout << start.x << " " << start.y << std::endl;

    // Termination crit
    if (start == goal) {
      return true;
    }
    if (depth == 0) {
      return false;
    }

    uint32_t start_idx = MAP_IDX(map_.info.width, start.x, start.y);
    visited[start_idx] = true;

    std::vector<Point2d> neighbours;
    getOrderedNeighbours(start, goal, visited, neighbours, in_obstacle_space);
    for (size_t i = 0; i < neighbours.size(); ++i) {
      Point2d& n = neighbours[i];
      // Check if it has been visited again - quite likely that one of the 
      // previous loop iterations have covered this already
      uint32_t n_idx = MAP_IDX(map_.info.width, n.x, n.y);
      if (visited[n_idx]) {
        continue;
      }
      bool success = searchForPath(n, goal, depth - 1, visited);
      if (success)
        return true;
    }

    return false; // disconnected components
  }


  /**
   * \brief   Gets neighbours for a given node iff they are also obstacles 
   *          and have not been visited before
   */
  void DirectedDFS::getOrderedNeighbours(const Point2d &from, 
      const Point2d &goal, const std::vector<bool> &visited, 
      std::vector<Point2d> &neighbours, bool in_obstacle_space) {

    size_t neighbour_count = 8;
    int8_t x_offset[] = {1, 0, 1, -1, 1, -1, 0, 1};
    int8_t y_offset[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    neighbours.clear();
    for (size_t i = 0; i < neighbour_count; ++i) {
      // Check if neighbours are still on map
      Point2d p = from + Point2d(x_offset[i],y_offset[i]);
      // covers negative case as well (unsigned)
      if (p.x >= (int) map_.info.width || p.y >= (int) map_.info.height ||
          p.x <= 0 || p.y <= 0) { 
        continue;
      }
      uint32_t map_idx = MAP_IDX(map_.info.width, p.x, p.y);
      if (visited[map_idx] || (in_obstacle_space && map_.data[map_idx] == 0) ||
          (!in_obstacle_space && map_.data[map_idx] != 0)) {
        continue;
      }
      p.distance_from_ref = norm(p - goal); 
      neighbours.push_back(p);
    }
    std::sort(neighbours.begin(), neighbours.end(), Point2dDistanceComp());
  }

} /* bwi_mapper */
