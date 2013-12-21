/**
 * \file  voronoi_point.cpp
 * \brief  Implementation of the voronoi point class
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
 * $ Id: 03/04/2013 12:23:45 PM piyushk $
 *
 **/

#include <bwi_mapper/structures/voronoi_point.h>
#include <bwi_mapper/directed_dfs.h>
#include <bwi_mapper/point_utils.h>

namespace bwi_mapper {

  /**
   * \brief   Attempts to add a basis candidate to this voronoi point. The 
   *          basis candidate is only added if and only if it is
   *          2 * threshhold away from an existing basis for that  point, 
   *          and is not too close to existing basis point in obstacle space
   *          If it is too close, then it is only retained if it is better
   *          than an existing basis, in which case the existing basis is 
   *          thrown out.
   */
  void VoronoiPoint::addBasisCandidate(const Point2d& candidate, 
      uint32_t threshold, const nav_msgs::OccupancyGrid& map, bool is_naive) {

    if (is_naive) {
      if (basis_points.size() == 0) {
        basis_points.push_back(candidate);
        basis_distance = candidate.distance_from_ref;
        return;
      }
      if (candidate.distance_from_ref > basis_distance + 0.01) {
        return;
      }
      if (candidate.distance_from_ref <= basis_distance - 0.01) {
        basis_points.clear();
        basis_points.push_back(candidate);
        basis_distance = candidate.distance_from_ref;
        return;
      }
      
      // Check if the point we are adding is next to an existing basis point 
      if (basis_points.size() == 1) {
        Point2d basis = basis_points[0];
        int diff_x = abs(basis.x - candidate.x);
        int diff_y = abs(basis.y - candidate.y);
        if (diff_x <= 1 && diff_y <= 1) {
          // Point is from the same site
          return;
        }
      }
      basis_points.push_back(candidate);
      return;
    }
    
    if (basis_points.size() == 0) {
      basis_points.push_back(candidate);
      basis_distance = candidate.distance_from_ref;
      return;
    }
    if (candidate.distance_from_ref > basis_distance + 1) {
      return;
    }

    basis_points.push_back(candidate);
    std::sort(basis_points.begin(), basis_points.end(), 
        Point2dDistanceComp());
    basis_distance = basis_points[0].distance_from_ref;

    // Get the directed DFS searcher
    DirectedDFS dfs(map);

    // Mark elements that are too close to be erased
    std::vector<size_t> elements_to_erase;
    for (size_t i = 0; i < basis_points.size(); ++i) {

      // Check if the clearance for this point is much further away from the 
      // minimum clearance
      if (basis_points[i].distance_from_ref > basis_distance + 1) {
        elements_to_erase.push_back(i);
        break;
      }

      std::vector<size_t>::iterator erase_iterator = 
        elements_to_erase.begin();

      for (size_t j = 0; j < i; ++j) {

        while (erase_iterator != elements_to_erase.end() && 
            *erase_iterator < j) {
          erase_iterator++;
        }

        if (erase_iterator != elements_to_erase.end() && 
            *erase_iterator == j) { 
          continue;
        }

        // See if basis point i is too close to basis point j. retain j
        float distance = bwi_mapper::getMagnitude(basis_points[i] - basis_points[j]);
        if (distance < 2 * threshold) {
          elements_to_erase.push_back(i); // does not affect erase_iterator
          break;
        }

        // See if these basis points are really close by searching for a
        // short path in the walls
        if (dfs.searchForPath(basis_points[i], basis_points[j], 
              2 * basis_distance)) {
          elements_to_erase.push_back(i);
          break;
        }
      }
    }

    // Actually remove elements from the basis point array
    for (size_t i = elements_to_erase.size() - 1; 
        i <= elements_to_erase.size(); --i) {
      basis_points.erase(basis_points.begin() + elements_to_erase[i]);
    }
  }

} /* bwi_mapper */
