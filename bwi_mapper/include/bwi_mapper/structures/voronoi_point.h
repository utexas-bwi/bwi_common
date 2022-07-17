/**
 * \file  voronoi_point.h
 * \brief  Base class for a voronoi point. Simple wrapper around Point2d that
 *         maintains a given separation between basis points
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
 * $ Id: 03/01/2013 01:17:50 PM piyushk $
 *
 **/

#pragma once

#include <vector>
#include <nav_msgs/OccupancyGrid.h>
#include <bwi_mapper/structures/point.h>

namespace bwi_mapper {

  class VoronoiPoint : public Point2d {

    public:
      VoronoiPoint() : Point2d() {}
      VoronoiPoint(int x, int y) : Point2d(x, y) {}
      VoronoiPoint(const Point2d& pt) : Point2d(pt) {}

      std::vector<Point2d> basis_points;

      /** /brief minimum clearance to a basis point - used while computing 
       *         basis points
       */
      float basis_distance;

      /** /brief average clearance from all basis points - 
       *         used after all basis points have been computed 
       */ 
      float average_clearance; 

      /** /brief if this point is a critical point, how lower is the clearance 
       *         of this point in comparison of its neighbours 
       */
      float critical_clearance_diff;

       
      /**
       * \brief   Attempts to add a basis candidate to this voronoi point. The 
       *          basis candidate is only added if and only if it is
       *          2 * threshhold away from an existing basis for that  point, 
       *          and is not too close to existing basis point in obstacle space
       *          If it is too close, then it is only retained if it is better
       *          than an existing basis, in which case the existing basis is 
       *          thrown out.
       * \param   threshold Any point having an obstacle closer than threshold 
       *          is rejected as it is not useful, even if it might be a valid 
       *          voronoi point
       */
      void addBasisCandidate(const Point2d& candidate, uint32_t threshold, 
          const nav_msgs::OccupancyGrid& map, bool use_naive);

  }; /* VoronoiPoint */
  
} /* bwi_mapper */