/**
 * \file  map_utils.cpp
 * \brief  Implementation for map utilities
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
 * $ Id: 04/23/2013 03:39:42 PM piyushk $
 *
 **/

#include <bwi_mapper/map_utils.h>

namespace bwi_mapper {

  bool locationsInDirectLineOfSight(const Point2f& pt1, const Point2f& pt2,
      const nav_msgs::OccupancyGrid map) {

    int x0 = lrint(pt1.x), y0 = lrint(pt1.y);
    int x1 = lrint(pt2.x), y1 = lrint(pt2.y);
    int dx = abs(x1-x0), dy = abs(y1-y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    float err = dx - dy;
    bool is_occupied = false;
    while (!is_occupied) {
      is_occupied = map.data[MAP_IDX(map.info.width, x0, y0)] > 50;
      if (x0 == x1 && y0 == y1) break;
      float e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        x0 += sx;
      }
      if (x0 == x1 && y0 == y1) {
        is_occupied = map.data[MAP_IDX(map.info.width, x0, y0)] > 50;
        break;
      }
      if (e2 < dx) {
        err += dx;
        y0 += sy;
      }
    }
    return !is_occupied;
  }
  
  Point2f toGrid(const Point2f& pt, const nav_msgs::MapMetaData& info) {
    return (pt - Point2f(info.origin.position.x, info.origin.position.y)) * 
        (1.0 / info.resolution);
  }

  Point2f toMap(const Point2f& pt, const nav_msgs::MapMetaData& info) {
    return Point2f(info.origin.position.x, info.origin.position.y) + 
        info.resolution * pt;
  }

} /* bwi_mapper */
