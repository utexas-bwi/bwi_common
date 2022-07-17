/**
 * \file  point_utils.cpp
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
 * $ Id: 04/23/2013 04:32:54 PM piyushk $
 *
 **/

#include <bwi_mapper/point_utils.h>

namespace bwi_mapper {

  /* http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment */
  float minimumDistanceToLineSegment(Point2f v, Point2f w, Point2f p) {
    // Return minimum distance between line segment vw and point p
    const float l2 = getMagnitude(w-v);  
    if (l2 == 0.0) return getMagnitude(p-v);   // v == w case
    // Consider the line extending the segment, parameterized as v + t (w - v).
    // We find projection of point p onto the line. 
    // It falls where t = [(p-v) . (w-v)] / |w-v|^2
    const float t = (p - v).dot(w - v) / (l2 * l2);
    if (t < 0.0) return getMagnitude(p - v);  // Beyond the 'v' end 
    else if (t > 1.0) return getMagnitude(p - w);  // Beyond the 'w' 
    const Point2f projection = v + t * (w - v); 
    return getMagnitude(p - projection);
  }

  float getMagnitude(Point2f p) {
    return cv::norm(p);
  }
  
} /* bwi_mapper */
