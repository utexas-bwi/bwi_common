/**
 * \file  point.h
 * \brief  Contains basic point data structures for bwi_mapper
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
 * $ Id: 02/27/2013 04:19:42 PM piyushk $
 *
 **/

#pragma once

#include <opencv/cv.h>
#include <stdint.h>

namespace bwi_mapper {

  /**
   * \class Point2d
   * \brief A simple class to hold a 2D pixel point along with distance to a 
   *        reference position
   */
  class Point2d : public cv::Point {
    public:
      Point2d() : cv::Point() {}
      Point2d(int x, int y) : cv::Point(x, y) {}
      Point2d(const Point2d& pt) : cv::Point(pt.x, pt.y), 
          distance_from_ref(pt.distance_from_ref) {}
      Point2d(const cv::Point& pt) : cv::Point(pt) {}
      /* Variables */
      float distance_from_ref;
      
  }; /* Point2d */

  /**
   * \class Point2dDistanceComp
   * \brief A simple class acting as a comparator for Point2d using the 
   *        reference distance. Useful while using std::sort.
   */
  struct Point2dDistanceComp {

    /**
     * \brief   comapares 2 Point2d objects. When used with std::sort, returns a
     *          sorted ist of Point2d objects (ascending w.r.t distance_from_ref
     */
    bool operator() (Point2d i, Point2d j);

  }; /* Point2dDistanceComp */

  /**
   * \class Point2f
   * \brief A floating point 2D point
   */
  typedef cv::Point2f Point2f;
  
} /* bwi_mapper */
