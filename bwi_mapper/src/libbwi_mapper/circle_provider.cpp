/**
 * \file  circle_provider.cpp
 * \brief  Implementation for circle_provider.h
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
 * $ Id: 02/25/2013 04:58:37 PM piyushk $
 *
 **/

#include <bwi_mapper/circle_provider.h>

#include <iostream>

namespace bwi_mapper {

  /**
   * \brief   Returns the singleton instance
   */
  CircleProvider& CircleProvider::getInstance() {
    static CircleProvider instance;
    return instance;
  }

  /**
   * \brief   Returns the circle in the form of a boolean array. The array
   *          represents a box with the locations of the circle set to true.
   */
  void CircleProvider::getCircle(int radius, std::vector<bool>& points, 
      bool connect4) {

    points.resize((2*radius + 1)*(2*radius + 1));
    int error = -radius;
    int x = radius;
    int y =0;
    while (x >= y) {
      plot8Points(x, y, points, radius);
      error += y;
      ++y;
      error += y;
      if (error >= 0) {
        error -= x;
        if (connect4) {
          plot8Points(x, y, points, radius);
        }
        --x;
        error -= x;
      }
    }
  }

  /**
   * \brief   Prints a given circle to the screen, useful for debugging
   */
  void CircleProvider::printCircle(const std::vector<bool>& points, 
      int radius) {

    for (size_t i = 0; i < points.size(); i++) {
      if (i % (2*radius + 1) == 0 && i != 0) {
        std::cout << std::endl;
      }
      if (points[i]) {
        std::cout << "%";
      } else {
        std::cout << " ";
      }
    }
    std::cout << std::endl;
  }

  /**
   * \brief   given an (x,y) in the first octant, plots mirrored values
   *          along all 8 octants. See getCircle for description of the 
   *          parameters
   */
  void CircleProvider::plot8Points(int x, int y, std::vector<bool>& points, 
      int radius) {
    plot4Points(x, y, points, radius);
    if (x != y) {
      plot4Points(y, x, points, radius);
    }
  }

  /**
   * \brief   given an (x,y) in the first octant, plots this value in all
   *          quadrants. See getCircle for description of the parameters
   */
  void CircleProvider::plot4Points(int x, int y, std::vector<bool>& points, 
      int radius) {
    CIRCLE_REF(points, radius, x, y) = true;
    CIRCLE_REF(points, radius, x, -y) = true;
    CIRCLE_REF(points, radius, -x, -y) = true;
    CIRCLE_REF(points, radius, -x, y) = true;
  }

} /* bwi_mapper */
