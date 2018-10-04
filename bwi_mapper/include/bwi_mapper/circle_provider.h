/**
 * \file  circle_provider.h
 * \briefs Provides an implementation of Bresenham's mid-point circle algorithm.
 * The code is directly based from the wikipedia entry at 
 * http://en.wikipedia.org/wiki/Midpoint_circle_algorithm
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
 * $ Id: 02/20/2013 01:24:48 PM piyushk $
 *
 **/

#pragma once

#include <vector>

#define CIRCLE_REF(c,r,x,y) c[(y+r)*(2*r + 1) + (x+r)]

namespace bwi_mapper {

  /**
   * \class CircleProvider
   * \brief Singleton class which provides the mid-point circle algorithm
   * implementation
   */
  class CircleProvider {
    public:

      /**
       * \brief   Returns the singleton instance
       * \return  A reference to the instance
       */
      static CircleProvider& getInstance();

      /**
       * \brief   Returns the circle in the form of a boolean array. The array
       *          represents a box with the locations of the circle set to true.
       * \param   radius radius of the desired circle (pixels)
       * \param   points reference returning which points in a box of size 
       *          (2*radius + 1)^2 are part of the circle. The vector represents
       *          the box in row major form, with the 0th element starting at
       *          (-radius, -radius)
       * \param   connect4 boolean value when true returns a 4 connected circle.
       *          If false, a 8-connected circle is returned.
       */
      void getCircle(int radius, std::vector<bool>& points, 
          bool connect4 = true);

      
      /**
       * \brief   Prints a given circle to the screen, useful for debugging
       * \param   points circle returned by getCircle 
       * \param   radius radius of the circle. should be same as that supplied
       *          to getCircle to construct points
       */
      void printCircle(const std::vector<bool>& points, int radius);

    private:

      /**
       * \brief Private constructor for singleton instance.
       */
      CircleProvider () {}

      /**
       * \brief Unimplemented copy constructor
       */
      CircleProvider (CircleProvider const&); //Don't implement

      /**
       * \brief Unimplemented assignment operator
       */
      void operator=(CircleProvider const&); //Don't implement

      
      /**
       * \brief   given an (x,y) in the first octant, plots mirrored values
       *          along all 8 octants. See getCircle for description of the 
       *          parameters
       */
      void plot8Points(int x, int y, std::vector<bool>& points, int radius);

      /**
       * \brief   given an (x,y) in the first octant, plots this value in all
       *          quadrants. See getCircle for description of the parameters
       */
      void plot4Points(int x, int y, std::vector<bool>& points, int radius);

  }; /* CircleProvider */

} /* bwi_mapper */
