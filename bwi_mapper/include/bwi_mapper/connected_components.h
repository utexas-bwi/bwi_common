/**
 * \file  connected_components.h
 * \brief  Connected Components implementation to get all the points in a
 *         critical region, along with neighbouring critical points to form the
 *         topological graph.
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
 * $ Id: 03/01/2013 01:04:40 PM piyushk $
 *
 **/

#pragma once

#include <opencv/cv.h>
#include <vector>

namespace bwi_mapper {

  /**
   * \class ConnectedComponents
   * \brief API for 8-connected connected components algorithm to find critical
   *        regions in a map demarcated by obstalces and critical lines
   *        Neigbouring critical regions help form the topological graph. Since
   *        this algorithm is 8-connected, any demarcating lines should be drawn
   *        4-connected if they are single pixel.
   */
  class ConnectedComponents {

    public:

      /**
       * \brief  Given an image map containing obstacles and demarcating
       *         critical lines, the constructor runs a critical components
       *         algorithm to find critical regions.
       * \param  image map with obstacles and critical lines 
       * \return  
       */
      ConnectedComponents (const cv::Mat& image, std::vector<int32_t>& component_map);

      /**
       * \brief Returns the number of components obtained by running the
       *        critical components algorithm.
       */
      size_t getNumberComponents();

    private:

      /** \brief The number of critical components that are found is stored so
       *         that it can be retrieved later using getNumberComponents
       */
      size_t number_components_;

  }; /* ConnectedComponents */

} /* bwi_mapper */
