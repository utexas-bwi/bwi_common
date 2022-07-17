/**
 * \file  voronoi_approximator.h
 * \brief  Constructs a voronoi approximation given a map of the world. The map
 *         is a discrete grid world with each cell set to occupied or not, and
 *         the voronoi approximation is done in this discrete space.
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
 * $ Id: 02/21/2013 11:52:37 AM piyushk $
 *
 **/

#pragma once

#include <bwi_mapper/map_loader.h>
#include <bwi_mapper/map_inflator.h>
#include <bwi_mapper/directed_dfs.h>
#include <bwi_mapper/structures/point.h>
#include <bwi_mapper/structures/voronoi_point.h>

namespace bwi_mapper {


  /**
   * \class VoronoiApproximator
   * \brief The voronoi approximator API. Inherits MapLoader to directly load
   *        a map from file and be able to draw a map
   */
  class VoronoiApproximator : public MapLoader {

    public:

      /**
       * \brief   Constructor. Initializes map_resp_ from given file. Only used
       *          to call the base class constructor
       * \param   fname absolute or relative system file location for the YAML
       *          file
       */
      VoronoiApproximator(const std::string& fname) :
        MapLoader(fname), initialized_(false) {}

      /**
       * \brief   Computes the points that lie on the Voronoi Graph using an 
       *          approximate strategy useful for voronoi approximation during
       *          mapping
       * \param   threshold minimum obstacle distance in meters Any point which 
       *          is less than threshold away from an obstacle is rejected as a
       *          voronoi candidate. This allows for not caring about minor
       *          breaks in a wall which can happen for any SLAM algorithm
       */
      void findVoronoiPoints(double threshold, bool use_naive = false,
          int sub_pixel_sampling = 1); 

      /**
       * \brief   Draws the base map and voronoi points on to image. Should be
       *          only used for testing the output for Voronoi Approximator.
       * \param   image OpenCV image we are drawing output on 
       */
      void drawOutput(cv::Mat &image);

      /**
       * \brief   Prints information about all the voronoi points to screen. 
       *          Used for testing the output for Voronoi Approximator.
       */
      void printVoronoiPoints();

      /**
       * \brief   Draw voronoi points onto image starting at (orig_x, orig_y)
       * \param   image OpenCV image we are writing the map onto
       */
      void drawVoronoiPoints(cv::Mat &image, 
          uint32_t orig_x = 0, uint32_t orig_y = 0);

    protected:

      /** \brief the compute voronoi points are placed in here */
      std::vector<VoronoiPoint> voronoi_points_;

      /** \brief inflated map used to throw out points too close to obstacle  */
      nav_msgs::OccupancyGrid inflated_map_;

      /** \brief Safety check to make sure findVoronoiPoints has been called */
      bool initialized_;
        
  }; /* VoronoiApproximator */
  
} /* bwi_mapper */
