/**
 * \file  topological_mapper.h
 * \brief  Constructs the topological graph using the voronoi approximation
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
 * $ Id: 03/01/2013 12:06:45 PM piyushk $
 *
 **/

#pragma once

#include <bwi_mapper/voronoi_approximator.h>
#include <bwi_mapper/graph.h>

namespace bwi_mapper {


  /**
   * \class TopologicalMapper
   * \brief Computes the 2 variants of the topological graph using the voronoi
   *        approximator.
   */
  class TopologicalMapper : public VoronoiApproximator {

    public:

      /**
       * \brief   Constructor. Initializes map_resp_ from given file. Only used
       *          to call the base class constructor
       * \param   fname absolute or relative system file location for the YAML
       *          file
       */
      TopologicalMapper (const std::string &fname) :
        VoronoiApproximator(fname) {}

      /**
       * \brief   computes the topological graph given the threshold for 
       *          VoronoiApproximator and a parameter controlling the size of 
       *          critical regions.
       * \param   threshold same as threhold in  VoronoiApproximator()
       * \param   critical_epsilon (meters) no 2 critical points can be closer
       *          than this distance.
       * \param   merge_threshold graph vertices having a smaller area than this
       *          should be merged together(meter^2)
       */
      void computeTopologicalGraph(double threshold, double critical_epsilon,
          double merge_threshold); 

      /**
       * \brief   draws critical points and lines onto a given image starting at
       *          (orig_x, orig_y)
       */
      void drawCriticalPoints(cv::Mat &image, 
          uint32_t orig_x = 0, uint32_t orig_y = 0); 

      /**
       * \brief   draws colored regions corresponding to each critical region
       *          onto a given image starting at (orig_x, orig_y)
       */
      void drawConnectedComponents(cv::Mat &image,
          uint32_t orig_x = 0, uint32_t orig_y = 0); 

      void drawRegionGraph(cv::Mat &image, 
          uint32_t orig_x = 0, uint32_t orig_y = 0);
      void drawPointGraph(cv::Mat &image, 
          uint32_t orig_x = 0, uint32_t orig_y = 0);

      void writeRegionGraphToFile(std::string &filename);
      void writePointGraphToFile(std::string &filename);

      /**
       * \brief   Prints information about all the critical points to screen. 
       *          Used for testing the output of Topological Mapper.
       */
      void printCriticalPoints(); 

      /**
       * \brief   Draws the base map, voronoi points and critical points, lines
       *          and regions on to image. Should be only used for testing the 
       *          output for Topological Mapper.
       * \param   image OpenCV image we are drawing output on 
       */
      void drawOutput(cv::Mat &image); 
      void saveOutput(); 

    protected:
      
      /**
       * \brief   Compute critical regions once the voronoi points have been
       *          calculated
       * \param   critical_epsilon see TopologicalMapper() for more details
       */
      void computeCriticalRegions (double critical_epsilon);

      void computeGraph (double merge_threshold);

      /**
       * \brief   Draws critical lines (4-connected) onto given image starting
       *          at (orig_x, orig_y)
       */
      void drawCriticalLines(cv::Mat &image, uint32_t orig_x = 0, 
          uint32_t orig_y = 0, bool draw_idx = false, 
          bool visualization_image = false); 

      /** \brief stores the array of computed Critical Points **/
      std::vector<VoronoiPoint> critical_points_;

      /** \brief image containing just the obstacles and critical lines. used
       *         for finding connected components**/
      cv::Mat component_image_;

      /** \brief a vector the size of the map that contains the index of the
       *         component to which that pixel belongs (0 if background) */
      std::vector<int32_t> component_map_;

      /** \brief a randomized color mapping selected for each component idx */
      std::vector<cv::Vec3b> component_colors_;

      /** \brief the totatl number of components found + 1 for background */
      size_t num_components_;

      Graph region_graph_;
      Graph pass_1_graph_;
      Graph pass_2_graph_;
      Graph pass_3_graph_;
      Graph pass_4_graph_;
      Graph point_graph_;

  }; /* TopologicalMapper */

} /* bwi_mapper */
