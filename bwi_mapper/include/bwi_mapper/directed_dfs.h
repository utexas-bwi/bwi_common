/**
 * \file  directed_dfs.h
 * \brief  A specific implementation of Directed DFS (DFS with a priority queue)
 *         to find whether 2 points are close in obstacle space. Priority is 
 *         done on using the Euclidean distance to the goal as a heuristic.
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
 * $ Id: 02/27/2013 04:28:26 PM piyushk $
 *
 **/
#pragma once

#include <nav_msgs/OccupancyGrid.h>
#include <bwi_mapper/structures/point.h>

namespace bwi_mapper {

  /**
   * \class DirectedDFS
   * \brief The directed DFS class. Checks whether 2 points are close inside the
   *        map in obstacle space. Uses euclidean distance to goal to guide the
   *        search
   */
  class DirectedDFS {

    public:
      
      /**
       * \brief   Constructor initializing underlying map
       * \param   map OccupancyGrid representing underlying map 
       */
      DirectedDFS(const nav_msgs::OccupancyGrid& map) : map_(map) {}

      
      /**
       * \brief   Non-recusrive start point for performing DFS
       * \param   depth DFS attempts an 8 connected search. This is the max 
       *          pixel depth that we perform the search to.
       * \return  bool true if path found within depth, false otherwise
       */
      bool searchForPath(const Point2d &start, const Point2d &goal, 
          uint32_t depth, bool in_obstacle_space = true);

    private:

      /**
       * \brief   Recursive function performing DFS
       * \param   depth DFS attempts an 8 connected search. This is the max 
       *          pixel depth that we perform the search to.
       * \param   visited a boolean vector of size height * width containing
       *          information whether a map_idx has been visited already or not
       * \return  bool true if path found within depth, false otherwise
       */
      bool searchForPath(const Point2d &start, const Point2d &goal, 
          uint32_t depth, std::vector<bool> &visited, 
          bool in_obstacle_space = true);

      /**
       * \brief   Gets neighbours for a given node iff they are also obstacles 
       *          and have not been visited before
       * \param   visited a boolean vector of size height * width containing
       *          information whether a map_idx has been visited already or not
       * \param   neighbours returned neighbours 
       */
      void getOrderedNeighbours(const Point2d &from, const Point2d &goal, 
          const std::vector<bool> &visited, std::vector<Point2d> &neighbours,
          bool in_obstacle_space = true);

      /** /brief the underlying map over which DFS is performed */
      const nav_msgs::OccupancyGrid& map_;

  };

} /* bwi_mapper */
