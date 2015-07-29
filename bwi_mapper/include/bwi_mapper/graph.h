/**
 * \file  graph.h
 * \brief  Contains some simple data structures for holding the graph
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
 * $ Id: 03/04/2013 04:15:26 PM piyushk $
 *
 **/

#ifndef GRAPH_E8QGZKSM
#define GRAPH_E8QGZKSM

#include <boost/lexical_cast.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/labeled_graph.hpp>
#include <bwi_mapper/structures/point.h>

#include <opencv/cv.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>

namespace bwi_mapper {

  // Graph
  struct Vertex {
    Point2f location;
    double pixels;
  };

  // Edge
  struct Edge {
    double weight;
  };

  //Define the graph using those classes
  typedef boost::adjacency_list<
    boost::vecS, boost::vecS, boost::undirectedS, Vertex, Edge
  > Graph;

  /**
   * \brief   draws the given graph onto an image starting at
   *          (orig_x, orig_y)
   */
  void drawGraph(cv::Mat &image, const Graph& graph,
      uint32_t orig_x = 0, uint32_t orig_y = 0,
      bool put_text = true, bool put_all_edges = true,
      std::vector<std::pair<size_t, size_t> > specific_edges =
      std::vector<std::pair<size_t, size_t> >());

  void drawArrowOnImage(cv::Mat &image, const cv::Point2f &arrow_center, float orientation,
                        const cv::Scalar &color, int size, int thickness);

  void drawArrowOnGraph(cv::Mat &image, const Graph& graph,
      std::pair<size_t, float> arrow, uint32_t map_width, uint32_t map_height,
      cv::Scalar color = cv::Scalar(0,0,255),
      uint32_t orig_x = 0, uint32_t orig_y = 0);

  void drawCircleOnGraph(cv::Mat &image, const Graph& graph,
      size_t node, cv::Scalar color = cv::Scalar(0,0,255),
      uint32_t orig_x = 0, uint32_t orig_y = 0);

  void drawSquareOnGraph(cv::Mat &image, const Graph& graph,
      size_t node, cv::Scalar color = cv::Scalar(0,0,255),
      uint32_t orig_x = 0, uint32_t orig_y = 0, int size = 30,
      int thickness = 2);

  void writeGraphToFile(const std::string &filename,
      const Graph& graph, const nav_msgs::MapMetaData& info);

  void readGraphFromFile(const std::string &filename,
      const nav_msgs::MapMetaData& info, Graph& graph);

  Point2f getLocationFromGraphId(int idx, const Graph& graph);

  size_t getClosestIdOnGraph(const Point2f &point,
      const Graph &graph, double threshold = 0.0);

  size_t getClosestIdOnGraphFromEdge(const Point2f &point,
      const Graph &graph, size_t prev_graph_id);

  size_t getClosestEdgeOnGraphGivenId(const Point2f& point, const Graph &graph, size_t one_graph_id);
  /* Functions defined in paper */

  bool isVisible(size_t u, size_t v, const Graph &graph,
      const nav_msgs::OccupancyGrid& map);

  float getNodeAngle(size_t u, size_t v, const Graph &graph);

  float getEuclideanDistance(size_t u, size_t v, const Graph &graph);

  float getShortestPathWithDistance(size_t start_idx, size_t goal_idx,
      std::vector<size_t> &path_from_goal, const Graph &graph);

  float getShortestPathDistance(size_t start_idx, size_t goal_idx,
      const Graph &graph);

  void getAdjacentNodes(size_t v, const Graph& graph,
      std::vector<size_t>& adjacent_vertices);

  void getVisibleNodes(size_t v, const Graph& graph,
      const nav_msgs::OccupancyGrid& grid,
      std::vector<size_t>& visible_vertices, float visibility_range = 0.0f);

} /* bwi_mapper */

#endif /* end of include guard: GRAPH_E8QGZKSM */
