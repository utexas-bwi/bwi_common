/**
 * \file  topological_mapper.cpp
 * \brief Implementation for the topological mapper 
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
 * $ Id: 03/04/2013 03:07:20 PM piyushk $
 *
 **/

#include <bwi_mapper/topological_mapper.h>
#include <bwi_mapper/connected_components.h>
#include <bwi_mapper/map_utils.h>
#include <bwi_mapper/point_utils.h>

#include <boost/foreach.hpp>

#include <opencv/highgui.h>
#include <opencv2/opencv_modules.hpp>
#ifdef HAVE_OPENCV_IMGCODECS
// this is OpenCV 3 and we need extra includes
#include <opencv2/imgcodecs.hpp>
#endif
#ifdef HAVE_OPENCV_IMGPROC
// this is OpenCV 3 and we need extra includes
#include <opencv2/imgproc/imgproc.hpp>
#endif

namespace bwi_mapper {

  /**
   * \brief   computes the topological graph given the threshold for 
   *          VoronoiApproximator and a parameter controlling the size of 
   *          critical regions.
   * \param   threshold same as threhold in  VoronoiApproximator()
   * \param   critical_epsilon (meters) no 2 critical points can be closer
   *          than this distance.
   * \param   merge_threshold graph vertices having a smaller area than this
   *          should be merged together (meter^2)
   */
  void TopologicalMapper::computeTopologicalGraph(double threshold, 
      double critical_epsilon, double merge_threshold) {

    std::cout << "computeTopologicalGraph(): find voronoi points" << std::endl;
    findVoronoiPoints(threshold);
    std::cout << "computeTopologicalGraph(): compute critical regions" << std::endl;
    computeCriticalRegions(critical_epsilon);
    std::cout << "computeTopologicalGraph(): compute ze graph" << std::endl;
    computeGraph(merge_threshold);
  }

  /**
   * \brief   draws critical points and lines onto a given image starting at
   *          (orig_x, orig_y)
   */
  void TopologicalMapper::drawCriticalPoints(cv::Mat &image, 
      uint32_t orig_x, uint32_t orig_y) {

    for (size_t i = 0; i < critical_points_.size(); ++i) {
      VoronoiPoint &vp = critical_points_[i];
      cv::circle(image, 
          cv::Point(vp.x + orig_x, vp.y + orig_y), 8, cv::Scalar(0,0,255), -1);
    }

    drawCriticalLines(image, orig_x, orig_y, false, true);
  }

  /**
   * \brief   draws colored regions corresponding to each critical region
   *          onto a given image starting at (orig_x, orig_y)
   */
  void TopologicalMapper::drawConnectedComponents(cv::Mat &image,
      uint32_t orig_x, uint32_t orig_y) {

    // Figure out different colors - 1st color should always be black
    size_t num_colors = num_components_;
    component_colors_.resize(num_colors);
    for (size_t i = 0; i < num_colors; ++i) {
      component_colors_[i] = 
        cv::Vec3b(160 + rand() % 64, 160 + rand() % 64, 160 + rand() % 64);
    }

    // component 0 is obstacles + background. don't draw?

    // Now paint!
    for (uint32_t j = 1; j < map_resp_.map.info.height; ++j) {
      cv::Vec3b* image_row_j = image.ptr<cv::Vec3b>(j + orig_y);
      for (uint32_t i = 0; i < map_resp_.map.info.width; ++i) {
        size_t map_idx = MAP_IDX(map_resp_.map.info.width, i, j);
        if (component_map_[map_idx] == -1)
          continue;
        cv::Vec3b& pixel = image_row_j[i + orig_x];
        pixel[0] = component_colors_[component_map_[map_idx]][0];
        pixel[1] = component_colors_[component_map_[map_idx]][1];
        pixel[2] = component_colors_[component_map_[map_idx]][2];

      }
    }
  }


  void TopologicalMapper::drawRegionGraph(cv::Mat &image,
      uint32_t orig_x, uint32_t orig_y) {
    drawGraph(image, region_graph_, orig_x, orig_y);
  }

  void TopologicalMapper::drawPointGraph(cv::Mat &image,
      uint32_t orig_x, uint32_t orig_y) {
    drawGraph(image, point_graph_, orig_x, orig_y);
  }

  void TopologicalMapper::writeRegionGraphToFile(std::string &filename) {
    writeGraphToFile(filename, region_graph_, map_resp_.map.info);
  }

  void TopologicalMapper::writePointGraphToFile(std::string &filename) {
    writeGraphToFile(filename, point_graph_, map_resp_.map.info);
  }

  /**
   * \brief   Prints information about all the critical points to screen. 
   *          Used for testing the output of Topological Mapper.
   */
  void TopologicalMapper::printCriticalPoints() {
    for (size_t i = 0; i < critical_points_.size(); ++i) {
      VoronoiPoint &vp = critical_points_[i];
      std::cout << " (" << vp.x << "," << vp.y << "): ";
      for (size_t j = 0; j < vp.basis_points.size(); ++j) {
        std::cout << " (" << vp.basis_points[j].x << "," 
                  << vp.basis_points[j].y << "," 
                  << vp.basis_points[j].distance_from_ref << "), ";
      }
      std::cout << std::endl;
    }
  }

  /**
   * \brief   Draws the base map, voronoi points and critical points, lines
   *          and regions on to image. Should be only used for testing the 
   *          output for Topological Mapper.
   * \param   image OpenCV image we are drawing output on 
   */
  void TopologicalMapper::drawOutput(cv::Mat &image) {
    VoronoiApproximator::drawOutput(image);
    drawMap(image, 2 * map_resp_.map.info.width);
    drawConnectedComponents(image, 2 * map_resp_.map.info.width);
    drawCriticalPoints(image, 2 * map_resp_.map.info.width);
    drawMap(image, 3 * map_resp_.map.info.width);
    drawRegionGraph(image, 3 * map_resp_.map.info.width);
    drawMap(image, 4 * map_resp_.map.info.width);
    drawPointGraph(image, 4 * map_resp_.map.info.width);
  }

  void TopologicalMapper::saveOutput() {
    cv::Mat image;
    drawMap(image);
    drawVoronoiPoints(image);
    cv::imwrite("graphvoronoi.png", image);
    drawMap(image);
    drawConnectedComponents(image);
    drawCriticalPoints(image);
    cv::imwrite("graphcritical.png", image);
    drawMap(image);
    drawGraph(image, region_graph_);
    cv::imwrite("graphoriginal.png", image);
    drawMap(image);
    drawGraph(image, pass_1_graph_);
    cv::imwrite("graphpass1.png", image);
    drawMap(image);
    drawGraph(image, pass_2_graph_);
    cv::imwrite("graphpass2.png", image);
    drawMap(image);
    drawGraph(image, pass_3_graph_);
    cv::imwrite("graphpass3.png", image);
    drawMap(image);
    drawGraph(image, pass_4_graph_);
    cv::imwrite("graphpass4.png", image);
    drawMap(image);
    drawGraph(image, point_graph_);
    cv::imwrite("graphfinal.png", image);
  }

  /**
   * \brief   Compute critical regions once the voronoi points have been
   *          calculated
   * \param   critical_epsilon see TopologicalMapper() for more details
   */
  void TopologicalMapper::computeCriticalRegions (double critical_epsilon) {

    // Compute critical points
    size_t pixel_critical_epsilon = 
      critical_epsilon / map_resp_.map.info.resolution;

    for (size_t i = 0; i < voronoi_points_.size(); ++i) {
      VoronoiPoint &vpi = voronoi_points_[i];
      float average_neighbourhood_clearance = 0;
      size_t neighbour_count = 0;
      bool is_clearance_minima = true;
      // Get all voronoi points in a region around this voronoi point
      for (size_t j = 0; j < voronoi_points_.size(); ++j) {
        // Don't check if it is the same point
        if (j == i) {
          continue;
        }

        // Compute distance of jth point to ith point - 
        // don't compare if too far away
        VoronoiPoint &vpj = voronoi_points_[j];
        float distance = bwi_mapper::getMagnitude(vpj - vpi); 
        if (distance > pixel_critical_epsilon) {
          continue;
        }

        average_neighbourhood_clearance += vpj.average_clearance;
        ++neighbour_count;

        // See if the jth point has lower clearance than the ith
        if (vpj.average_clearance < vpi.average_clearance) {
          is_clearance_minima = false;
          break;
        }
      }

      // If no neighbours, then this cannot be a critical point
      if (neighbour_count == 0) {
        continue;
      }

      // Check if the point is indeed better than the neighbourhood
      // This can happen is any 2 walls of the map are too straight
      average_neighbourhood_clearance /= neighbour_count;
      if (vpi.average_clearance >= 0.999 * average_neighbourhood_clearance) {
        continue;
      }

      /* std::cout << "Check1: VP at " << vpi << " with clearance " << vpi.average_clearance << " and avg neighbourhood clearance " << average_neighbourhood_clearance << std::endl; */

      vpi.critical_clearance_diff = 
        average_neighbourhood_clearance - vpi.average_clearance;

      std::vector<size_t> mark_for_removal;
      // This removal is not perfect, but ensures you don't have critical 
      // points too close.
      for (size_t j = 0; j < critical_points_.size(); ++j) {

        // Check if in same neighbourhood
        VoronoiPoint &vpj = critical_points_[j];
        float distance = norm(vpj - vpi); 
        if (distance > pixel_critical_epsilon) {
          continue;
        }

        // If in same neighbourhood, retain better point
        if (vpj.critical_clearance_diff >= vpi.critical_clearance_diff) {
          is_clearance_minima = false;
          break;
        } else {
          mark_for_removal.push_back(j);
        }
      }

      if (is_clearance_minima) {
        // Let's remove any points marked for removal
        for (size_t j = mark_for_removal.size() - 1; 
            j < mark_for_removal.size(); --j) { //unsigned
          critical_points_.erase(
              critical_points_.begin() + mark_for_removal[j]);
        }

        // And then add this critical point
        critical_points_.push_back(vpi);
      }
    }

    // Remove any critical points where the point itself does not lie on the line
    std::vector<size_t> mark_for_removal;
    for (size_t i = 0; i < critical_points_.size(); ++i) {
      VoronoiPoint &cp = critical_points_[i];
      float theta0 = 
        atan2((cp.basis_points[0] - cp).y,
              (cp.basis_points[0] - cp).x);
      float theta1 = 
        atan2((cp.basis_points[1] - cp).y,
              (cp.basis_points[1] - cp).x);
      float thetadiff = fabs(theta1 - theta0);

      // We don't need to worry about wrapping here due known range of atan2
      if (thetadiff > M_PI + M_PI/12 || thetadiff < M_PI - M_PI/12) {
        mark_for_removal.push_back(i);
      }
    }

    // Remove bad critical points
    for (size_t j = mark_for_removal.size() - 1; 
        j < mark_for_removal.size(); --j) { //unsigned
      critical_points_.erase(
          critical_points_.begin() + mark_for_removal[j]);
    }

    // Once you have critical lines, produce connected regions (4-connected)
    // draw the critical lines on to a copy of the map so that we can find
    // connected regions
    cv::Mat component_map_color;
    drawMap(component_map_color, inflated_map_);
    cvtColor(component_map_color, component_image_, CV_RGB2GRAY);
    drawCriticalLines(component_image_);

    component_map_.resize(
        component_image_.rows * component_image_.cols);
    ConnectedComponents cc(component_image_, component_map_);
    num_components_ = cc.getNumberComponents();

  }

  void TopologicalMapper::computeGraph(double merge_threshold) {

    // First for each critical point, find out the neigbouring regions
    std::vector<std::set<uint32_t> > point_neighbour_sets;
    point_neighbour_sets.resize(critical_points_.size());

    // Draw the critical lines as their indexes on the map
    cv::Mat lines(map_resp_.map.info.height, map_resp_.map.info.width, CV_16UC1,
        cv::Scalar((uint16_t)-1));
    drawCriticalLines(lines, 0, 0, true);

    // Go over all the pixels in the lines image and find neighbouring critical
    // regions
    for (int j = 0; j < lines.rows; ++j) {
      uint16_t* image_row_j = lines.ptr<uint16_t>(j);
      for (int i = 0; i < lines.cols; ++i) {
        uint16_t pixel = image_row_j[i];
        if (pixel == (uint16_t)-1) {
          continue;
        }
        int x_offset[] = {0, -1, 1, 0};
        int y_offset[] = {-1, 0, 0, 1};
        size_t num_neighbours = 4;
        for (size_t count = 0; count < num_neighbours; ++count) {
          uint32_t x_n = i + x_offset[count];
          uint32_t y_n = j + y_offset[count];
          if (x_n >= (uint16_t) lines.cols || y_n >= (uint16_t) lines.rows) {
            continue;
          }
          size_t map_idx = MAP_IDX(lines.cols, x_n, y_n);
          if (component_map_[map_idx] >= 0 && 
              component_map_[map_idx] < (int32_t) num_components_) {
            point_neighbour_sets[pixel].insert(component_map_[map_idx]);
          }
        }
      }
    }

    // Throw out any critical points that do not have 2 adjoining regions.
    // This can happen if the regions are too small
    std::vector<std::vector<uint32_t> > point_neighbours;
    point_neighbours.resize(critical_points_.size());
    std::vector<int> remove_crit_points;
    for (size_t i = 0; i < critical_points_.size(); ++i) {
      if (point_neighbour_sets[i].size() == 2) {
        point_neighbours[i] = 
          std::vector<uint32_t>(point_neighbour_sets[i].begin(),
              point_neighbour_sets[i].end());
      } else {
        remove_crit_points.push_back(i);
      }
    }

    for (int i = remove_crit_points.size() - 1; i >=0 ; --i) {
      critical_points_.erase(critical_points_.begin() + i);
      point_neighbour_sets.erase(point_neighbour_sets.begin() + i);
      point_neighbours.erase(point_neighbours.begin() + i);
    }

    // Find connectivity to remove any isolated sub-graphs
    std::vector<bool> checked_region(critical_points_.size(), false);
    std::vector<std::set<int> > graph_sets;
    for (size_t i = 0; i < num_components_; ++i) {
      if (checked_region[i]) {
        continue;
      }
      std::set<int> open_set, closed_set;
      open_set.insert(i);
      while (open_set.size() != 0) {
        int current_region = *(open_set.begin());
        open_set.erase(open_set.begin());
        closed_set.insert(current_region);
        checked_region[current_region] = true;
        for (int j = 0; j < critical_points_.size(); ++j) {
          for (int k = 0; k < 2; ++k) {
            if (point_neighbours[j][k] == current_region && 
                std::find(closed_set.begin(), closed_set.end(), 
                  point_neighbours[j][1-k]) == 
                closed_set.end()) {
              open_set.insert(point_neighbours[j][1-k]);
            }
          }
        }
      }
      graph_sets.push_back(closed_set);
    }

    std::set<int> master_region_set;
    if (graph_sets.size() != 1) {
      std::cout << "WARNING: Master graph is fragmented into " <<
        graph_sets.size() << " sub-graphs!!!" << std::endl;
      int max_idx = 0;
      int max_size = graph_sets[0].size();
      for (size_t i = 1; i < graph_sets.size(); ++i) {
        if (graph_sets[i].size() > max_size) {
          max_idx = i;
          max_size = graph_sets[i].size();
        }
      }
      master_region_set = graph_sets[max_idx];
    } else {
      master_region_set = graph_sets[0];
    }
    
    // Create the region graph next
    std::map<int, int> region_to_vertex_map;
    int vertex_count = 0;
    for (size_t r = 0; r < num_components_; ++r) { 
      if (std::find(master_region_set.begin(), master_region_set.end(), r) ==
          master_region_set.end()) {
        region_to_vertex_map[r] = -1;
        continue;
      }

      Graph::vertex_descriptor vi = boost::add_vertex(region_graph_);

      // Calculate the centroid
      uint32_t avg_i = 0, avg_j = 0, pixel_count = 0;
      for (size_t j = 0; j < map_resp_.map.info.height; ++j) {
        for (size_t i = 0; i < map_resp_.map.info.width; ++i) {
          size_t map_idx = MAP_IDX(map_resp_.map.info.width, i, j);
          if (component_map_[map_idx] == (int)r) {
            avg_j += j;
            avg_i += i;
            pixel_count++;
          }
        }
      }

      region_graph_[vi].location.x = ((float) avg_i) / pixel_count;
      region_graph_[vi].location.y = ((float) avg_j) / pixel_count;
      region_graph_[vi].pixels = floor(sqrt(pixel_count));

      // This map is only required till the point we form edges on this graph 
      region_to_vertex_map[r] = vertex_count;

      ++vertex_count;
    }

    // Now that region_to_vertex_map is complete, 
    // forward critical points to next graph
    std::map<int, std::map<int, VoronoiPoint> > 
      region_vertex_crit_points;
    for (size_t r = 0; r < num_components_; ++r) { 
      if (std::find(master_region_set.begin(), master_region_set.end(), r) ==
          master_region_set.end()) {
        region_to_vertex_map[r] = -1;
        continue;
      }
      // Store the critical points corresponding to this vertex
      for (int j = 0; j < critical_points_.size(); ++j) {
        for (int k = 0; k < 2; ++k) {
          if (point_neighbours[j][k] == r) {
            int region_vertex = region_to_vertex_map[r];
            int other_region_vertex = region_to_vertex_map[point_neighbours[j][1-k]];
            if (region_vertex == 192 && other_region_vertex == 202) {
              std::cout << "192-202: " << r << " " << point_neighbours[j][1-k] << " " << critical_points_[j] << std::endl;
            }
            if (region_vertex == 202 && other_region_vertex == 192) {
              std::cout << "192-202: " << r << " " << point_neighbours[j][1-k] << " " << critical_points_[j] << std::endl;
            }
            region_vertex_crit_points[region_vertex]
              [region_to_vertex_map[point_neighbours[j][1-k]]] =  
                critical_points_[j];
          }
        }
      }
    }

    // Create 1 edge per critical point
    for (size_t i = 0; i < critical_points_.size(); ++i) {
      int region1 = region_to_vertex_map[point_neighbours[i][0]];
      int region2 = region_to_vertex_map[point_neighbours[i][1]];
      if (region1 == -1) {
        // part of one of the sub-graphs that was discarded
        continue;
      }
      int count = 0;
      Graph::vertex_descriptor vi,vj;
      vi = boost::vertex(region1, region_graph_);
      vj = boost::vertex(region2, region_graph_);
      Graph::edge_descriptor e; bool b;
      boost::tie(e,b) = boost::edge(vi, vj, region_graph_);
      if (!b) {
        boost::tie(e,b) = boost::add_edge(vi, vj, region_graph_);
      }
      /* boost::tie(e,b) = boost::add_edge(vi, vj, region_graph_); */
      // region_graph_[e].weight = bwi_mapper::getMagnitude(
      //     region_graph_[vi].location - region_graph_[vj].location);
    }

    // Refine the region graph into the point graph:
    int pixel_threshold = merge_threshold / map_resp_.map.info.resolution;

    enum {
      PRESENT = 0,
      REMOVED_REGION_VERTEX = 1,
      CONVERT_TO_CRITICAL_POINT = 2,
      MERGE_VERTEX = 3
    };
    Graph::vertex_iterator vi, vend;

    // PASS 1 - resolve all vertices that are too big and have more than 2 
    // critical points to their underlying critical points
    std::cout << std::endl << "==============================" << std::endl;
    std::cout << "PASS 1" << std::endl;
    std::cout << "==============================" << std::endl << std::endl;
    Graph pass_0_graph = region_graph_;
    Graph pass_1_graph;
    int pass_0_count = 0;
    int pass_1_count = 0;
    std::vector<int> pass_0_vertex_status(
        boost::num_vertices(pass_0_graph), PRESENT);
    std::map<int, int> pass_0_vertex_to_pass_1_map;
    for (boost::tie(vi, vend) = boost::vertices(pass_0_graph); vi != vend;
        ++vi, ++pass_0_count) {

      std::cout << "Analyzing pass 0 graph vertex: " << pass_0_count << std::endl;

      std::vector<size_t> adj_vertices;
      getAdjacentNodes(pass_0_count, pass_0_graph, adj_vertices);
      // See if the area of this region is too big to be directly pushed into 
      // the pass 3 graph
      if (pass_0_graph[*vi].pixels >= pixel_threshold &&
          adj_vertices.size() > 2) {
        pass_0_vertex_status[pass_0_count] = CONVERT_TO_CRITICAL_POINT;
        std::cout << " - throwing it out (needs to be resolved to CPs)" <<
          std::endl;
        continue;
      }

      // Otherwise insert this as is into the point graph
      Graph::vertex_descriptor point_vi = boost::add_vertex(pass_1_graph);
      pass_1_graph[point_vi] = pass_0_graph[*vi];
      pass_0_vertex_to_pass_1_map[pass_0_count] = pass_1_count;

      ++pass_1_count;
    }

    // Now for each vertex that needs to be resolved into critical points, 
    // check neighbours recursively to convert these vertices into critical
    // points
    std::map<int, std::map<int, int> > pass_0_vertex_to_cp_map;
    pass_0_count = 0;
    for (boost::tie(vi, vend) = boost::vertices(pass_0_graph); vi != vend;
        ++vi, ++pass_0_count) {

      if (pass_0_vertex_status[pass_0_count] != CONVERT_TO_CRITICAL_POINT) {
        continue;
      }
      std::cout << "Converting pass1 vtx to CPs: " << pass_0_count << std::endl;

      std::vector<size_t> adj_vertices;
      getAdjacentNodes(pass_0_count, pass_0_graph, adj_vertices);
      std::vector<int> connect_edges;
      BOOST_FOREACH(size_t vtx, adj_vertices) {
        if (pass_0_vertex_status[vtx] == PRESENT) {
          Graph::vertex_descriptor point_vi = boost::add_vertex(pass_1_graph);
          pass_1_graph[point_vi].location =
            region_vertex_crit_points[pass_0_count][vtx];
          std::cout << " - added vtx at " << pass_1_graph[point_vi].location 
            << std::endl;
          int pass_1_vertex = pass_0_vertex_to_pass_1_map[vtx];
          std::cout << " - connecting vtx to pass_1_vertex: " << pass_1_vertex
            << " (pass0 = " << vtx << ")" << std::endl;
          Graph::vertex_descriptor vj;
          vj = boost::vertex(pass_1_vertex, pass_1_graph);
          Graph::edge_descriptor e; bool b;
          boost::tie(e,b) = boost::add_edge(point_vi, vj, pass_1_graph);
          connect_edges.push_back(pass_1_count);
          ++pass_1_count;
        } else if (vtx > pass_0_count) {
          // The CP is shared, but has not been added
          Graph::vertex_descriptor point_vi = boost::add_vertex(pass_1_graph);
          pass_1_graph[point_vi].location =
            region_vertex_crit_points[pass_0_count][vtx];
          std::cout << " - added shared cp with " << vtx << " at " 
            << pass_1_graph[point_vi].location << std::endl;
          pass_0_vertex_to_cp_map[pass_0_count][vtx] = pass_1_count;
          connect_edges.push_back(pass_1_count);
          ++pass_1_count;
        } else {
          // Retrieve existing CP
          int cp_vtx = pass_0_vertex_to_cp_map[vtx][pass_0_count];
          connect_edges.push_back(cp_vtx);
        }
      }

      /* Connect all the edges */
      for (int i = 0; i < connect_edges.size(); ++i) {
        for (int j = 0; j < i; ++j) {
          Graph::vertex_descriptor vi, vj;
          vi = boost::vertex(connect_edges[i], pass_1_graph);
          vj = boost::vertex(connect_edges[j], pass_1_graph);
          Graph::edge_descriptor e; bool b;
          boost::tie(e,b) = boost::add_edge(vi, vj, pass_1_graph);
        }
      }

    }

    // Connect all the edges as is from pass 1
    pass_0_count = 0;
    for (boost::tie(vi, vend) = boost::vertices(pass_0_graph); vi != vend;
        ++vi, ++pass_0_count) {

      if (pass_0_vertex_status[pass_0_count] != PRESENT) {
        continue;
      }
      std::cout << "Adding pass 1 edges for: " << pass_0_count << std::endl;

      std::vector<size_t> adj_vertices;
      getAdjacentNodes(pass_0_count, pass_0_graph, adj_vertices);
      BOOST_FOREACH(size_t vtx, adj_vertices) {
        if (pass_0_vertex_status[vtx] == PRESENT && vtx < pass_0_count) {
          Graph::vertex_descriptor vi, vj;
          vi = boost::vertex(
              pass_0_vertex_to_pass_1_map[pass_0_count], pass_1_graph);
          vj = boost::vertex(
              pass_0_vertex_to_pass_1_map[vtx], pass_1_graph);
          Graph::edge_descriptor e; bool b;
          boost::tie(e,b) = boost::add_edge(vi, vj, pass_1_graph);
        }
      }
    }

    pass_1_graph_ = pass_1_graph;

    // PASS 2 - remove any vertices that are adjacent to only 2 other vertices,
    // where both those vertices are visible to each other
    std::cout << std::endl << "==============================" << std::endl;
    std::cout << "PASS 2" << std::endl;
    std::cout << "==============================" << std::endl << std::endl;

    Graph pass_2_graph;

    std::vector<int> pass_1_vertex_status(boost::num_vertices(pass_1_graph), PRESENT);
    std::map<int, int> pass_1_vertex_to_pass_2_vertex_map;
    std::vector<std::pair<int, int> > rr_extra_edges;
    std::map<int, std::pair<int, int> > removed_vertex_map;

    pass_1_count = 0;
    int pass_2_count = 0;
    for (boost::tie(vi, vend) = boost::vertices(pass_1_graph); vi != vend;
        ++vi, ++pass_1_count) {

      std::cout << "Analyzing pass_1 graph vertex: " << pass_1_count << std::endl;

      if (pass_1_vertex_status[pass_1_count] != PRESENT) {
        std::cout << " - the vertex has been thrown out already " << std::endl;
        continue;
      }
      
      // See if this only has 2 neighbours, and the 2 neighbours are visible to
      // each other
      std::vector<size_t> open_vertices;
      int current_vertex = pass_1_count;
      getAdjacentNodes(current_vertex, pass_1_graph, open_vertices);
      std::vector<int> removed_vertices;
      std::pair<int, int> edge;
      if (open_vertices.size() == 2 &&
          pass_1_vertex_status[open_vertices[0]] == PRESENT &&
          pass_1_vertex_status[open_vertices[1]] == PRESENT) {
        while(true) {
          std::cout << " - has 2 adjacent vertices " << open_vertices[0] << ", " 
            << open_vertices[1] << std::endl;
          // Check if the 2 adjacent vertices are visible
          Point2f location1 = 
            getLocationFromGraphId(open_vertices[0], pass_1_graph);
          Point2f location2 = 
            getLocationFromGraphId(open_vertices[1], pass_1_graph);
          bool locations_visible = 
            locationsInDirectLineOfSight(location1, location2, map_resp_.map);
          if (locations_visible) {
            std::cout << " - the 2 adjacent vertices are visible " 
              << "- removing vertex." << std::endl; 
            pass_1_vertex_status[current_vertex] = REMOVED_REGION_VERTEX;
            removed_vertices.push_back(current_vertex);
            edge = std::make_pair(open_vertices[0], open_vertices[1]);
            bool replacement_found = false;
            for (int i = 0; i < 2; ++i) {
              std::vector<size_t> adj_vertices;
              getAdjacentNodes(open_vertices[i], pass_1_graph, 
                  adj_vertices);
              if (adj_vertices.size() == 2) {
                size_t new_vertex = (adj_vertices[0] == current_vertex) ? 
                  adj_vertices[1] : adj_vertices[0];
                if (pass_1_vertex_status[new_vertex] == PRESENT) {
                  current_vertex = open_vertices[i];
                  std::cout << " - neighbours may suffer from the same problem"
                    << ". checking vertex " << current_vertex << std::endl;
                  open_vertices[i] = new_vertex;
                  replacement_found = true;
                  break;
                }
              }
            }
            if (!replacement_found) {
              // Both neighbours on either side had 1 or more than 2 adjacent 
              // vertices
              break;
            }
          } else {
            // left and right vertices not visible
            break;
          }
        }
      }

      BOOST_FOREACH(int removed_vertex, removed_vertices) {
        removed_vertex_map[removed_vertex] = edge;
      }

      if (removed_vertices.size() != 0) {
        // Add the extra edge
        std::cout << " - adding extra edge between " << edge.first <<
          " " << edge.second << std::endl;
        rr_extra_edges.push_back(edge);
        continue;
      }

      // Otherwise insert this as is into the point graph
      Graph::vertex_descriptor point_vi = boost::add_vertex(pass_2_graph);
      pass_2_graph[point_vi] = pass_1_graph[*vi];
      pass_1_vertex_to_pass_2_vertex_map[pass_1_count] = 
        pass_2_count;

      ++pass_2_count;
    }

    // Insert all edges that can be inserted
    
    // Add edges from the pass_1 graph that can be placed as is
    pass_1_count = 0;
    for (boost::tie(vi, vend) = boost::vertices(pass_1_graph); vi != vend;
        ++vi, ++pass_1_count) {
      if (pass_1_vertex_status[pass_1_count] == PRESENT) {
        std::vector<size_t> adj_vertices;
        getAdjacentNodes(pass_1_count, pass_1_graph, adj_vertices);
        BOOST_FOREACH(size_t adj_vertex, adj_vertices) {
          if (pass_1_vertex_status[adj_vertex] == PRESENT &&
              adj_vertex > pass_1_count) {
            Graph::vertex_descriptor vi,vj;
            int vertex1 = 
              pass_1_vertex_to_pass_2_vertex_map[pass_1_count];
            int vertex2 = pass_1_vertex_to_pass_2_vertex_map[adj_vertex];
            vi = boost::vertex(vertex1, pass_2_graph);
            vj = boost::vertex(vertex2, pass_2_graph);
            Graph::edge_descriptor e; bool b;
            boost::tie(e,b) = boost::add_edge(vi, vj, pass_2_graph);
          }
        }
      }
    }

    // Add all the extra edges
    typedef std::pair<int, int> int2pair;
    BOOST_FOREACH(const int2pair& edge, rr_extra_edges) {
      Graph::vertex_descriptor vi,vj;
      int vertex1 = 
        pass_1_vertex_to_pass_2_vertex_map[edge.first];
      int vertex2 = pass_1_vertex_to_pass_2_vertex_map[edge.second];
      vi = boost::vertex(vertex1, pass_2_graph);
      vj = boost::vertex(vertex2, pass_2_graph);
      Graph::edge_descriptor e; bool b;
      boost::tie(e,b) = boost::add_edge(vi, vj, pass_2_graph);
    }

    pass_2_graph_ = pass_2_graph;

    // Pass 3 - Merge all close vertices and remove any edges that can be
    // replaced by 2 edges of marginally longer length
    int vertex_merge_threshold = 2.0f / map_resp_.map.info.resolution;
    std::cout << std::endl << "==============================" << std::endl;
    std::cout << "PASS 3" << std::endl;
    std::cout << "==============================" << std::endl << std::endl;
    Graph pass_3_graph;
    pass_2_count = 0;
    int pass_3_count = 0;
    std::map<int, int> pass_2_vertex_to_pass_3_vertex_map;
    std::vector<int> pass_2_vertex_status(
        boost::num_vertices(pass_2_graph), PRESENT);
    for (boost::tie(vi, vend) = boost::vertices(pass_2_graph); vi != vend;
        ++vi, ++pass_2_count) {

      std::cout << "Analyzing pass 2 graph vtx: " << pass_2_count << std::endl;
      if (pass_2_vertex_status[pass_2_count] != PRESENT) {
        std::cout << " - vtx already thrown out " << std::endl;
        continue;
      }

      std::set<int> open_set;
      std::set<int> closed_set;
      open_set.insert(pass_2_count);
      while(open_set.size() != 0) {
        int current_vertex = *open_set.begin();
        open_set.erase(open_set.begin());
        closed_set.insert(current_vertex);
        std::vector<size_t> adj_vertices;
        getAdjacentNodes(current_vertex, pass_2_graph, adj_vertices);
        BOOST_FOREACH(size_t vtx, adj_vertices) {
          if (closed_set.count(vtx) || open_set.count(vtx)) {
            continue;
          }
          std::cout << " - checking " << current_vertex << ", " << vtx;
          Point2f l1 = getLocationFromGraphId(current_vertex, pass_2_graph);
          Point2f l2 = getLocationFromGraphId(vtx, pass_2_graph);
          float distance = getMagnitude(l1-l2);
          std::cout << " - " << distance;
          if (distance < vertex_merge_threshold) {
            open_set.insert(vtx);
            std::cout << " - WILL MERGE";
          }
          std::cout << std::endl;
        }
      }

      Point2f vertex_location = pass_2_graph[*vi].location;
      if (closed_set.size() != 1) {
        Point2f sum(0,0);
        BOOST_FOREACH(int vtx, closed_set) {
          pass_2_vertex_status[vtx] = MERGE_VERTEX;
          pass_2_vertex_to_pass_3_vertex_map[vtx] = pass_3_count;
          sum += getLocationFromGraphId(vtx, pass_2_graph); 
        }
        vertex_location = (1.0f / closed_set.size()) * sum;
      } else {
        pass_2_vertex_to_pass_3_vertex_map[pass_2_count] = pass_3_count;
      }
        
      Graph::vertex_descriptor point_vi = boost::add_vertex(pass_3_graph);
      pass_3_graph[point_vi].location = vertex_location;
      ++pass_3_count;
    }

    // Add edges from the pass_1 graph that can be placed as is
    pass_2_count = 0;
    for (boost::tie(vi, vend) = boost::vertices(pass_2_graph); vi != vend;
        ++vi, ++pass_2_count) {
      std::cout << "Gen edges pass 2 graph vtx: " << pass_2_count << std::endl;
      std::vector<size_t> adj_vertices;
      getAdjacentNodes(pass_2_count, pass_2_graph, adj_vertices);
      BOOST_FOREACH(size_t adj_vertex, adj_vertices) {
        if (adj_vertex > pass_2_count) {
          Graph::vertex_descriptor vi,vj;
          int vertex1 = pass_2_vertex_to_pass_3_vertex_map[pass_2_count];
          int vertex2 = pass_2_vertex_to_pass_3_vertex_map[adj_vertex];
          if (vertex1 != vertex2) {
            vi = boost::vertex(vertex1, pass_3_graph);
            vj = boost::vertex(vertex2, pass_3_graph);
            Graph::edge_descriptor e; bool b;
            boost::tie(e,b) = boost::edge(vi, vj, pass_3_graph);
            if (!b) {
              boost::tie(e,b) = boost::add_edge(vi, vj, pass_3_graph);
              pass_3_graph[e].weight = getMagnitude(
                  getLocationFromGraphId(vertex1, pass_3_graph) - 
                  getLocationFromGraphId(vertex2, pass_3_graph));
              std::cout << " - gen edge " << vertex1 << " " << vertex2 << std::endl;
            }
          }
        }
      }
    }

    pass_3_graph_ = pass_3_graph;

    // Now, nudge all vertices to see if you can increase visibility.
    pass_3_count = 0;
    int neighbourhood = 0.5 / map_resp_.map.info.resolution;
    int num_vertices_pass_3 = boost::num_vertices(pass_3_graph);
    for (boost::tie(vi, vend) = boost::vertices(pass_3_graph); vi != vend;
        ++vi, ++pass_3_count) {
      std::cout << "Attempting nudge on vtx: " << pass_3_count << std::endl;
      Point2f vtx_loc = getLocationFromGraphId(pass_3_count, pass_3_graph);
      int max_visibility = 0, best_i = vtx_loc.x, best_j = vtx_loc.y; 
      for (int k = 0; k < num_vertices_pass_3; ++k) {
        Point2f k_loc = getLocationFromGraphId(k, pass_3_graph);
        if (locationsInDirectLineOfSight(vtx_loc, k_loc, map_resp_.map)) {
          ++max_visibility;
        }
      }
      for (int j = vtx_loc.y - neighbourhood / 2; 
          j <= vtx_loc.y + neighbourhood / 2; ++j) {
        for (int i = vtx_loc.x - neighbourhood / 2; 
            i <= vtx_loc.x + neighbourhood / 2; ++i) {
          Point2f point_loc(i, j); 
          int visibility = 0;
          for (int k = 0; k < num_vertices_pass_3; ++k) {
            Point2f k_loc = getLocationFromGraphId(k, pass_3_graph);
            if (locationsInDirectLineOfSight(point_loc, k_loc, map_resp_.map)) {
              ++visibility;
            }
          }
          if (visibility > max_visibility) {
            max_visibility = visibility;
            best_i = i;
            best_j = j;
          }
        }
      }
      pass_3_graph[*vi].location = Point2f(best_i, best_j);
    }

    // Recompute edge weights
    pass_3_count = 0;
    for (boost::tie(vi, vend) = boost::vertices(pass_3_graph); vi != vend;
        ++vi, ++pass_3_count) {
      Point2f vtx1 = getLocationFromGraphId(pass_3_count, pass_3_graph);
      std::vector<size_t> adj_vertices;
      getAdjacentNodes(pass_3_count, pass_3_graph, adj_vertices);
      BOOST_FOREACH(size_t adj_vertex, adj_vertices) {
        if (adj_vertex > pass_3_count) {
          Point2f vtx2 = getLocationFromGraphId(adj_vertex, pass_3_graph);
          Graph::vertex_descriptor vi,vj;
          vi = boost::vertex(pass_3_count, pass_3_graph);
          vj = boost::vertex(adj_vertex, pass_3_graph);
          Graph::edge_descriptor e; bool b;
          boost::tie(e,b) = boost::edge(vi, vj, pass_3_graph);
          if (b) {
            pass_3_graph[e].weight = getMagnitude(vtx1 - vtx2);
          }
        }
      }
    }

    pass_4_graph_ = pass_3_graph;

    // Once all the edges have been added, remove edges that fail triangle inequality
    pass_3_count = 0;
    for (boost::tie(vi, vend) = boost::vertices(pass_3_graph); vi != vend;
        ++vi, ++pass_3_count) {
      std::cout << "Analyzing edges on vtx " << pass_3_count << std::endl;
      std::vector<size_t> adj_vertices;
      getAdjacentNodes(pass_3_count, pass_3_graph, adj_vertices);
      Point2f vtx1 = getLocationFromGraphId(pass_3_count, pass_3_graph);
      BOOST_FOREACH(size_t adj_vertex, adj_vertices) {
        if (adj_vertex < pass_3_count)
          continue;
        std::cout << " - analyzing edge between " << pass_3_count << " and " <<
          adj_vertex << std::endl;
        Point2f vtx2 = getLocationFromGraphId(adj_vertex, pass_3_graph);
        float edge_length = getMagnitude(vtx1 - vtx2);
        Graph::vertex_descriptor vi,vj;
        vi = boost::vertex(pass_3_count, pass_3_graph);
        vj = boost::vertex(adj_vertex, pass_3_graph);
        boost::remove_edge(vi, vj, pass_3_graph);
        std::vector<size_t> replacement_path;
        float path_cost = 
          getShortestPathWithDistance(pass_3_count, adj_vertex, replacement_path, pass_3_graph);
        if (replacement_path.size() >= 2) {
          std::cout << " - fount alternate path cost: " << path_cost << std::endl;
        }
        if (replacement_path.size() < 2 || path_cost  > 1.05 * edge_length) {
          // re-add edge to graph
          Graph::edge_descriptor e; bool b;
          boost::tie(e,b) = boost::add_edge(vi, vj, pass_3_graph);
          pass_3_graph[e].weight = getMagnitude(
              getLocationFromGraphId(pass_3_count, pass_3_graph) - 
              getLocationFromGraphId(adj_vertex, pass_3_graph));
        } else {
          std::cout << " - removing edge between " << pass_3_count << " and " <<
            adj_vertex << std::endl;
        }
      }
    }

    point_graph_ = pass_3_graph;
    
  }

  /**
   * \brief   Draws critical lines (4-connected) onto given image starting
   *          at (orig_x, orig_y)
   */
  void TopologicalMapper::drawCriticalLines(cv::Mat &image, 
      uint32_t orig_x, uint32_t orig_y, bool draw_idx, 
      bool visualization_image) {

    for (size_t i = 0; i < critical_points_.size(); ++i) {
      cv::Scalar color = cv::Scalar(0);
      if (draw_idx) {
        color = cv::Scalar((uint16_t)i);
      }
      VoronoiPoint &vp = critical_points_[i];
      if (vp.basis_points.size() != 2) {
        std::cerr << "ERROR: Found a critical point with more than 2 basis" 
          << "points. This should not have happened" << std::endl;
      } else {
        Point2d &p1(vp.basis_points[0]);
        Point2d &p2(vp.basis_points[1]);
        if (!visualization_image) {
          cv::line(image, 
              cv::Point(orig_x + p1.x, orig_y + p1.y),
              cv::Point(orig_x + p2.x, orig_y + p2.y), 
              color,
              1, 4); // draw a 4 connected line
        } else {
          color = cv::Scalar(0, 0, 255);
          cv::line(image, 
              cv::Point(orig_x + p1.x, orig_y + p1.y),
              cv::Point(orig_x + p2.x, orig_y + p2.y), 
              color,
              2, CV_AA); // draw a 4 connected line
        }
      }
    }
  }

} /* bwi_mapper */

