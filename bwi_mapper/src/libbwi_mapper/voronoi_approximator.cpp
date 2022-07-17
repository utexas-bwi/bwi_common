/**
 * \file  voronoi_approximator.cpp
 * \brief  Implementation for the voronoi approximator
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
 * $ Id: 03/04/2013 02:44:42 PM piyushk $
 *
 **/

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include <bwi_mapper/voronoi_approximator.h>
#include <bwi_mapper/map_utils.h>
#include <bwi_mapper/point_utils.h>

#include <opencv2/opencv_modules.hpp>
#ifdef HAVE_OPENCV_IMGPROC
// this is OpenCV 3 and we need extra includes
#include <opencv2/imgproc/imgproc.hpp>
#endif

namespace bwi_mapper {

  /**
   * \brief   Computes the points that lie on the Voronoi Graph using an 
   *          approximate strategy useful for voronoi approximation during
   *          mapping
   * \param   threshold minimum obstacle distance in meters Any point which 
   *          is less than threshold away from an obstacle is rejected as a
   *          voronoi candidate. This allows for not caring about minor
   *          breaks in a wall which can happen for any SLAM algorithm
   */
  void VoronoiApproximator::findVoronoiPoints(double threshold, 
      bool is_naive, int sub_pixel_sampling) {

    // Get the inflated cost map
    inflateMap(threshold, map_resp_.map, inflated_map_);

    // Compute the voronoi points
    double pixel_threshold = 
      ceil(threshold / map_resp_.map.info.resolution);

    uint32_t max_dimension = 
      std::max(inflated_map_.info.height, inflated_map_.info.width);

    for (int j = 0; j < sub_pixel_sampling * (inflated_map_.info.height - 1); ++j) {
      if (j % sub_pixel_sampling == 0) {
        std::cout << "findVoronoiPoints(): On row : " << j / sub_pixel_sampling << std::endl;
      }
      for (int i = 0; i < sub_pixel_sampling * (inflated_map_.info.width - 1); 
          ++i) {

        Point2f center_pt((i + ((float)sub_pixel_sampling / 2)) / ((float)sub_pixel_sampling) + 0.001,
            (j + ((float)sub_pixel_sampling / 2)) / ((float)sub_pixel_sampling) + 0.001);

        // if (center_pt.x < inflated_map_.info.width / 2 ||
        //     i >= inflated_map_.info.width / 2 + 1)
        //   continue;
        // if (j < inflated_map_.info.height / 2 ||
        //     j >= inflated_map_.info.height / 2 + 1)
        //   continue;

        VoronoiPoint vp(lrint(center_pt.x), lrint(center_pt.y));

        // Check if this location is too close to a given obstacle
        uint32_t map_idx = MAP_IDX(inflated_map_.info.width, vp.x, vp.y);
        if (inflated_map_.data[map_idx] != 0) {
          continue;
        }

        /* std::cout << "Checking " << center_pt << ", will mark " << vp << std::endl; */

        // Use the boxes to find obstacles
        for (int box = pixel_threshold; box < max_dimension; ++box) {

          // Early termination crit - no more hope of finding more obstacles
          // at the same distance as the one found already
          if (vp.basis_points.size() != 0 &&
              box > vp.basis_distance + 1) {
            break;
          }

          // Get obstacles at this box size
          std::vector<Point2d> obstacles;
          int low_j = lrint(center_pt.y - box - 1);
          int high_j = lrint(center_pt.y + box);
          int low_i = lrint(center_pt.x - box - 1);
          int high_i = lrint(center_pt.x + box);

          /* std::cout << " - box " << box << " [" << low_i << "," << low_j << "," << high_i << "," << high_j << "]" << std::endl;  */

          // If we hit the side of the images + 1, we've seen all possible
          // sites
          if (low_i < 0 || low_j < 0 || 
              high_i >= inflated_map_.info.width 
              || high_j >= inflated_map_.info.height) {
            break;
          }

          // Corners of the box + vertical edges
          for (int j_box = low_j; j_box <= high_j; ++j_box) {
            for (int i_box = low_i; i_box <= high_i; 
                i_box += high_i - low_i) {
              uint32_t map_idx_box = 
                MAP_IDX(inflated_map_.info.width, i_box, j_box);
              bool occupied = map_resp_.map.data[map_idx_box];
              if (occupied) {
                Point2d p(i_box, j_box);
                Point2f f(i_box + 0.5, j_box + 0.5);
                p.distance_from_ref = bwi_mapper::getMagnitude(f - center_pt);
                obstacles.push_back(p);
              }
            }
          }

          // horizontal edges
          for (int j_box = low_j; j_box <= high_j;
              j_box += high_j - low_j) {
            for (int i_box = low_i + 1; i_box < high_i + 1; ++i_box) {
              uint32_t map_idx_box = 
                MAP_IDX(inflated_map_.info.width, i_box, j_box);
              bool occupied = map_resp_.map.data[map_idx_box];
              if (occupied) {
                Point2d p(i_box, j_box);
                Point2f f(i_box + 0.5, j_box + 0.5);
                p.distance_from_ref = bwi_mapper::getMagnitude(f - center_pt);
                obstacles.push_back(p);
              }
            }
          }

          // Check if any obstacles are found
          if (obstacles.size() == 0) {
            continue;
          }

          // Now that obstacles are available, sort and add to the vp as
          // necessary
          if (!is_naive) {
            std::sort(obstacles.begin(), obstacles.end(), 
                Point2dDistanceComp());
          }
          for (size_t q = 0; q < obstacles.size(); q++) {
            vp.addBasisCandidate(obstacles[q], pixel_threshold, 
                inflated_map_, is_naive);
          }
        }

        if (vp.basis_points.size() >= 2) {
          voronoi_points_.push_back(vp);
        }
        
      }
    }

    // Compute average basis distance for each voronoi point
    for (size_t i = 0; i < voronoi_points_.size(); ++i) {
      VoronoiPoint &vp = voronoi_points_[i];
      float basis_distance_sum = 0;
      for (size_t j = 0; j < vp.basis_points.size(); ++j) {
        basis_distance_sum += vp.basis_points[j].distance_from_ref;
      }
      vp.average_clearance = basis_distance_sum / vp.basis_points.size();
      std::cout << "Found VP at " << vp << " with clearance " << vp.average_clearance << std::endl;
    }

    // Label the voronoi diagram as being available
    initialized_ = true;
  }

  /**
   * \brief   Draws the base map and voronoi points on to image. Should be
   *          only used for testing the output for Voronoi Approximator.
   * \param   image OpenCV image we are drawing output on 
   */
  void VoronoiApproximator::drawOutput(cv::Mat &image) {
    if (!initialized_) {
      throw std::runtime_error("drawOutput(): voronoi diagram not "
          "initialized, call findVoronoiPoints first");
    }
    drawMap(image);
    drawMap(image, map_resp_.map.info.width);
    drawVoronoiPoints(image, map_resp_.map.info.width);
  }

  /**
   * \brief   Prints information about all the voronoi points to screen. 
   *          Used for testing the output for Voronoi Approximator.
   */
  void VoronoiApproximator::printVoronoiPoints() {
    for (size_t i = 0; i < voronoi_points_.size(); ++i) {
      VoronoiPoint &vp = voronoi_points_[i];
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
   * \brief   Draw voronoi points onto image starting at (orig_x, orig_y)
   * \param   image OpenCV image we are writing the map onto
   */
  void VoronoiApproximator::drawVoronoiPoints(cv::Mat &image, 
      uint32_t orig_x, uint32_t orig_y) {
    for (size_t i = 0; i < voronoi_points_.size(); ++i) {
      VoronoiPoint &vp = voronoi_points_[i];
      cv::Point p(vp.x + orig_x, orig_y + vp.y);
      cv::circle(image, p, 1, cv::Scalar(255,0,0), -1, CV_AA);
      // image.at<cv::Vec3b>
      //   (orig_y + vp.y, vp.x + orig_x) = 
      //   cv::Vec3b(255,0,0);
    }
  }

} /* bwi_mapper */
