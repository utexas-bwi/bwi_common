/**
 * \file  test_voronoi.cpp
 * \brief  Simple test for the voronoi approximator. Reads a map and displays 
 * information from the voronoi approximator on to the screen
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
 * $ Id: 02/20/2013 05:09:13 PM piyushk $
 *
 **/

#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <bwi_mapper/voronoi_approximator.h>

int main(int argc, char** argv) {

  if (argc != 2) {
    std::cerr << "USAGE: " << argv[0] << " <yaml-map-file>" << std::endl;
    return -1;
  }

  cv::Mat display_image, image;
  nav_msgs::OccupancyGrid map;
  std::cout << "Computing Voronoi Points using new approach" << std::endl;
  bwi_mapper::VoronoiApproximator voronoi(argv[1]);
  voronoi.findVoronoiPoints(0.3);
  voronoi.getMap(map);
  voronoi.drawMap(display_image, map, 0, 0);
  voronoi.drawVoronoiPoints(display_image, 0, 0);
  voronoi.drawMap(image);
  voronoi.drawVoronoiPoints(image);
  cv::imwrite("voronoi_new.png", image);

  std::cout << "Computing Voronoi Points using naive approach and no sub pixel sampling" << std::endl;
  bwi_mapper::VoronoiApproximator voronoi2(argv[1]);
  voronoi2.findVoronoiPoints(0.3, true, 1);
  voronoi2.drawMap(display_image, map, map.info.width, 0);
  voronoi2.drawVoronoiPoints(display_image, map.info.width, 0);
  voronoi2.drawMap(image);
  voronoi2.drawVoronoiPoints(image);
  cv::imwrite("voronoi_naive_1.png", image);

  std::cout << "Computing Voronoi Points using naive approach and sub pixel sampling of 2" << std::endl;
  bwi_mapper::VoronoiApproximator voronoi3(argv[1]);
  voronoi3.findVoronoiPoints(0.3, true, 2);
  voronoi3.drawMap(display_image, map, 2*map.info.width, 0);
  voronoi3.drawVoronoiPoints(display_image, 2*map.info.width, 0);
  voronoi3.drawMap(image);
  voronoi3.drawVoronoiPoints(image);
  cv::imwrite("voronoi_naive_2.png", image);

  std::cout << "Computing Voronoi Points using naive approach and sub pixel sampling of 4" << std::endl;
  bwi_mapper::VoronoiApproximator voronoi4(argv[1]);
  voronoi4.findVoronoiPoints(0.3, true, 4);
  voronoi4.drawMap(display_image, map, 3*map.info.width, 0);
  voronoi4.drawVoronoiPoints(display_image, 3*map.info.width, 0);
  voronoi4.drawMap(image);
  voronoi4.drawVoronoiPoints(image);
  cv::imwrite("voronoi_naive_4.png", image);

  cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
  cv::imshow("Display window", display_image);                

  cv::waitKey(0); // Wait for a keystroke in the window
  return 0;
}


