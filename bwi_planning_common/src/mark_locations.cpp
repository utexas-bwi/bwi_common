/**
 * \file  create_experiment.cpp
 * \brief  Create a single experiment in a series of experiments for the bwi_web
 *         experiments
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
 * $ Id: 03/23/2013 09:07:41 PM piyushk $
 *
 **/

#include <fstream>


#include <bwi_mapper/map_loader.h>
#include <bwi_planning_common/structures.h>
#include <bwi_tools/filesystem.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

cv::Point clicked_pt, release_pt, mouseover_pt;
bool button_clicked = false;
bool button_released = false;
void mouseCallback(int event, int x, int y, int, void*) {
  if (event == cv::EVENT_LBUTTONDOWN) {
    button_clicked = true;
    clicked_pt = cv::Point(x, y);
  } else {
    button_clicked = false;
  }

  if (event == cv::EVENT_LBUTTONUP) {
    button_released = true;
    release_pt = cv::Point(x, y);
  } else {
    button_released = false;
  }

  if (event == cv::EVENT_MOUSEMOVE) {
    mouseover_pt = cv::Point(x, y);
  }
}

int main(int argc, char** argv) {

  if (argc < 3) {
    std::cerr << "USAGE: " << argv[0] 
        << " <yaml-map-file> <location-file>" << std::endl;
    return -1;
  }

  std::string location_file(argv[2]);

  bwi_mapper::MapLoader mapper(argv[1]);
  nav_msgs::MapMetaData info;
  mapper.getMapInfo(info);

  cv::Mat image;

  cv::namedWindow("Display", CV_WINDOW_AUTOSIZE);
  cv::setMouseCallback("Display", mouseCallback, 0);

  std::vector<int> component_map(info.height * info.width, -1);
  std::vector<std::string> locations;
  int current_location_id = 0;

  bool region_under_consideration = false;
  bool region_awaiting_approval = false;
  cv::Point start_pt;
  cv::Point end_pt;

  std::cout << "Will write locations to file: " << 
    bwi::fs::canonicalize(location_file) << std::endl;
  std::cout << "Select rectangular region using click + drag." << std::endl <<
    "Press 'a' to accept selection into current location." << std::endl <<
    "Press 'n' to accept current location with name." << std::endl <<
    "Press 'w' to write locations file and exit program." << std::endl;

  while (true) {

    mapper.drawMap(image,0,0);
    for (int i = 0; i < image.rows * image.cols; ++i) {
      int cid = component_map[i];
      if (cid == -1) {
        continue;
      }
      cv::Vec3b color(0, 0, 64); //red
      cv::Vec3b anti_color(64, 64, 0); //anti-red
      if (cid != current_location_id) { // change color to anti-blue
        color = cv::Vec3b(64, 0, 0);
        anti_color = cv::Vec3b(0, 64, 64);
      }
      int x = i % image.cols;
      int y = i / image.cols;
      if (image.at<cv::Vec3b>(y, x)[0] == 0)  {
        image.at<cv::Vec3b>(y,x) += color;
      } else {
        image.at<cv::Vec3b>(y,x) -= anti_color;
      }
    }

    if (region_under_consideration || region_awaiting_approval) {
      cv::Point final_pt = mouseover_pt;
      if (region_awaiting_approval) {
        final_pt = end_pt;
      }
      int start_x = (start_pt.x < final_pt.x) ? start_pt.x : final_pt.x;
      int start_y = (start_pt.y < final_pt.y) ? start_pt.y : final_pt.y;
      int end_x = (start_pt.x > final_pt.x) ? start_pt.x : final_pt.x;
      int end_y = (start_pt.y > final_pt.y) ? start_pt.y : final_pt.y;
      for (int y = start_y; y <= end_y; ++y) {
        for (int x = start_x; x <= end_x; ++x) {
          cv::Vec3b color(0, 64, 0); //green
          cv::Vec3b anti_color(64, 0, 64); //anti-green
          if (image.at<cv::Vec3b>(y, x)[0] == 0)  {
            image.at<cv::Vec3b>(y,x) += color;
          } else {
            image.at<cv::Vec3b>(y,x) -= anti_color;
          }

        }
      }
    }


    cv::imshow("Display", image);

    unsigned char c = cv::waitKey(10);

    if (button_clicked) {
      region_under_consideration = true;
      region_awaiting_approval = false;
      start_pt = clicked_pt;
    }

    if (button_released) {
      end_pt = mouseover_pt;
      region_awaiting_approval = true;
      region_under_consideration = false;
    }
       
    if (c == 27) {
      return 0;
    } else if (c == 'a' && region_awaiting_approval) {
      /* approve region */
      int start_x = (start_pt.x < end_pt.x) ? start_pt.x : end_pt.x;
      int start_y = (start_pt.y < end_pt.y) ? start_pt.y : end_pt.y;
      int end_x = (start_pt.x > end_pt.x) ? start_pt.x : end_pt.x;
      int end_y = (start_pt.y > end_pt.y) ? start_pt.y : end_pt.y;
      for (int y = start_y; y <= end_y; ++y) {
        for (int x = start_x; x <= end_x; ++x) {
          component_map[y * image.cols + x] = current_location_id;
        }
      }
      region_awaiting_approval = false;
    } else if (c == 'n') {
      std::string location_string;
      std::cout << "Enter location name: ";
      std::cin >> location_string;
      locations.push_back(location_string);
      current_location_id++;
    } else if (c == 'w') {
      std::stringstream location_ss;
      location_ss << "locations:" << std::endl;
      for (size_t i = 0; i < locations.size(); ++i) {
        std::string &loc = locations[i];
        location_ss << "  - " << loc << std::endl;
      }
      location_ss << "data: [" << std::endl;
      for (size_t x = 0; x < component_map.size(); ++x) {
        location_ss << component_map[x];
        if (x != component_map.size() - 1)
          location_ss << ", ";
      }
      location_ss << "]" << std::endl;

      std::ofstream location_fout(location_file.c_str());
      location_fout << location_ss.str();
      location_fout.close();
      std::cout << "Wrote locations to file: " <<
        bwi::fs::canonicalize(location_file) << std::endl;
      return 0;
    } 
  }

  return 0;
}




  
