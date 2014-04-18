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

#include <bwi_mapper/topological_mapper.h>
#include <bwi_mapper/map_utils.h>
#include <bwi_mapper/map_inflator.h>
#include <bwi_planning_common/structures.h>
#include <bwi_tools/filesystem.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using bwi_mapper::toGrid;
using bwi_mapper::toMap;

enum State {
  DOOR_CENTER = 0,
  DOOR_APPROACH = 1,
  DOOR_WIDTH = 2,
  DOOR_WAIT = 3,
} global_state = DOOR_CENTER;

cv::Point mouseover_pt;
bool increment_state = false;
void mouseCallback(int event, int x, int y, int, void*) {
  if (event == cv::EVENT_LBUTTONDOWN) {
    increment_state = true;
  } else if (event == cv::EVENT_MOUSEMOVE) {
    mouseover_pt = cv::Point(x, y);
  }
}

int main(int argc, char** argv) {

  if (argc < 4) {
    std::cerr << "USAGE: " << argv[0] 
        << " <yaml-map-file> <door-file> <location-file>" << std::endl;
    std::cout << "Use mark_locations to generate location file first." << std::endl;
    return -1;
  }

  std::string door_file(argv[2]);

  bwi_mapper::TopologicalMapper mapper(argv[1]);
  bwi_mapper::Graph graph;
  nav_msgs::MapMetaData info;
  mapper.getMapInfo(info);

  cv::Mat image;

  cv::namedWindow("Display", CV_WINDOW_AUTOSIZE);
  cv::setMouseCallback("Display", mouseCallback, 0);

  std::vector<bwi_planning_common::Door> doors;
  bwi_planning_common::Door current_door;
  cv::Point current_center_pt;

  std::vector<int32_t> location_map;
  std::vector<std::string> locations;

  std::cout << "Reading locations from file: " << 
    bwi::fs::canonicalize(argv[3]) << std::endl;
  std::cout << "Will write doors to file: " << 
    bwi::fs::canonicalize(door_file) << std::endl;

  std::cout << "First click selects door center." << std::endl <<
    "Second click selects the 2 approach points." << std::endl <<
    "Third click selects the door width." << std::endl <<
    "Press 'c' at any time to clear current selection." << std::endl <<
    "Press 'n' after thee clicks to accept door with name." << std::endl <<
    "Press 'w' to write door file and exit program." << std::endl;

  bwi_planning_common::readLocationFile(argv[3], locations, location_map);

  while (true) {

    mapper.drawMap(image,0,0);

    /* Handle drawing door related information */
    if (global_state == DOOR_CENTER) {
      cv::circle(image, mouseover_pt, 2, cv::Scalar(0,255,0), -1);
      // Save info
      current_center_pt = mouseover_pt;
    }
    if (global_state == DOOR_APPROACH) {
      cv::circle(image, current_center_pt, 3, cv::Scalar(0,0,255), -1);
      cv::circle(image, mouseover_pt, 2, cv::Scalar(0,255,0), -1); 
      cv::Point other_pt = (2.0f * current_center_pt) - mouseover_pt;
      cv::circle(image, other_pt, 2, cv::Scalar(0,255,0), -1); 
      // Compute info
      current_door.approach_points[0] = toMap(mouseover_pt, info);
      current_door.approach_points[1] = toMap(other_pt, info);
      int loc1_id = 
        location_map[MAP_IDX(image.cols, mouseover_pt.x, mouseover_pt.y)];
      int loc2_id = location_map[MAP_IDX(image.cols, other_pt.x, other_pt.y)];
      current_door.approach_names[0] = 
        (loc1_id != bwi_planning_common::INVALID_LOCATION) ? 
        locations[loc1_id] : "";
      current_door.approach_names[1] =
        (loc2_id != bwi_planning_common::INVALID_LOCATION) ? 
        locations[loc2_id] : "";
    } 
    if (global_state == DOOR_WIDTH) {
      bwi::Point2f difference_pt = 
        (current_door.approach_points[1] - current_door.approach_points[0]);
      cv::circle(image, toGrid(current_door.approach_points[0], info), 
          2, cv::Scalar(0,255,0), -1); 
      cv::circle(image, toGrid(current_door.approach_points[1], info),
          2, cv::Scalar(0,255,0), -1); 

      float approach_orientation = atan2f(difference_pt.y, difference_pt.x);
      float door_width_pxls = cv::norm(mouseover_pt - current_center_pt); 
      cv::Point door_add = door_width_pxls * 
        cv::Point2f(cosf(approach_orientation + M_PI / 2),
                    sinf(approach_orientation + M_PI / 2));
      cv::Point door_end_1 = current_center_pt + door_add;
      cv::Point door_end_2 = current_center_pt - door_add;
      cv::line(image, door_end_1, door_end_2, cv::Scalar(0,0,255), 2, 8);

      // Save info
      current_door.approach_yaw[0] = approach_orientation;
      current_door.approach_yaw[1] = approach_orientation + M_PI;
      current_door.width = door_width_pxls * info.resolution;
      
    }
    if (global_state == DOOR_WAIT) {
      bwi::Point2f difference_pt = 
        (current_door.approach_points[1] - current_door.approach_points[0]);
      cv::Point center_pt = toGrid(0.5 * 
          (current_door.approach_points[1] + current_door.approach_points[0]),
          info);
      cv::circle(image, toGrid(current_door.approach_points[0], info), 
          2, cv::Scalar(0,0,255), -1); 
      cv::circle(image, toGrid(current_door.approach_points[1], info),
          2, cv::Scalar(0,0,255), -1); 

      float approach_orientation = atan2f(difference_pt.y, difference_pt.x);
      float door_width_pxls = current_door.width / info.resolution;
      cv::Point door_add = door_width_pxls * 
        cv::Point2f(cosf(approach_orientation + M_PI / 2),
                    sinf(approach_orientation + M_PI / 2));
      cv::Point door_end_1 = center_pt + door_add;
      cv::Point door_end_2 = center_pt - door_add;
      cv::line(image, door_end_1, door_end_2, cv::Scalar(0,0,255), 2, 8);
      
    }
   
    for (size_t i = 0; i < doors.size(); ++i) {
      bwi::Point2f difference_pt = 
        (doors[i].approach_points[1] - doors[i].approach_points[0]);
      cv::Point center_pt = toGrid(0.5 * 
          (doors[i].approach_points[1] + doors[i].approach_points[0]),
          info);
      cv::circle(image, toGrid(doors[i].approach_points[0], info), 
          2, cv::Scalar(0,0,255), -1); 
      cv::circle(image, toGrid(doors[i].approach_points[1], info),
          2, cv::Scalar(0,0,255), -1); 

      float approach_orientation = atan2f(difference_pt.y, difference_pt.x);
      float door_width_pxls = doors[i].width / info.resolution;
      cv::Point door_add = door_width_pxls * 
        cv::Point2f(cosf(approach_orientation + M_PI / 2),
                    sinf(approach_orientation + M_PI / 2));
      cv::Point door_end_1 = center_pt + door_add;
      cv::Point door_end_2 = center_pt - door_add;
      cv::line(image, door_end_1, door_end_2, cv::Scalar(0,0,255), 2, 8);
    }

    cv::imshow("Display", image);

    unsigned char c = cv::waitKey(10);
    if (c == 27) { // Escape
      return 0;
    } else if (c == 'n' && global_state == DOOR_WAIT) {
      std::cout << "Enter door name: ";
      std::cin >> current_door.name;
      doors.push_back(current_door);
      global_state = DOOR_CENTER;
    } else if (c == 'c') {
      global_state = DOOR_CENTER;
    } else if (c == 'w') {
      std::stringstream door_ss;
      for (size_t i = 0; i < doors.size(); ++i) {
        bwi_planning_common::Door &door = doors[i];
        door_ss << "- name: " << door.name << std::endl;
        door_ss << "  approach:" << std::endl;
        for (size_t j = 0; j < 2; ++j) {
          bwi_mapper::Point2f grid_pt = toGrid(door.approach_points[j], info);
          size_t map_idx = 
            MAP_IDX(image.cols, (int) grid_pt.x, (int) grid_pt.y); 
          size_t loc_idx = location_map[map_idx];
          std::string from_loc = locations[loc_idx];
          door_ss << "    - from: " << from_loc << std::endl;
          door_ss << "      point: " << "[" << door.approach_points[j].x << 
            ", " << door.approach_points[j].y << ", " << door.approach_yaw[j] <<
            "]" << std::endl;
        }
      }
      std::ofstream door_fout(door_file.c_str());
      door_fout << door_ss.str();
      door_fout.close();
      std::cout << "Wrote doors to file: " << 
        bwi::fs::canonicalize(door_file) << std::endl;

      return 0;
    }

    if (increment_state) {
      switch (global_state) {
        case DOOR_CENTER: 
          global_state = DOOR_APPROACH;
          break;
        case DOOR_APPROACH: 
          global_state = DOOR_WIDTH;
          break;
        case DOOR_WIDTH: 
          global_state = DOOR_WAIT;
          break;
      }
      increment_state = false;
    }
  }
  return 0;
}




  
