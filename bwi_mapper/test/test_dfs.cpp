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

#include <bwi_mapper/voronoi_approximator.h>

int main(int argc, char** argv) {

  nav_msgs::OccupancyGrid grid;
  grid.info.height = 5;
  grid.info.width = 5;
  grid.data.resize(25);
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      grid.data[5*j + i] = 0;
    }
  }

  grid.data[5*1 + 1] = 100;
  grid.data[5*2 + 1] = 100;
  grid.data[5*3 + 1] = 100;

  grid.data[5*1 + 3] = 100;
  grid.data[5*2 + 3] = 100;
  grid.data[5*3 + 3] = 100;

  bwi_mapper::DirectedDFS dfs(grid);
  bwi_mapper::Point2d p1, p2;
  p1.x = 1, p1.y = 1;
  p2.x = 1, p2.y = 3;
  std::cout << dfs.searchForPath(p1, p2, 5) << std::endl; // 1
  p2.x = 3, p2.y = 3;
  std::cout << dfs.searchForPath(p1, p2, 5) << std::endl; // 0

  grid.data[5*4 + 2] = 100;
  std::cout << dfs.searchForPath(p1, p2, 4) << std::endl; // 1
  std::cout << dfs.searchForPath(p1, p2, 3) << std::endl; // 0

  return 0;
}


