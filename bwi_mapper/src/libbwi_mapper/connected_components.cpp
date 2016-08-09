/**
 * \file  connected_components.h
 * \brief  Implementation for Connected components. Uses cvblobslib internally.
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
 * $ Id: 03/04/2013 12:28:10 PM piyushk $
 *
 **/

#include <bwi_mapper/connected_components.h>
#include <bwi_mapper/map_utils.h>

#include <opencv2/opencv_modules.hpp>
#ifdef HAVE_OPENCV_IMGPROC
// this is OpenCV 3 and we need extra includes
#include <opencv2/imgproc/imgproc.hpp>
#endif

namespace bwi_mapper {

  /**
   * \brief  Given an image map containing obstacles and demarcating
   *         critical lines, the constructor runs a critical components
   *         algorithm to find critical regions.
   * \param  image map with obstacles and critical lines 
   * \return  
   */
  ConnectedComponents::ConnectedComponents (const cv::Mat& image,
      std::vector<int32_t>& component_map) {

    cv::Mat src = image.clone();
    cv::Mat dst = cv::Mat::zeros(src.rows, src.cols, CV_32S);
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(src, contours, hierarchy, 
        CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    number_components_ = 0;
    for(int idx = 0; idx >= 0; idx = hierarchy[idx][0]) {
      cv::Scalar color(number_components_ + 1);
      cv::drawContours(dst, contours, idx, color, CV_FILLED, 8, hierarchy);
      number_components_++;
    }

    component_map.clear();
    component_map.resize(dst.rows * dst.cols);

    size_t map_idx = 0;
    for (int j = 0; j < dst.rows; ++j) {
      const int32_t* dst_row = dst.ptr<int32_t>(j);
      for (int i = 0; i < dst.cols; ++i) {
        component_map[map_idx] = dst_row[i] - 1;
        ++map_idx;
      }
    }

  }

  /**
   * \brief Returns the number of components obtained by running the
   *        critical components algorithm.
   */
  size_t ConnectedComponents::getNumberComponents() {
    return number_components_;
  }

} /* bwi_mapper */
