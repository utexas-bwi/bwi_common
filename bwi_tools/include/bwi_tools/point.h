#ifndef BWI_TOOLS_POINT_H
#define BWI_TOOLS_POINT_H

#include <opencv/cv.h>

namespace bwi {

  typedef cv::Point2f Point2f;

  inline float getMagnitude(Point2f p) {
    return cv::norm(p);
  }
  
} /* bwi */

#endif /* end of include guard: BWI_TOOLS_POINT_H */
