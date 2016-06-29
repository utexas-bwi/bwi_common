#ifndef MULTI_LEVEL_MAP_UTILS
#define MULTI_LEVEL_MAP_UTILS

#include <iostream>
#include <cassert>
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>

namespace multi_level_map {

  std::string frameIdFromLevelId(std::string level_id);
  std::string levelIdFromFrameId(std::string frame_id);
}

#endif
