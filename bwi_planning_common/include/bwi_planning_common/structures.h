#ifndef STRUCTURES_9TKSKPHM
#define STRUCTURES_9TKSKPHM

#include <bwi_tools/point.h>
#include <stdint.h>
#include <string>
#include <yaml-cpp/yaml.h>

#include <geometry_msgs/Pose.h>

namespace bwi_planning_common {

  void readLocationFile(const std::string& filename, 
      std::vector<std::string>& locations, std::vector<int32_t>& location_map);

  class Door {
    public:
      std::string name;
      std::string approach_names[2];
      bwi::Point2f approach_points[2];
      float approach_yaw[2];
      float width;
  };

  void readDoorFile(const std::string& filename, std::vector<Door>& doors);
  size_t resolveDoor(const std::string& door, const std::vector<Door>& doors);

  bool readObjectApproachFile(const std::string& filename, 
      std::map<std::string, geometry_msgs::Pose>& object_approach_map);

  const size_t NO_DOOR_IDX = (size_t) -1; 
  const int INVALID_LOCATION = -1;

} /* bwi_planning_common */

#endif /* end of include guard: STRUCTURES_9TKSKPHM */
