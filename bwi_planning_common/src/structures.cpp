#include <boost/filesystem.hpp>
#include <fstream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

#include <bwi_planning_common/structures.h>
#include <tf/transform_datatypes.h> 

#ifdef HAVE_NEW_YAMLCPP
namespace YAML {
  // The >> operator disappeared in yaml-cpp 0.5, so this function is
  // added to provide support for code written under the yaml-cpp 0.3 API.
  template<typename T>
  void operator >> (const YAML::Node& node, T& i)
  {
    i = node.as<T>();
  }
}
#endif

namespace bwi_planning_common {
  
  void readLocationFile(const std::string& filename, 
      std::vector<std::string>& locations, std::vector<int32_t>& location_map) {

    if (!boost::filesystem::exists(filename)) {
      throw std::runtime_error("Location file does not exist: " + filename);
    }
    std::ifstream fin(filename.c_str());

    YAML::Node doc;
#ifdef HAVE_NEW_YAMLCPP
    doc = YAML::Load(fin);
#else
    YAML::Parser parser(fin);
    parser.GetNextDocument(doc);
#endif

    locations.clear();
    const YAML::Node &loc_node = doc["locations"];
    for (size_t i = 0; i < loc_node.size(); i++) {
      std::string location;
      loc_node[i] >> location;
      locations.push_back(location);
    }
    const YAML::Node &data_node = doc["data"];
    location_map.resize(data_node.size());
    for (size_t i = 0; i < data_node.size(); i++) {
      data_node[i] >> location_map[i];

    }
  }

  void readDoorFile(const std::string& filename, std::vector<Door>& doors) {

    if (!boost::filesystem::exists(filename)) {
      throw std::runtime_error("Door file does not exist: " + filename);
    }

    std::ifstream fin(filename.c_str());

    YAML::Node doc;
#ifdef HAVE_NEW_YAMLCPP
    doc = YAML::Load(fin);
#else
    YAML::Parser parser(fin);
    parser.GetNextDocument(doc);
#endif

    doors.clear();
    for (size_t i = 0; i < doc.size(); i++) {
      Door door;
      const YAML::Node &approach_node = doc[i]["approach"];
      for (size_t j = 0; j < 2; ++j) {
        approach_node[j]["from"] >> door.approach_names[j];
        approach_node[j]["point"][0] >> door.approach_points[j].x; 
        approach_node[j]["point"][1] >> door.approach_points[j].y; 
        approach_node[j]["point"][2] >> door.approach_yaw[j]; 
      }
      doc[i]["name"] >> door.name;
      try {
        doc[i]["width"] >> door.width;
      } catch(YAML::TypedKeyNotFound<std::string>& e) {
        door.width = 0.5;
      }
      doors.push_back(door);
    }
  }

  bool readObjectApproachFile(const std::string& filename, 
      std::map<std::string, geometry_msgs::Pose>& object_approach_map) {

    object_approach_map.clear();
    if (!boost::filesystem::exists(filename)) {

      return false;
    }

    std::ifstream fin(filename.c_str());

    YAML::Node doc;
#ifdef HAVE_NEW_YAMLCPP
    doc = YAML::Load(fin);
#else
    YAML::Parser parser(fin);
    parser.GetNextDocument(doc);
#endif

    for (size_t i = 0; i < doc.size(); i++) {
      std::string name;
      geometry_msgs::Pose pose;
      float yaw;
      doc[i]["name"] >> name;
      doc[i]["point"][0] >> pose.position.x;
      doc[i]["point"][1] >> pose.position.y;
      pose.position.z = 0;
      doc[i]["point"][2] >> yaw;
      tf::quaternionTFToMsg(                                                     
          tf::createQuaternionFromYaw(yaw), pose.orientation);
      object_approach_map[name] = pose;
    }

    fin.close();
    
    return true;
  }

  size_t resolveDoor(const std::string& door, const std::vector<Door>& doors) {
    
    for (size_t i = 0; i < doors.size(); ++i) {
      if (doors[i].name == door) {
        return i;
      }
    }

    return bwi_planning_common::NO_DOOR_IDX;
  }

} /* bwi_common */
