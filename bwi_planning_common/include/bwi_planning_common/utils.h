#ifndef BWI_PLANNING_COMMON_UTILS_H_
#define BWI_PLANNING_COMMON_UTILS_H_

#include <string>

namespace bwi_planning_common {

  inline std::string getDoorsFileLocationFromDataDirectory(const std::string& data_directory) {
    return data_directory + "/doors.yaml"
  }

  inline std::string getDoorsMapLocationFromDataDirectory(const std::string& data_directory) {
    return data_directory + "/doors_map.yaml"
  }

  inline std::string getLocationsFileLocationFromDataDirectory(const std::string& data_directory) {
    return data_directory + "/locations.yaml"
  }

  inline std::string getObjectsFileLocationFromDataDirectory(const std::string& data_directory) {
    return data_directory + "/objects.yaml"
  }

  inline std::string getLocationsImageFileLocationFromDataDirectory(const std::string& data_directory) {
    return data_directory + "/locations.pgm"
  }

} /* bwi_planning_common */

#endif /* end of include guard: BWI_PLANNING_COMMON_UTILS_H_ */
