#include <boost/filesystem.hpp>
#include <fstream>
#include <libgen.h>
#include <stdexcept>
#include <stdio.h>
#include <yaml-cpp/yaml.h>

#include <bwi_planning_common/structures.h>
#include <tf/transform_datatypes.h> 

// We use SDL_image to load the image from disk
#include <SDL/SDL_image.h>

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

    // const YAML::Node &data_node = doc["data"];
    // location_map.resize(data_node.size());
    // for (size_t i = 0; i < data_node.size(); i++) {
    //   data_node[i] >> location_map[i];

    // }

    std::string mapfname;
    doc["data"] >> mapfname;                                                                                          
    if(mapfname.size() == 0) {                                                                                         
      std::string errmsg = "FATAL: The data tag cannot be an empty string.";
      throw std::runtime_error(errmsg);
    }                                                                                                                  
    if(mapfname[0] != '/') {                                                                                           
      // dirname can modify what you pass it                                                                           
      char* fname_copy = strdup(filename.c_str());                                                                        
      mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;                                                    
      free(fname_copy);                                                                                                
    }

    SDL_Surface* img;
    // Load the image using SDL.  If we get NULL back, the image load failed.
    if(!(img = IMG_Load(mapfname.c_str())))
    {
      std::string errmsg = std::string("failed to open image file \"") + mapfname + std::string("\"");
      throw std::runtime_error(errmsg);
    }

    // Get values that we'll need to iterate through the pixels
    int rowstride = img->pitch;
    int n_channels = img->format->BytesPerPixel;
    int avg_channels = (n_channels == 3 || n_channels == 1) ? n_channels : n_channels - 1;

    // Copy pixel data into the map structure
    unsigned char* pixels = (unsigned char*)(img->pixels);
    location_map.resize(img->h * img->w);

    for(int j = 0; j < img->h; j++)
    {
      for (int i = 0; i < img->w; i++)
      {
        // Compute mean of RGB for this pixel
        unsigned char* p = pixels + j*rowstride + i*n_channels;
        int color_sum = 0;
        for(int k = 0; k < avg_channels; k++)
          color_sum += *(p + (k));
        int color_avg = color_sum / (double)avg_channels;

        int location_idx = (color_avg != 255) ? color_avg : -1; 

        location_map[(img->h - j - 1) * img->w + i] = color_avg;
      }

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
      doc[i]["door_corner_pt_1"][0] >> door.door_corners[0].x; 
      doc[i]["door_corner_pt_1"][1] >> door.door_corners[0].y; 
      doc[i]["door_corner_pt_2"][0] >> door.door_corners[1].x; 
      doc[i]["door_corner_pt_2"][1] >> door.door_corners[1].y; 
      for (size_t j = 0; j < 2; ++j) {
        approach_node[j]["from"] >> door.approach_names[j];
        approach_node[j]["point"][0] >> door.approach_points[j].x; 
        approach_node[j]["point"][1] >> door.approach_points[j].y; 
        approach_node[j]["point"][2] >> door.approach_yaw[j]; 
        door.approach_yaw[j] += M_PI;
      }
      doc[i]["name"] >> door.name;

      door.door_center = 0.5 * (door.door_corners[0] + door.door_corners[1]);
      door.width = cv::norm(door.door_corners[0] - door.door_corners[1]);
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
      yaw += M_PI;
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
