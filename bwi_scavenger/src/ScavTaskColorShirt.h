
#ifndef SCAVTASKCOLORSHIRT_H
#define SCAVTASKCOLORSHIRT_H

#include <ros/ros.h>
#include <ros/package.h>
// #include <opencv2/improc/improc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include "ScavTask.h"
#include "SearchPlanner.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

enum Color{ RED, GREEN, BLUE, YELLOW, ORANGE, COLOR_LENGTH}; 

struct Rgb {
    float r; float g; float b;        
    Rgb() : r(), g(), b() {}
    Rgb( float rr, float gg, float bb ) : r(rr), g(gg), b(bb) {}
};

class ScavTaskColorShirt : public ScavTask {

public:
 
    ScavTaskColorShirt(); 

    ScavTaskColorShirt(ros::NodeHandle *node_handle, std::string path_of_dir, Color shirt_color); 

    void executeTask(int timeout, TaskResult &result, std::string &record); 
    void visionThread();
    void motionThread(); 

    SearchPlanner *search_planner; 

    Color color; 
    std::string directory; 

    void callback_image_saver(const sensor_msgs::ImageConstPtr& msg); 
    void callback_human_detection(const PointCloud::ConstPtr& msg); 

    // to compute the distance between a Rgb struct and a PCL RGB
    float getColorDistance(const pcl::PointXYZRGB *c1, const Rgb *c2) 
    {
        return pow(pow(c1->r- c2->r, 2.0) + pow(c1->g - c2->g, 2.0) + pow(c1->b - c2->b, 2.0), 0.5);
    }

}; 

#endif
