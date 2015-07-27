#ifndef SCAVTASKWHITEBOARD_H
#define SCAVTASKWHITEBOARD_H

#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include "ScavTask.h"
#include "SearchPlanner.h"

struct Pose { 
    float x; float y; 
    Pose() {}
    Pose(float xx, float yy) : x(xx), y(yy) {}
}; 

class ScavTaskWhiteBoard : public ScavTask {
public:

    ScavTaskWhiteBoard() {}
    ScavTaskWhiteBoard(ros::NodeHandle *node_handle, std::string path_of_dir); 

    void executeTask(int timeout, TaskResult &result, std::string &record); 
    void visionThread();
    void motionThread(); 

    void callback_human_detected(const geometry_msgs::PoseStamped::ConstPtr& msg); 
    void callback_image(const sensor_msgs::ImageConstPtr& msg); 

    SearchPlanner *search_planner; 
    std::string directory;

    bool inRectangle(Pose p, Pose top_left, Pose top_right, Pose bottom_left); 
}; 

#endif
