#ifndef SCAVTASKHUMANFOLLOWING_H
#define SCAVTASKHUMANFOLLOWING_H

#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include "ScavTask.h"
#include "SearchPlanner.h"

namespace scav_task_human_following {

class ScavTaskHumanFollowing : public ScavTask {
public:

    ScavTaskHumanFollowing() {}
    ScavTaskHumanFollowing(ros::NodeHandle *node_handle, std::string path_of_dir); 

    void executeTask(int timeout, TaskResult &result, std::string &record); 
    void visionThread();
    void motionThread(); 

    void callback_human_detected(const geometry_msgs::PoseStamped::ConstPtr& msg); 
    void callback_image(const sensor_msgs::ImageConstPtr& msg); 

    void moveToPose(const geometry_msgs::Pose); 

    SearchPlanner *search_planner; 
    std::string directory;
    ros::Publisher pub_simple_goal; 

    bool task_completed; 
}; 
}
#endif
