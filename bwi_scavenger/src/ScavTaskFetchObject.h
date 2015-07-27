

#ifndef SCAVTASKFETCHOBJECT_H
#define SCAVTASKFETCHOBJECT_H

#include <string>
#include <ros/ros.h> 
#include <ros/package.h>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>

#include "ScavTask.h"
#include "SearchPlanner.h"

class ScavTaskFetchObject : public ScavTask {
public:

    ScavTaskFetchObject();
    ~ScavTaskFetchObject() { delete gui_service_client; }

    ScavTaskFetchObject(ros::NodeHandle *node_handle, std::string path_of_dir); 

    void executeTask(int timeout, TaskResult &result, std::string &record); 
    void hriThread();
    void motionThread(); 

    bool random_walk_flag; 

    SearchPlanner *search_planner; 

    std::string object_name, room_name_from, room_name_to; 

    std::string directory; 

    ros::ServiceClient *gui_service_client; 

}; 

#endif
