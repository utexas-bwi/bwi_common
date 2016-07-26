
#ifndef TASKMANAGER_H
#define TASKMANAGER_H

#include <ros/ros.h>
#include <vector>
#include <string>

#include "bwi_msgs/ScavStatus.h"
#include "ScavTask.h"

enum TaskStatus { ONGOING, FINISHED, TODO }; 

struct TaskWithStatus {
    ScavTask *task; 
    TaskStatus status; 
    std::string certificate; 

    TaskWithStatus() : task(NULL), status(TODO) {}

    TaskWithStatus(ScavTask *scav_task, TaskStatus scav_status) : 
        task(scav_task), status(scav_status) {}
}; 

class TaskManager {

public:

    TaskManager() {}
    TaskManager(ros::NodeHandle *); 

    ros::NodeHandle *nh; 

    ros::ServiceClient gui_client; 

    std::vector<TaskWithStatus> tasks;

    void addTask(TaskWithStatus* task); 

    void executeNextTask(int timeout, TaskWithStatus *task); 

    void updateStatusGui(); 

    void publishStatus(); 

    bool allFinished(); 

    TaskWithStatus *selectNextTask(); 

    bool paused; 
private:
    
    ros::Publisher pub; 
    bwi_msgs::ScavStatus msg; 



}; 


#endif
