
#ifndef TASKMANAGER_H
#define TASKMANAGER_H

#include <ros/ros.h>
#include <vector>
#include <string>

#include "ScavTask.h"

enum TaskStatus { ONGOING, FINISHED, TODO }; 

struct TaskWithStatus {
    ScavTask *task; 
    TaskStatus status; 

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
    TaskWithStatus *selectNextTask(); 
    void updateStatusGui(); 
    bool allFinished(); 

}; 


#endif
