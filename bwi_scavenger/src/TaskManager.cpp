
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <boost/lexical_cast.hpp>

#include "bwi_msgs/QuestionDialog.h"
#include "TaskManager.h"

TaskManager::TaskManager (ros::NodeHandle *nh) {
    this->nh = nh; 
    gui_client = nh->serviceClient <bwi_msgs::QuestionDialog> ("question_dialog");
}

void TaskManager::addTask(TaskWithStatus *task_with_status) {
    tasks.push_back(*task_with_status); 
    std::string str = task_with_status->task->task_name; 
    for (int i=0; i<task_with_status->task->task_parameters.size(); i++) {
        str += " " + task_with_status->task->task_parameters[i]; 
    }
    ROS_INFO_STREAM("Task added: " << str); 
}

TaskWithStatus* TaskManager::selectNextTask() {

    for (int i=0; i<tasks.size(); i++) {
        if (tasks[i].status == ONGOING) {
            return NULL; // robot works on one task at a time
        } else if (tasks[i].status == TODO) {
            ROS_INFO_STREAM("Start to work on: " << tasks[i].task->task_name); 
            tasks[i].status = ONGOING; 
            return & (tasks[i]); 
        }
    }
}

void TaskManager::executeNextTask(int timeout, TaskWithStatus *task_with_status) {
    TaskResult result;
    std::string record(""); 

    task_with_status->task->executeTask(timeout, result, record); 
    task_with_status->status = FINISHED; 
}

void TaskManager::updateStatusGui() {

    std::string message(""); 

    for (int i=0; i<tasks.size(); i++) {
        if (tasks[i].status == ONGOING) {
            message += "-->\t\t" + tasks[i].task->task_description; 
        } else if (tasks[i].status == TODO) {
            message += "\t\t" + tasks[i].task->task_description; 
        } else if (tasks[i].status == FINISHED) {
            message += "done\t" + tasks[i].task->task_description; 
        }

        for (int j=0; j<tasks[i].task->task_parameters.size(); j++) {
            message += " " + tasks[i].task->task_parameters[j]; 
        }
        message += "\n"; 
    }
    
    bwi_msgs::QuestionDialog srv; 

    srv.request.type = 0;
    srv.request.message = message; 

    gui_client.call(srv);
}


bool TaskManager::allFinished() {
    for (int i=0; i<tasks.size(); i++) {
        if (tasks[i].status != FINISHED)
            return false; 
    }
    return true; 
}

