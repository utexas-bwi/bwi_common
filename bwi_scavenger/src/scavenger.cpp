
#include <ros/ros.h>
#include <ros/package.h>

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include "ScavTask.h"
#include "TaskManager.h"
#include "SearchPlanner.h"

#include "ScavTaskColorShirt.h"
#include "ScavTaskWhiteBoard.h"
#include "ScavTaskFetchObject.h"
#include "ScavTaskHumanFollowing.h"

#define TIMEOUT (600) // return failure if not finished in 10 minutes
#define NUM_OF_TASKS (16)
#define NUM_OF_TASK_TYPES (4)

using namespace scav_task_human_following; 

int main(int argc, char **argv) {

    ros::init(argc, argv, "scavenger");
    ros::NodeHandle *nh = new ros::NodeHandle();
    TaskManager* task_manager = new TaskManager(nh); 

    std::string dir(ros::package::getPath("bwi_logging") + "/log_files/"); 
    
    int cnt = 0; 
    srand(time(NULL)); 
    while (cnt < NUM_OF_TASKS) {

        cnt++; 
        ros::Duration(0.1).sleep();       
        int r = rand() % NUM_OF_TASK_TYPES; 

        // to initialize a task and label its status as Todo
        if (r == 0) {
            Color color = static_cast<Color>(rand() % COLOR_LENGTH);
            task_manager->addTask(new TaskWithStatus(new ScavTaskColorShirt(nh, dir, color), TODO)); 
        } 
        else if (r == 1) {
            task_manager->addTask(new TaskWithStatus(new ScavTaskWhiteBoard(nh, dir), TODO)); 
        } 
        else if (r == 2) {
            task_manager->addTask(new TaskWithStatus(new ScavTaskFetchObject(nh, dir), TODO)); 
        } 
        else if (r == 3) {
            task_manager->addTask(new TaskWithStatus(new ScavTaskHumanFollowing(nh, dir), TODO)); 
        } 
        else {
            cnt--; 
        }
    }

    task_manager->updateStatusGui(); 

    while (task_manager->allFinished() == false) {
        TaskWithStatus *task_status; 
        task_status = task_manager->selectNextTask();
        task_manager->updateStatusGui(); 
        task_manager->executeNextTask(TIMEOUT, task_status); 
        task_manager->updateStatusGui(); 
    }

    return 0; 
}

