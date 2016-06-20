
#include <ros/ros.h>
#include <ros/package.h>

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <boost/thread.hpp>

#include "ScavTask.h"
#include "TaskManager.h"
#include "SearchPlanner.h"

#include "ScavTaskColorShirt.h"
#include "ScavTaskWhiteBoard.h"
#include "ScavTaskFetchObject.h"
#include "ScavTaskHumanFollowing.h"
#include "bwi_msgs/ScavHunt.h"

#define TIMEOUT (600) // return failure if not finished in 10 minutes
#define NUM_OF_TASKS (5)
#define NUM_OF_TASK_TYPES (4)

using namespace scav_task_human_following; 
TaskManager* task_manager; 
TaskWithStatus *curr_task; 

void publishThread() {
    while (ros::ok() && false == task_manager->allFinished()) {
        ros::Duration(5).sleep(); 
        ROS_INFO_STREAM("publishing scavenger hunt status"); 
        task_manager->publishStatus(); 
    }
}

bool callback_srv_scav(bwi_msgs::ScavHunt::Request &req, 
                       bwi_msgs::ScavHunt::Response &res) {
 
    if ((int) req.type == bwi_msgs::ScavHuntRequest::SCAV_PAUSE) {
        curr_task->status = TODO; 
        curr_task->task->stopEarly(); 
        task_manager->paused = true; 
    } else if ((int) req.type == bwi_msgs::ScavHuntRequest::SCAV_RESUME) {
        task_manager->paused  = false; 
    }
    return true; 
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "scavenger");
    ros::NodeHandle *nh = new ros::NodeHandle();
    task_manager = new TaskManager(nh); 

    std::string dir(ros::package::getPath("bwi_logging") + "/log_files/"); 

    ros::ServiceServer scav_srv = nh->advertiseService("/scav_control", callback_srv_scav);
    ROS_INFO("Ready to provide scav task pause/resume service"); 
    ros:spin(); 
    
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
        } else if (r == 1) {
            task_manager->addTask(new TaskWithStatus(new ScavTaskWhiteBoard(nh, dir), TODO)); 
        } else if (r == 2) {
            task_manager->addTask(new TaskWithStatus(new ScavTaskFetchObject(nh, dir), TODO)); 
        } else if (r == 3) {
            task_manager->addTask(new TaskWithStatus(new ScavTaskHumanFollowing(nh, dir), TODO)); 
        } else {
            cnt--; 
        }
    }

    task_manager->updateStatusGui(); 
    boost::thread p_thread( &publishThread); 

    while (ros::ok() && task_manager->allFinished() == false) {

        if (task_manager->paused) {
            ros::Duration(2).sleep(); 
            continue; 
        }

        ROS_INFO_STREAM("selecting next task"); 
        curr_task = task_manager->selectNextTask();

        ROS_INFO_STREAM("updating GUI"); 
        task_manager->updateStatusGui(); 

        ROS_INFO_STREAM("executing next task"); 
        task_manager->executeNextTask(TIMEOUT, curr_task); 
    }

    p_thread.detach(); 

    return 0; 
}


