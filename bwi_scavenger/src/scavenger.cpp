
#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include "plan_execution/ExecutePlanAction.h"
#include <actionlib/client/simple_action_client.h>

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
ros::NodeHandle *nh;

void publishThread() {
    ros::Rate r(5); 
    while (ros::ok() and r.sleep() and false == task_manager->allFinished()) {
        // ROS_INFO_STREAM("publishing scavenger hunt status"); 
        task_manager->publishStatus(); 
    }
}

bool callback_srv_scav(bwi_msgs::ScavHunt::Request &req, 
                       bwi_msgs::ScavHunt::Response &res) {
 
    ROS_INFO("callback_srv_scav called"); 
    ROS_INFO_STREAM("current task status: 0 ongoing, 1 finished, 2 todo): " 
        << curr_task->status);

    if ((int) req.type == bwi_msgs::ScavHuntRequest::SCAV_PAUSE) {
        curr_task->status = TODO; 
        ROS_INFO("calling curr_task->task->stopEarly()");
        curr_task->task->stopEarly(); 
        task_manager->paused = true; 

        ROS_INFO("stopping movements at kr and motion levels"); 
        actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> 
            client("/action_executor/execute_plan", true);

        ros::spinOnce(); 
        client.cancelAllGoals (); 
        ros::spinOnce(); 

        ros::Publisher pub = nh->advertise<actionlib_msgs::GoalID>
            ("/move_base/cancel", 10);

        actionlib_msgs::GoalID msg; 
        msg.id = ""; 
        // msg.stamp = ros::Time::now();
        
        ros::spinOnce(); 
        pub.publish(msg);
        ros::spinOnce(); 
        ROS_INFO("the robot is supposed to stop moving now"); 

    } else if ((int) req.type == bwi_msgs::ScavHuntRequest::SCAV_RESUME) {
        task_manager->paused  = false; 
    }
    return true; 
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "scavenger");
    nh = new ros::NodeHandle();
    task_manager = new TaskManager(nh); 

    std::string dir(ros::package::getPath("bwi_logging") + "/log_files/"); 

    ros::ServiceServer scav_srv = nh->advertiseService("/scav_control", callback_srv_scav);
    ROS_INFO("Ready to provide scav task pause/resume service"); 
    ros::AsyncSpinner spinner(2); 
    spinner.start(); 
    // ros::spin(); 
    
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

    ros::Rate r(2); 
    while (ros::ok() and r.sleep() and task_manager->allFinished() == false) {
 
        ROS_INFO_STREAM("paused? " << task_manager->paused);
        if (task_manager->paused) {
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


