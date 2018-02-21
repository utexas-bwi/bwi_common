#ifndef SEARCHPLANNER_H
#define SEARCHPLANNER_H

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "plan_execution/ExecutePlanAction.h"

#define PI (3.1415926)
typedef actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> KrClient;

class SearchPlannerSimple {

public:

    SearchPlannerSimple() {}; 
    SearchPlannerSimple(ros::NodeHandle *nh); 

    bool moveToNextDoor(); 

    bool cancelCurrentGoal(); 

    bool busy; 

protected: 

    ros::NodeHandle *nh;
    KrClient *client; 

private: 

    std::vector<std::string> doors; 
    int goal_door; 

}; 

class SearchPlanner : public SearchPlannerSimple {
public: 

    SearchPlanner() {}; 
    SearchPlanner(ros::NodeHandle *nh, std::string path_to_yaml, float goal_tolerance); 

    std::vector<geometry_msgs::PoseStamped> positions; 
    std::vector<float> belief;

    ros::ServiceClient client_make_plan;
    ros::Publisher pub_simple_goal; 
    ros::Subscriber sub_amcl_pose; 

    // ros::NodeHandle *nh;

    geometry_msgs::PoseStamped next_goal; 

    bool targetDetected; 
    float tolerance; 

    void setTargetDetection(bool detected) {
        targetDetected = detected; 
    }
    bool getTargetDetection() { return targetDetected; }

    geometry_msgs::PoseWithCovarianceStamped getCurrPosition(); 

    geometry_msgs::PoseStamped selectNextScene(const std::vector<float> &b, int &next_goal_index); 
    void moveToNextScene(const geometry_msgs::PoseStamped &next_goal); 
    void analyzeScene(float angle, float angular_vel); 
    void updateBelief(int ); 

}; 

#endif
