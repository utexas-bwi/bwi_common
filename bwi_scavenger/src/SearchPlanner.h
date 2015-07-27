#ifndef SEARCHPLANNER_H
#define SEARCHPLANNER_H

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#define PI (3.1415926)

class SearchPlanner {
public: 

    SearchPlanner() {}
    SearchPlanner(ros::NodeHandle *nh, std::string path_to_yaml, float goal_tolerance); 

    std::vector<geometry_msgs::PoseStamped> positions; 
    std::vector<float> belief;

    ros::ServiceClient client_make_plan;
    ros::Publisher pub_simple_goal; 
    ros::Subscriber sub_amcl_pose; 

    ros::NodeHandle *nh;

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
