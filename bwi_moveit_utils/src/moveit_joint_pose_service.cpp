#include <signal.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <boost/assign/std/vector.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "bwi_moveit_utils/MoveitJointPose.h"
using namespace boost::assign;
    
#define NUM_JOINTS 6

bool g_caught_sigint = false;


moveit::planning_interface::MoveGroupInterface *group;


void sig_handler(int sig){
    g_caught_sigint = true;
    ROS_INFO("caugt sigint, init shutdown seq...");
    ros::shutdown();
    exit(1);
};  
bool service_cb(bwi_moveit_utils::MoveitJointPose::Request &req, bwi_moveit_utils::MoveitJointPose::Response &res){

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.addCollisionObjects(req.collision_objects);

    std::vector<double> q_vals;

    group->setJointValueTarget(req.target);
    group->setStartState(*group->getCurrentState());

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success = group->plan(my_plan);
    ROS_INFO("Planning success: %s", success ? "true" : "false");

    //call service

    if (!success) {
        res.completed = false;
        return true;
    }
    moveit::planning_interface::MoveItErrorCode error = group->move();
    res.completed = error == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    return true;
}   
int main(int argc, char **argv)
{   
    ros::init(argc, argv, "moveit_joint_pose_service");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    group = new moveit::planning_interface::MoveGroupInterface("arm");
    group->setGoalTolerance(0.01);
    group->setPoseReferenceFrame("m1n6s200_end_effector");
    ros::ServiceServer srv = nh.advertiseService("joint_pose_service", service_cb);

    //TODO: as a rosparam, option for planning time
    ros::spin();
    return 0;
}
 


