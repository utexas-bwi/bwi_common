#include <signal.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include "bwi_moveit_utils/MoveitCartesianPose.h"

bool g_caught_sigint = false;


ros::Publisher pose_pub;
moveit::planning_interface::MoveGroupInterface *group;

void sig_handler(int sig){
    g_caught_sigint = true;
    ROS_INFO("caugt sigint, init shutdown seq...");
    ros::shutdown();
    exit(1);
}

bool service_cb(bwi_moveit_utils::MoveitCartesianPose::Request &req, bwi_moveit_utils::MoveitCartesianPose::Response &res){
    ROS_INFO("[mico_moveit_cartesianpose_service.cpp] Request received!");

    //publish target pose for visualization
    pose_pub.publish(req.target);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.addCollisionObjects(req.collision_objects);
    group->setPoseTarget(req.target);
    group->setStartState(*group->getCurrentState());
    group->setPathConstraints(req.constraints);

    ROS_INFO("[mico_moveit_cartesianpose_service.cpp] starting to plan...");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success = group->plan(my_plan);
    ROS_INFO("Planning success: %s", success ? "true" : "false");

    if (!success) {
        res.completed = false;
        return true;
    }
    moveit::planning_interface::MoveItErrorCode error = group->move();
    res.completed = error == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "moveit_cartesian_pose_service");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    group = new moveit::planning_interface::MoveGroupInterface("arm");
    group->setGoalTolerance(0.01);
    group->setPoseReferenceFrame("m1n6s200_end_effector");
    ros::ServiceServer srv = nh.advertiseService("cartesian_pose_service", service_cb);

    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mico_cartesian_pose_service/target_pose", 1, true);

    ros::spin();
    return 0;
}


