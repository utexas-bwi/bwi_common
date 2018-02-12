#include <signal.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/conversions.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "bwi_moveit_utils/MoveitWaypoint.h"

bool g_caught_sigint = false;


ros::Publisher pose_pub;
moveit::planning_interface::MoveGroupInterface *group;

void sig_handler(int sig){
    g_caught_sigint = true;
    ROS_INFO("caugt sigint, init shutdown seq...");
    ros::shutdown();
    exit(1);
}

bool service_cb(bwi_moveit_utils::MoveitWaypoint::Request &req, bwi_moveit_utils::MoveitWaypoint::Response &res){
    ROS_INFO("[mico_moveit_cartesianpose_service.cpp] Request received!");

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.addCollisionObjects(req.collision_objects);
    group->setStartState(*group->getCurrentState());
    group->setPathConstraints(req.constraints);
    ROS_INFO("[mico_moveit_waypoint_service.cpp] starting to plan...");

    moveit_msgs::RobotTrajectory result;
    std::vector<geometry_msgs::PoseStamped> waypoints;
    double fraction = group->computeCartesianPath(req.waypoints, 0.01, 0,result);
    // We'll only execute if the full trajectory was generated
    if (fraction < 1) {
        return false;
    }

    robot_trajectory::RobotTrajectory rt(group->getCurrentState()->getRobotModel(), "arm");
    rt.setRobotTrajectoryMsg(*group->getCurrentState(), result);

    // Thrid create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;

    // Fourth compute computeTimeStamps
    bool success = iptp.computeTimeStamps(rt);
    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(result);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    moveit_msgs::RobotState start_state;
    moveit::core::robotStateToRobotStateMsg(*group->getCurrentState(), start_state);
    my_plan.start_state_= start_state;
    my_plan.trajectory_= result;
    moveit::planning_interface::MoveItErrorCode error = group->execute(my_plan);
    return error == moveit::planning_interface::MoveItErrorCode::SUCCESS;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "moveit_waypoint_service");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    group = new moveit::planning_interface::MoveGroupInterface("arm");
    group->setGoalTolerance(0.01);
    group->setPoseReferenceFrame("m1n6s200_end_effector");
    ros::ServiceServer srv = nh.advertiseService("waypoint_service", service_cb);

    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mico_waypoint_service/target_pose", 1, true);

    ros::spin();
    return 0;
}