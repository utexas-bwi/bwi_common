#include "plan_execution/LogicalAction.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include <ros/ros.h>
using namespace plan_exec;
using namespace actasp;

void updateStates(const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {
    ROS_INFO_STREAM(msg->status_list[0]);
    //if ((msg->status_list[0].status > 1) and (msg->status_list[0].status < 6)  or (msg->status_list[0].status == 8)) { //2,3,4,5,8 are terminal states
    if (((*(msg->status_list.end())).status == 3)) {
        LogicalAction setState("noop");
        setState.run();
    }
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "state_tracker");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/execute_logical_goal/status", 1, updateStates);

    while (ros::ok()) {
        ros::Duration(5).sleep();
        ros::spinOnce();
    }
}