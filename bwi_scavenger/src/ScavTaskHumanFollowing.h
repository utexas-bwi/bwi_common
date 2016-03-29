#ifndef SCAVTASKHUMANFOLLOWING_H
#define SCAVTASKHUMANFOLLOWING_H

#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include "ScavTask.h"
#include "SearchPlanner.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace scav_task_human_following {

class ScavTaskHumanFollowing : public ScavTask {
public:

    ScavTaskHumanFollowing() : ac("") {}
    ScavTaskHumanFollowing(ros::NodeHandle *node_handle, std::string path_of_dir);

    void executeTask(int timeout, TaskResult &result, std::string &record);
    void visionThread();
    void motionThread();

    void callback_human_detected(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void callback_image(const sensor_msgs::ImageConstPtr& msg);
    void callback_ac_followed_done(const actionlib::SimpleClientGoalState& state,
                                  const move_base_msgs::MoveBaseResultConstPtr& result);
    void callback_ac_reached_done(const actionlib::SimpleClientGoalState& state,
                                  const move_base_msgs::MoveBaseResultConstPtr& result);


    void amclPoseCallback(
        const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);


    void moveToPose(const geometry_msgs::Pose);

    SearchPlanner *search_planner;
    std::string directory;

    MoveBaseClient ac;

    // ros::Publisher pub_simple_goal;
    // ros::Subscriber sub_goal_result;

    bool task_completed;
};
}
#endif
