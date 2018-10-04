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
#include <sound_play/SoundRequest.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace scav_task_human_following {

class ScavTaskHumanFollowing : public ScavTask {
public:

    ScavTaskHumanFollowing() : ac("") {}
    ScavTaskHumanFollowing(ros::NodeHandle *node_handle, std::string path_of_dir);

    void executeTask(int timeout, TaskResult &result, std::string &record);
    void visionThread();
    void motionThread();
    void stopEarly(); 

    void callback_human_detected(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void callback_image(const sensor_msgs::ImageConstPtr& msg);
    void callback_ac_followed(const actionlib::SimpleClientGoalState& state,
                              const move_base_msgs::MoveBaseResultConstPtr& result);
    void callback_ac_reached(const actionlib::SimpleClientGoalState& state,
                             const move_base_msgs::MoveBaseResultConstPtr& result);
    void callback_ac_done(const actionlib::SimpleClientGoalState& state,
                          const move_base_msgs::MoveBaseResultConstPtr& result);

    void amclPoseCallback(
        const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);


    void moveToPose(const geometry_msgs::Pose);

    SearchPlannerSimple *search_planner_simple;
    std::string directory;

    bool task_completed;

private:
    static constexpr double human_following_pose_offset = 1;
    static constexpr double human_pose_delta_threshold = 1.0;

    geometry_msgs::PoseStamped human_pose;
    geometry_msgs::PoseStamped human_following_pose;
    geometry_msgs::Pose current_pose;
    int human_detected;
    ros::Time detected_time;
    ros::Publisher cmd_vel_pub;
    ros::Publisher sound_pub; 
    MoveBaseClient ac;

};
}
#endif
