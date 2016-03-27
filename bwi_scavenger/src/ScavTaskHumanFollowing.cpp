#include <math.h>

#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "ScavTaskHumanFollowing.h"



namespace scav_task_human_following {

sensor_msgs::ImageConstPtr wb_image;
std::string wb_path_to_image;
geometry_msgs::PoseStamped human_pose;
geometry_msgs::PoseStamped human_following_pose;
geometry_msgs::Pose current_pose;

double human_following_pose_offset = 1;
bool human_detected;

ros::Time detected_time;



ScavTaskHumanFollowing::ScavTaskHumanFollowing(ros::NodeHandle *nh, std::string dir) {
    this->nh = nh;
    directory = dir;
    task_description = "following a human for a distance";
    task_name = "Human following";

    pub_simple_goal = nh->advertise<geometry_msgs::PoseStamped>(
        "/move_base_interruptable_simple/goal", 100);

    // task_completed = false;
    human_detected = false;
}

void ScavTaskHumanFollowing::callback_human_detected(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    detected_time = ros::Time::now();

    ROS_INFO("People detected");

    // Filename formatting
    boost::posix_time::ptime curr_time = boost::posix_time::second_clock::local_time();
    wb_path_to_image = directory + "human_following_" + boost::posix_time::to_simple_string(curr_time) + ".PNG";

    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(wb_image, sensor_msgs::image_encodings::BGR8);

    if (false == boost::filesystem::is_directory(directory)) {
        boost::filesystem::path tmp_path(directory);
        boost::filesystem::create_directory(tmp_path);
    }
    cv::imwrite(wb_path_to_image, cv_ptr->image);

    // search_planner->setTargetDetection(true); // change status to terminate the motion thread

    human_pose.pose.position = msg->pose.position;
    human_pose.header.frame_id = "level_mux/map";
    human_pose.pose.orientation.x = 0;
    human_pose.pose.orientation.y = 0;
    human_pose.pose.orientation.z = 0;
    human_pose.pose.orientation.w = 1;
    ROS_INFO("Human_pose before (%.2f, %.2f)", human_pose.pose.position.x, human_pose.pose.position.y);

    // Modify the location of the human so that it's slightly off in the direction
    // of the robot
    human_following_pose = human_pose;
    double theta = atan2(human_pose.pose.position.y - current_pose.position.y,
                         human_pose.pose.position.x - current_pose.position.x);
    // Convert angle to the right range by adding PI? Doesn't feel like this is needed for ROS
    human_following_pose.pose.position.x -= human_following_pose_offset * cos(theta);
    human_following_pose.pose.position.y -= human_following_pose_offset * sin(theta);
    human_following_pose.pose.orientation.x = 0;
    human_following_pose.pose.orientation.y = 0;
    human_following_pose.pose.orientation.z = sin((theta)/2);
    human_following_pose.pose.orientation.w = cos((theta)/2);
    ROS_INFO("human_following_pose after  (%.2f, %.2f)", human_following_pose.pose.position.x, human_following_pose.pose.position.y);

    human_detected = true;
}

void ScavTaskHumanFollowing::callback_image(const sensor_msgs::ImageConstPtr& msg) {
    wb_image = msg;
}


void ScavTaskHumanFollowing::amclPoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    current_pose = msg->pose.pose;
}


void ScavTaskHumanFollowing::motionThread() {
    // Keep track of the robot's current pose
    ros::Subscriber amcl_pose_sub;
    amcl_pose_sub = nh->subscribe("amcl_pose", 100, &ScavTaskHumanFollowing::amclPoseCallback, this);

    ros::Rate r(10);
    while (ros::ok() and r.sleep() and task_completed == false) {

        ros::spinOnce();
        if (human_detected) {

            pub_simple_goal.publish(human_following_pose);
            ros::spinOnce();
            pub_simple_goal.publish(human_following_pose);
            ros::spinOnce();
            pub_simple_goal.publish(human_following_pose);
            ros::spinOnce();

            human_detected = false;
            // task_completed = true;
        }
    }
}

void ScavTaskHumanFollowing::visionThread() {

    ros::Subscriber sub_human = nh->subscribe("/segbot_pcl_person_detector/human_poses",
        100, &ScavTaskHumanFollowing::callback_human_detected, this);

    image_transport::ImageTransport it(*nh);
    image_transport::Subscriber sub_image = it.subscribe("/nav_kinect/rgb/image_color",
        1, &ScavTaskHumanFollowing::callback_image, this);

    ros::Rate r(10);
    while (ros::ok() and r.sleep()) {
        ros::spinOnce();
    }
}

void ScavTaskHumanFollowing::executeTask(int timeout, TaskResult &result, std::string &record) {

    boost::thread motion( &ScavTaskHumanFollowing::motionThread, this);
    boost::thread vision( &ScavTaskHumanFollowing::visionThread, this);

    motion.join();
    vision.join();
    record = wb_path_to_image;
    result = SUCCEEDED;
}
}
