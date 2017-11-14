#include <stdio.h>
#include <math.h>

#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/Twist.h>
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
sensor_msgs::ImageConstPtr wb_image_candidate;
std::string wb_path_to_image;


ScavTaskHumanFollowing::ScavTaskHumanFollowing(ros::NodeHandle *nh, std::string dir) : ac("move_base", true)
{
    this->nh = nh;
    directory = dir;
    task_description = "following a human for a distance";
    task_name = "Human following";
    certificate = ""; 

    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);

    sound_pub = nh->advertise<sound_play::SoundRequest>("robotsound", 1); 

    search_planner_simple = new SearchPlannerSimple(nh); 

    task_completed = false;
    human_detected = -1;
}


void ScavTaskHumanFollowing::callback_human_detected(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (human_detected == -1)
        search_planner_simple->cancelCurrentGoal();

    detected_time = ros::Time::now();

    ROS_INFO("Person detected");


    // Check if the new human pose is far enough from the previous one the justify sending a new waypoint
    double human_pose_delta = sqrt(pow(msg->pose.position.x - human_pose.pose.position.x, 2) +
                                   pow(msg->pose.position.y - human_pose.pose.position.y, 2));

    if (human_pose_delta > human_pose_delta_threshold) {

        // Filename formatting
        boost::posix_time::ptime curr_time = boost::posix_time::second_clock::local_time();
        std::string time_str = boost::posix_time::to_iso_extended_string(curr_time);

        wb_path_to_image = directory + "human_following_" + time_str + ".PNG";
        wb_image = wb_image_candidate;

        // // Noise protection
        // human_pose_candidate_count += 1;
        // human_pose_candidate.pose.position = msg->pose.position;

        human_pose.pose.position = msg->pose.position;
        human_pose.header.frame_id = "level_mux_map";
        human_pose.header.stamp = msg->header.stamp;
        human_pose.pose.orientation.x = 0;
        human_pose.pose.orientation.y = 0;
        human_pose.pose.orientation.z = 0;
        human_pose.pose.orientation.w = 1;
        // ROS_INFO("Human_pose before (%.2f, %.2f)", human_pose.pose.position.x, human_pose.pose.position.y);

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
        human_pose.pose.orientation.x = 0;
        human_pose.pose.orientation.y = 0;
        human_pose.pose.orientation.z = sin((theta)/2);
        human_pose.pose.orientation.w = cos((theta)/2);
        // ROS_INFO("human_following_pose after  (%.2f, %.2f)", human_following_pose.pose.position.x, human_following_pose.pose.position.y);

        human_detected = 1;
    }
}


void ScavTaskHumanFollowing::callback_image(const sensor_msgs::ImageConstPtr& msg)
{
    wb_image_candidate = msg;
}


void ScavTaskHumanFollowing::callback_ac_followed(const actionlib::SimpleClientGoalState& state,
                                                       const move_base_msgs::MoveBaseResultConstPtr& result)
{
    ROS_INFO("Reached human following way point");
    // Only proceed to move to human's location if the person has not been seen for a fixed duraction
    ros::Duration last_detected_duration_threshold (0.2);
    ros::Duration detection_time_elapsed = ros::Time::now() - detected_time;
    if (detection_time_elapsed < last_detected_duration_threshold){
        ROS_INFO("Human still in view, not moving in; detection_elapsed: %f", detection_time_elapsed.toSec());
    } else {
        ROS_INFO("Move to human's last seen location");
        human_detected = 2;
    }
}

void ScavTaskHumanFollowing::callback_ac_reached(const actionlib::SimpleClientGoalState& state,
                                                      const move_base_msgs::MoveBaseResultConstPtr& result)
{
    ROS_INFO("Reached human last seen location");
    human_detected = 4;
}

void ScavTaskHumanFollowing::callback_ac_done(const actionlib::SimpleClientGoalState& state,
                                                 const move_base_msgs::MoveBaseResultConstPtr& result)
{
    human_detected = 4;
}

void ScavTaskHumanFollowing::amclPoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    current_pose = msg->pose.pose;
}

void ScavTaskHumanFollowing::motionThread() {

    ros::Rate r(2);

    // Keep track of the robot's current pose
    ros::Subscriber amcl_pose_sub;
    amcl_pose_sub = nh->subscribe("amcl_pose", 100, &ScavTaskHumanFollowing::amclPoseCallback, this);

    move_base_msgs::MoveBaseGoal goal;
    sound_play::SoundRequest sound_msg; 

    // Loop to send action requests
    while (ros::ok() and r.sleep() and task_completed == false) {
        ros::spinOnce();
        ROS_INFO_STREAM("human_detected: " << human_detected); 

        // random visit door until a human's detected
        if (human_detected == -1) {
            if (false == search_planner_simple->busy) 
                search_planner_simple->moveToNextDoor();   
        }



        // Set new waypoint if new human is detected
        if (human_detected == 1) {
 
            human_detected = 0;
            goal.target_pose = human_following_pose;
            ac.sendGoal(goal, boost::bind(&ScavTaskHumanFollowing::callback_ac_followed, this,  _1, _2));

            sound_msg.command = sound_play::SoundRequest::PLAY_ONCE; 
            sound_msg.arg = "following"; 
            sound_msg.arg2 = "voice_kal_diphone"; 

            sound_pub.publish(sound_msg); 
        }
        // callback_ac_followed should set the condition to go to human's last seen location
        if (human_detected == 2) {
            human_detected = 0;
            goal.target_pose = human_pose;
            ac.sendGoal(goal, boost::bind(&ScavTaskHumanFollowing::callback_ac_reached, this,  _1, _2));
        }
        // if (human_detected == 3) {
        //     // Spin
        //     human_detected = 0;
        //     geometry_msgs::Twist rotate;
        //     rotate.angular.z = -0.4;

        //     ros::Time start_time = ros::Time::now();
        //     ros::Duration spin_timeout(4.0); // Timeout of 2 seconds
        //     while(ros::Time::now() - start_time < spin_timeout) {
        //         cmd_vel_pub.publish(rotate);
        //         ros::spinOnce();
        //     }
        //     goal.target_pose = human_pose;
        //     ac.sendGoal(goal, boost::bind(&ScavTaskHumanFollowing::callback_ac_reached, this,  _1, _2));
        // }
        if (human_detected == 4) {
            human_detected = 0;
            ROS_INFO("Human lost, following task complete");

            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(wb_image, sensor_msgs::image_encodings::BGR8);

            if (boost::filesystem::is_directory(directory) == false) {
                boost::filesystem::path tmp_path(directory);
                boost::filesystem::create_directory(tmp_path);
            }
            ROS_INFO("Saving image at %s", wb_path_to_image.c_str());
            cv::imwrite(wb_path_to_image, cv_ptr->image);

            // Scp the log file to remote machine

            // std::string scp_remote_path = "wxie@hypnotoad.csres.utexas.edu:~/bwi_scavenger_log_files/";
            // std::string system_string = "scp -P 40 " + wb_path_to_image + " " + scp_remote_path;
            // ROS_INFO("%s", system_string.c_str());
            // if (system(system_string.c_str()) == -1) {
            //     ROS_WARN("Problem transferring log image file to remote machine. Please ssh permission and remote machine");
            // }

            // search_planner->setTargetDetection(true); // change status to terminate the motion thread

            ROS_INFO_STREAM("Finished saving image: " << wb_path_to_image); 

            task_completed = true;
        }
    }
}

void ScavTaskHumanFollowing::visionThread() {

    ros::Subscriber sub_human = nh->subscribe("/segbot_pcl_person_detector/human_poses",
        100, &ScavTaskHumanFollowing::callback_human_detected, this);

    image_transport::ImageTransport it(*nh);
    image_transport::Subscriber sub_image = it.subscribe("/nav_kinect/rgb/image_raw",
        1, &ScavTaskHumanFollowing::callback_image, this);

    ros::Rate r(10);
    while (ros::ok() and r.sleep() and !task_completed) {
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


void ScavTaskHumanFollowing::stopEarly() {
    task_completed = true; 
}
}
