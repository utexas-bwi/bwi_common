
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "ScavTaskWhiteBoard.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

sensor_msgs::ImageConstPtr wb_image; 
std::string wb_path_to_image; 

ScavTaskWhiteBoard::ScavTaskWhiteBoard(ros::NodeHandle *nh, std::string dir) {
    this->nh = nh; 
    directory = dir;
    task_description = "find a person standing near a whiteboard"; 
    task_name = "White board"; 
}


/*---------------------------------------------------------------
        a           b           1, board near conference room
           m                    2, board near 400/500 doors
        c           d       
    M of coordinates(x,y) is inside the rectangle iff, 
    (0 < AM dot AB < AB dot AB) AND (0 < AM dot AC < AC dot AC) 
-----------------------------------------------------------------*/

bool ScavTaskWhiteBoard::inRectangle(Pose m, Pose a, Pose b, Pose c) 
{
    float am_ab = (m.x - a.x) * (b.x - a.x) + (m.y - a.y) * (b.y - a.y);
    float ab_ab = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
    float am_ac = (m.x - a.x) * (c.x - a.x) + (m.y - a.y) * (c.y - a.y);
    float ac_ac = (c.x - a.x) * (c.x - a.x) + (c.y - a.y) * (c.y - a.y);

    return (0 < am_ab and am_ab < ab_ab) and (0 < am_ac and am_ac < ac_ac); 
}

void ScavTaskWhiteBoard::callback_human_detected(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Pose a1 = Pose(-30.88, 0.06);
    Pose b1 = Pose(-29.48, 0.06);
    Pose c1 = Pose(-30.89, -3.16);

    Pose a2 = Pose(-8.25, -5.99);
    Pose b2 = Pose(-6.83, -6.05);
    Pose c2 = Pose(-8.21, -11.26);

    Pose pose = Pose(msg->pose.position.x, msg->pose.position.y); 

    if (inRectangle(pose, a1, b1, c1) or inRectangle(pose, a2, b2, c2)) {

        ROS_INFO("People detected near a white board, picture saved.");

        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(wb_image, sensor_msgs::image_encodings::BGR8);

        boost::posix_time::ptime curr_time = boost::posix_time::second_clock::local_time(); 
        wb_path_to_image = directory + "white_board_" + boost::posix_time::to_simple_string(curr_time); 

        if (false == boost::filesystem::is_directory(directory)) {
            boost::filesystem::path tmp_path(directory);
            boost::filesystem::create_directory(tmp_path);
        } 
        cv::imwrite(wb_path_to_image, cv_ptr->image);
        search_planner->setTargetDetection(true); // change status to terminate the motion thread
    }
}

void ScavTaskWhiteBoard::callback_image(const sensor_msgs::ImageConstPtr& msg) {
    wb_image = msg;
}

void ScavTaskWhiteBoard::motionThread() {

    std::string path_to_yaml; 

    if (false == nh->hasParam("path_to_search_points"))
        ROS_ERROR("path to yaml file of search points not set"); 
    ros::param::get("path_to_search_points", path_to_yaml); 

    search_planner = new SearchPlanner(nh, path_to_yaml, tolerance);           

    int next_goal_index;                                                        
    while (ros::ok()) {
        search_planner->moveToNextScene( search_planner->selectNextScene(search_planner->belief, next_goal_index) );
        search_planner->analyzeScene(0.25*PI, PI/10.0);
        search_planner->updateBelief(next_goal_index);
    }
}

void ScavTaskWhiteBoard::visionThread() {

    ros::Subscriber sub_human = nh->subscribe("/segbot_pcl_person_detector/human_poses", 100, 
        &ScavTaskWhiteBoard::callback_human_detected, this); 

    image_transport::ImageTransport it(*nh);
    image_transport::Subscriber sub_image = it.subscribe("/nav_kinect/rgb/image_color", 1, 
        &ScavTaskWhiteBoard::callback_image, this);

    ros::Rate r(10); 
    while (ros::ok() and r.sleep()) {
        ros::spinOnce(); 
    }
}

void ScavTaskWhiteBoard::executeTask(int timeout, TaskResult &result, std::string &record) {

    boost::thread motion( &ScavTaskWhiteBoard::motionThread, this);
    boost::thread vision( &ScavTaskWhiteBoard::visionThread, this); 

    motion.join();
    vision.join(); 
    record = wb_path_to_image; 
    result = SUCCEEDED; 
}

