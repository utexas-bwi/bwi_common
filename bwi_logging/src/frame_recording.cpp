
#include <ros/ros.h>
#include <ros/package.h> // for getting path to current package
#include <message_filters/subscriber.h> // for synchronizing messages
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h> // for saving images
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h> // for amcl pose
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <sstream>
#include <fstream> // for writing (x, y, ori) text files
#include <math.h> // for atan2(y, x)

#define PI 3.14159265

ros::NodeHandle *nh_pt; 

ros::Time start, amcl_time, interp_time; 
int cnt; 
double frame_rate; 
std::string path; 

// supposed to share the same stamp
geometry_msgs::Pose amcl_pose; 
 
geometry_msgs::Pose odom_pose_at_amcl_freq, odom_pose_at_odom_freq; 

struct InterpolationPose {
    double x;  
    double y;
    double ori; 
} interp_pose; 

// callback for updating amcl time and amcl pose
void amclCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    amcl_time = msg->header.stamp; 
    amcl_pose = msg->pose.pose; 
    odom_pose_at_amcl_freq = odom_pose_at_odom_freq; 
}

// callback for odom
void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg)
{
    ros::Time odom_time = odom_msg->header.stamp; 
    odom_pose_at_odom_freq = odom_msg->pose.pose;

    ros::Duration diff = odom_time - amcl_time; 
    interp_time = odom_time; 

    double x, y, amcl_ori, ori_diff, angle_in_radians; 

    x = amcl_pose.position.x + odom_pose_at_odom_freq.position.x - 
        odom_pose_at_amcl_freq.position.x; 
    y = amcl_pose.position.y + odom_pose_at_odom_freq.position.y - 
        odom_pose_at_amcl_freq.position.y; 
    amcl_ori = 2.0 * atan2(amcl_pose.orientation.z, amcl_pose.orientation.w); 

    ori_diff = atan2(odom_pose_at_odom_freq.orientation.z, 
                     odom_pose_at_odom_freq.orientation.w) - 
               atan2(odom_pose_at_amcl_freq.orientation.z, 
                     odom_pose_at_amcl_freq.orientation.w); 
    ori_diff *= 2.0; 
    angle_in_radians = amcl_ori + ori_diff + (10.0 * PI); 

    if (angle_in_radians < 0) 
        ROS_ERROR("angle_in_radians is negative"); 

    angle_in_radians = fmod( angle_in_radians + PI, PI*2.0 ) - PI; 

    interp_pose.x = x; 
    interp_pose.y = y;
    interp_pose.ori = angle_in_radians; 
}

// callback for saving log files at a given frame rate
void syncCallback(const sensor_msgs::ImageConstPtr& rgb_img, 
    const sensor_msgs::ImageConstPtr& dep_img)
{
    std::ostringstream os; 
    ros::Duration diff = ros::Time::now() - start; 

    if (diff.toSec() < cnt * 1.0 / frame_rate) 
        return;
    
    cv_bridge::CvImageConstPtr rgb_pt, dep_pt;

    rgb_pt = cv_bridge::toCvShare(rgb_img, sensor_msgs::image_encodings::BGR8);
    dep_pt = cv_bridge::toCvShare(dep_img, sensor_msgs::image_encodings::TYPE_16UC1); 

    std::string rgb_file, dep_file, pos_file, rgb_sec, dep_sec; 

    os.str(""); 
    os << cnt; 
    rgb_file = "rgb" + os.str() + ".jpg"; 
    dep_file = "depth" + os.str() + ".jpg"; 
    pos_file = "position" + os.str() + ".txt"; 

    os.str(""); 
    os << rgb_pt->header.stamp.sec; 
    rgb_sec = os.str();

    os.str(""); 
    os << dep_pt->header.stamp.sec; 
    dep_sec = os.str(); 

    os.str("");
    os << interp_pose.x << " " << interp_pose.y << " " << interp_pose.ori; 

    cv::Mat rgb_frame = rgb_pt->image; 
    cv::Mat dep_frame = dep_pt->image; 
    cv::Mat float_dep_frame; 

    float_dep_frame = cv::Mat(dep_frame.size(), CV_8UC1);
    cv::convertScaleAbs(dep_frame, float_dep_frame, 4.5/255.0, 0.0);

    try {
        cv::imwrite(path + "/log_files/rgb/" + rgb_file, rgb_frame); 
        cv::imwrite(path + "/log_files/depth/" + dep_file, float_dep_frame); 
        std::ofstream output( (path + "/log_files/position/" + pos_file).c_str() ); 
        output << os.str(); 
        output.close(); 
    } 
    catch (std::runtime_error & ex) {
        ROS_ERROR("Not being able to save log files: %s", ex.what()); 
        return; 
    }
    
    // ROS_INFO_STREAM(rgb_file << " saved with stamp.sec: " << rgb_sec); 
    // ROS_INFO_STREAM(dep_file << " saved with stamp.sec: " << dep_sec); 
    // ROS_INFO_STREAM(pos_file << " saved with stamp.sec: " << interp_time.sec); 

    ROS_INFO("%d saved", cnt++); 
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "frame_recording_node");
    nh_pt = new ros::NodeHandle(); 

    cnt = 0; 
    ros::param::param <double> ("~frame_rate", frame_rate, 10.0);
    start = ros::Time::now(); 

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(*nh_pt,
        "/nav_kinect/rgb/image_color", 1); 
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(*nh_pt, 
        "/nav_kinect/depth_registered/image_raw", 1);

    // get path used for saving log files
    path = ros::package::getPath("bwi_logging");
    
    typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy; 

    // synchronize frames from rgb and depth topics
    // TODO: I am not sure 10 is good for this queue_size value
    message_filters::Synchronizer <MySyncPolicy> sync(MySyncPolicy(10),
        rgb_sub, depth_sub); 
    sync.registerCallback( boost::bind( &syncCallback, _1, _2 ) ); 

    ros::Subscriber amcl_sub = nh_pt->subscribe("amcl_pose", 10, amclCallback); 
    ros::Subscriber odom_sub = nh_pt->subscribe("odom", 1000, odomCallback); 

    ros::AsyncSpinner spinner(3); // for the above two callback functions
    spinner.start();
    ros::waitForShutdown(); 

    return 0; 
}


