#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <boost/lexical_cast.hpp>

visualization_msgs::Marker robot_line_strip;
visualization_msgs::Marker human_line_strip;
visualization_msgs::Marker text_view_facing;


ros::Publisher robot_marker_pub;
ros::Publisher human_marker_pub;
ros::Publisher text_marker_pub;

ros::Time frame_time; 
bool task_done; 
double scav_distance = 0.0; 

void robot_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {

    geometry_msgs::Point p;
    p.x = msg->pose.pose.position.x; 
    p.y = msg->pose.pose.position.y; 
    p.z = msg->pose.pose.position.z; 

    if (false  == robot_line_strip.points.empty()) {
        double dis = pow((p.x-robot_line_strip.points.back().x), 2.0) + 
                     pow((p.y-robot_line_strip.points.back().y), 2.0); 
        scav_distance += pow(dis, 0.5); 
    }
    robot_line_strip.points.push_back(p); 

    text_view_facing.pose.position = p;
    text_view_facing.pose.position.y += 1.0;
    text_view_facing.text = boost::lexical_cast<std::string>(scav_distance);

    text_marker_pub.publish(text_view_facing); 
    robot_marker_pub.publish(robot_line_strip);
}

void human_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

    ros::Duration d = ros::Time::now() - frame_time; 
    frame_time = ros::Time::now(); 
    if (d.toSec() > 2) {
        human_line_strip.points.clear(); 
    }

    geometry_msgs::Point p;
    p.x = msg->pose.position.x; 
    p.y = msg->pose.position.y; 
    p.z = msg->pose.position.z; 
    human_line_strip.points.push_back(p); 

    human_marker_pub.publish(human_line_strip);
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "trajectory_drawing");
    ros::NodeHandle nh;
    frame_time = ros::Time::now(); 
    task_done = false; 

    robot_marker_pub = nh.advertise<visualization_msgs::Marker> ("scav_robot_marker", 10);
    human_marker_pub = nh.advertise<visualization_msgs::Marker> ("scav_human_marker", 10);
    text_marker_pub = nh.advertise<visualization_msgs::Marker> ("scav_text_marker" ,10); 

    ros::Subscriber robot_pos_sub = nh.subscribe("/amcl_pose", 100, robot_callback); 
    ros::Subscriber human_pos_sub = nh.subscribe("/segbot_pcl_person_detector/human_poses", 100, human_callback); 

    robot_line_strip.header.frame_id = human_line_strip.header.frame_id = "level_mux_map";
    robot_line_strip.header.stamp = human_line_strip.header.stamp = ros::Time::now();
    robot_line_strip.ns = human_line_strip.ns = "trajectory_namespace";
    robot_line_strip.action = human_line_strip.action = visualization_msgs::Marker::ADD;
    robot_line_strip.pose.orientation.w = human_line_strip.pose.orientation.w = 1.0;

    robot_line_strip.id = 1;
    human_line_strip.id = 2;

    robot_line_strip.type = human_line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    robot_line_strip.scale.x = human_line_strip.scale.x = 0.1;

    // Line strip is blue

    robot_line_strip.color.a = human_line_strip.color.a = 1.0;

    robot_line_strip.color.b = 1.0;
    human_line_strip.color.r = 1.0;

    // for adding text
    geometry_msgs::Vector3 vec; 
    vec.x = vec.y = vec.z = 1.0; 
    text_view_facing.header.frame_id = "level_mux_map";
    text_view_facing.header.stamp = ros::Time::now();
    text_view_facing.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_view_facing.ns = "trajectory_namespace";
    text_view_facing.id = 3; 
    text_view_facing.scale = vec; 
    text_view_facing.color.a = text_view_facing.color.r = 1.0; 
    text_view_facing.color.b = 0.5; 
    text_view_facing.pose.orientation.w = 1.0;

    ros::Rate r(10);

    while (task_done == false and ros::ok()) { ros::spinOnce(); }
}

