#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cmath>

visualization_msgs::Marker line_strip;
ros::Publisher marker_pub;

void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    geometry_msgs::Point p;
    p.x = msg->pose.pose.position.x; 
    p.y = msg->pose.pose.position.y; 
    p.z = msg->pose.pose.position.z; 
    line_strip.points.push_back(p); 

    marker_pub.publish(line_strip);
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "trajectory_drawing");
    ros::NodeHandle nh;
    marker_pub = nh.advertise<visualization_msgs::Marker> ("visualization_marker", 10);

    ros::Subscriber amcl_sub = nh.subscribe("amcl_pose", 100, callback); 

    line_strip.header.frame_id = "level_mux/map";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "trajectory_namespace";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;

    line_strip.id = 1;

    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    ros::Rate r(10);
    ros::spin();
}

