#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"


sensor_msgs::LaserScan laser_msg;

void laser_cb(const sensor_msgs::LaserScan::ConstPtr& msg) {
  ROS_INFO("received laserscan msg\n");
  laser_msg = *msg;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "safety");
    ros::NodeHandle nh_;
    ros::Subscriber laser_sub = nh_.subscribe("/laserscan", 1000, laser_cb);
    ros::Publisher twist_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Publisher laser_poss_pub = nh_.advertise<std_msgs::Bool>("/is_possible",1000);
    // arbritrary twist msg for now
    ros::spinOnce();
    geometry_msgs::Twist twist_msg; 
    std_msgs::Bool is_poss_msg;
    twist_msg.linear.x = 0.25;
    twist_msg.angular.z = 0.75;
    // assuming that this value is not arbritary, checking if it is okay to proceed
    if (laser_msg.ranges[twist_msg.angular.z] > twist_msg.linear.x) {
      ROS_INFO ("Valid Twist Message. No obstacle detected by laser.");
      twist_pub.publish(twist_msg);
      is_poss_msg.data = true;
      laser_poss_pub.publish(is_poss_msg);
    }
    else {
      ROS_INFO ("Obstacle detected. Can't proceed");
      is_poss_msg.data = false;
      laser_poss_pub.publish(is_poss_msg);
    }

    return 0;
}
