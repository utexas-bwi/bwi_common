#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <cmath>

sensor_msgs::LaserScan laser_msg;
geometry_msgs::Twist meas_mes;
geometry_msgs::Twist vel_msg;

ros::Publisher safe_pub;
ros::Subscriber laser_sub;
ros::Subscriber twist_sub;

void laser_cb(const sensor_msgs::LaserScan::ConstPtr& msg) {
  ROS_INFO("Laser Data Received\n");
  laser_msg = *msg;
}

//Callback that checks if the move direction is valid
void vel_cb(const geometry_msgs::Twist::ConstPtr& msg) {
  vel_msg = *msg;
  std_msgs::Bool is_poss_msg;
  float z = vel_msg.angular.z;
  float rad_index = 0;
  if (z < laser_msg.range_min && z > laser_msg.range_max) {
    ROS_INFO ("Range out of bound");
    is_poss_msg.data = false;
    safe_pub.publish(is_poss_msg);
  }
  else {
    rad_index = std::abs(laser_msg.angle_min - z)/laser_msg.angle_increment;

    float dist = laser_msg.ranges[rad_index];
    ROS_INFO("Distance from laser is %f", dist);

    if (dist > vel_msg.linear.x && !std::isinf(dist)) {
      ROS_INFO ("Valid Twist Message. No obstacle detected by laser.");
      is_poss_msg.data = true;

    }
    else {
      ROS_INFO ("Obstacle detected. Can't proceed");
      is_poss_msg.data = false;
    }
    safe_pub.publish(is_poss_msg);
  }
}

int main(int argc, char **argv) {
    ROS_INFO("Started");
    ros::init(argc, argv, "safety");
    ros::NodeHandle nh_;

    twist_sub = nh_.subscribe("/cmd_vel", 1000, vel_cb);
    laser_sub = nh_.subscribe("/scan_filtered", 1000, laser_cb);
    safe_pub = nh_.advertise<std_msgs::Bool>("/cmd_safe",1000);

    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(0.5);
    ROS_INFO("Spinning for 0.5 seconds");
    while(ros::Time::now()-start_time < timeout)
      ros::spinOnce();

    // assuming that this value is not arbritary, checking if it is okay to proceed
    while(ros::ok())

    return 0;
}
