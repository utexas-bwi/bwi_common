#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <cmath>

sensor_msgs::LaserScan laser_msg;
ros::Publisher safe_pub;

void laser_cb(const sensor_msgs::LaserScan::ConstPtr& msg) {
  // ROS_INFO("Laser Data Received\n");
  laser_msg = *msg;
}

//Callback that checks if the move direction is valid
void vel_cb(const geometry_msgs::Twist::ConstPtr& msg) {
  ROS_INFO("Reached Here");
  geometry_msgs::Twist vel_msg = *msg;
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

    int index_min = rad_index-5 >= 0? rad_index-5 : rad_index;
    int index_max = rad_index+5 < laser_msg.ranges.size()? rad_index+5 : rad_index;

    is_poss_msg.data = true;

    for (int i = index_min; i <= index_max; i++) {
      float dist = laser_msg.ranges[i];
      ROS_INFO("Index: %d", i);
      ROS_INFO("Distance from laser is %f", dist);
      if (std::isinf(dist))
        continue;
      if (!(dist > vel_msg.linear.x))
        is_poss_msg.data = false;
    }

    if (is_poss_msg.data) 
      ROS_INFO ("Valid Twist Message. No obstacle detected by laser.");
    else 
      ROS_INFO ("Obstacle detected. Can't proceed");

    safe_pub.publish(is_poss_msg);
  }
}

int main(int argc, char **argv) {
    ROS_INFO("Started");
    ros::init(argc, argv, "safety");
    ros::NodeHandle nh_;

    ros::Subscriber twist_sub = nh_.subscribe("/cmd_vel_temp", 1000, vel_cb);
    ros::Subscriber laser_sub = nh_.subscribe("/scan_filtered", 1000, laser_cb);
    safe_pub = nh_.advertise<std_msgs::Bool>("/cmd_safe",1000);

    ros::spin();

    return 0;
}
