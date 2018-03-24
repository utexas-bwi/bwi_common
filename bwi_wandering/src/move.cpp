#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv) {
    ROS_INFO("Started");
    ros::init(argc, argv, "move");
    ros::NodeHandle nh_;
    ros::Publisher twist_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(0.5);
    ROS_INFO("Spinning for 0.5 seconds");
    while(ros::Time::now()-start_time < timeout)
      ros::spinOnce();


    float distance = 1;
    float inc_val = .5;


    geometry_msgs::Twist twist_msg; 
    twist_msg.linear.x = inc_val;
    twist_msg.angular.z = 0;

    twist_pub.publish(twist_msg);
    ros::Duration(2).sleep();

    twist_msg.angular.z = 3.14/4;

    twist_pub.publish(twist_msg);
    ros::Duration(2).sleep();

    ROS_INFO("DONE WITH MOVEMENT");
    return 0;
}
