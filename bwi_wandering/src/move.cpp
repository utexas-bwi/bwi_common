#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Bool.h"


bool proceed;
ros::Time proceed_time;
ros::Time temp_time;

void safe_cb(const std_msgs::Bool::ConstPtr& msg) {
  proceed = msg->data;
  proceed_time = ros::Time::now(); 
}


int main(int argc, char **argv) {
    ROS_INFO("Started");
    ros::init(argc, argv, "move");
    ros::NodeHandle nh_;

    ros::Publisher twist_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Publisher temp_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_temp", 1000);
    ros::Subscriber safe_sub = nh_.subscribe("/cmd_safe", 1000, safe_cb);

    proceed = false;
    proceed_time = ros::Time::now();
    temp_time = ros::Time::now();

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

    temp_pub.publish(twist_msg);

    ros::Duration timeout2(2.0);
    ROS_INFO("Spinning for 0.5 seconds");
    while(ros::Time::now()-start_time < timeout2)
      ros::spinOnce();

    if(proceed && proceed_time > temp_time)
      twist_pub.publish(twist_msg);


    // twist_msg.angular.z = 3.14/4;

    // twist_pub.publish(twist_msg);
    // ros::Duration(2).sleep();

    ROS_INFO("DONE WITH MOVEMENT");
    return 0;
}
