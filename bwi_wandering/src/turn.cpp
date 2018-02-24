#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/Twist.h"
#include <map>
#include <iostream>
#include <fstream>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <string>
#include <stdio.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

bool marker_seen = false;

visualization_msgs::Marker current_vis_msg;
geometry_msgs::PoseStamped tag_base_link_pose;
geometry_msgs::PoseStamped camera_pose;

int cur_tag_id = 0;
geometry_msgs::PoseWithCovarianceStamped robot_pose;

/* Call back method to get the pose of AR marker */
void vis_cb(const visualization_msgs::Marker::ConstPtr& msg) {
    current_vis_msg = *msg;
    marker_seen = true;
}

/* Call back method to get the pose of Segbot */
void amcl_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  robot_pose = *msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "data_collection");
    ros::NodeHandle n;
    tf::TransformListener tf_l;

    //subscriber to get tag information
    ros::Subscriber vis_sub = n.subscribe("/visualization_marker", 1, vis_cb);

    //subscriber to get robot location
    ros::Subscriber amcl_sub = n.subscribe("/amcl_pose", 1, amcl_cb);

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);

    while(ros::ok()) {
      ros::spinOnce();
      ros::Duration(2).sleep();
      if(marker_seen) {
        try {
          camera_pose.header = current_vis_msg.header;
          camera_pose.pose = current_vis_msg.pose;
          tf_l.waitForTransform("/base_link", camera_pose.header.frame_id, ros::Time(0), ros::Duration(4));
          tf_l.transformPose("/base_link", camera_pose, tag_base_link_pose);
        }
        catch (tf::TransformException ex) {ROS_INFO ("%s", ex.what());}

        move_base_msgs::MoveBaseGoal goal;

        double yaw_r = tf::getYaw(robot_pose.pose.pose.orientation);
        double yaw_t = tf::getYaw(tag_base_link_pose.pose.orientation);

        double delta_yaw = yaw_t - yaw_r;


        ROS_INFO("robot: %f     tag: %f  ", yaw_r, yaw_t);


        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.header.frame_id = "/base_link";
        goal.target_pose.pose.position.x = 0.0;
        goal.target_pose.pose.position.y = 0.0;
        goal.target_pose.pose.position.z = 0.0;
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(delta_yaw);

        ac.sendGoal(goal);
        ac.waitForResult();
      }
      marker_seen = false;
    }
  }
