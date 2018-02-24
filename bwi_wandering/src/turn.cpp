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
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/Dense"

bool marker_seen = false; // flag to check if the robot detected a marker

visualization_msgs::Marker current_vis_msg; //pose info of tag detected
geometry_msgs::PoseStamped tag_base_link_pose; //transformed pose of tag wrt base_link
geometry_msgs::PoseStamped camera_pose; //casting marker type to posestamped type

/* Call back method to get the pose of AR marker */
void vis_cb(const visualization_msgs::Marker::ConstPtr& msg) {
    current_vis_msg = *msg;
    marker_seen = true;
}

/* Method to convert a vector into a 4x4 matrix */
Eigen::Quaternionf inverse_quat(geometry_msgs::Pose cur_pose) {
    geometry_msgs::Quaternion quat = cur_pose.orientation;
    Eigen::Quaternionf quat_eigen = Eigen::Quaternionf(quat.w, quat.x, quat.y, quat.z);
    return quat_eigen.inverse();
}

/* Method to convert Eigen quat to ROS(tf) quat */
geometry_msgs::Quaternion eigen_to_geoQuat (Eigen::Quaternionf quat) {
  geometry_msgs::Quaternion q;
  q.x = quat.x(); q.y = quat.y(); q.w = quat.w(); q.z = quat.z();
  return q;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "data_collection");
    ros::NodeHandle n;
    tf::TransformListener tf_l;

    //subscriber to get tag information
    ros::Subscriber vis_sub = n.subscribe("/visualization_marker", 1, vis_cb);

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);

    while(ros::ok()) {
      ros::spinOnce();
      ros::Duration(2).sleep();
      if(marker_seen) {
        try {
          /* transform tag wrt base_link
             first need to change current_vis_msg into posestamped type */
          camera_pose.header = current_vis_msg.header;
          camera_pose.pose = current_vis_msg.pose;
          ROS_INFO("Header frame id : %s" , camera_pose.header.frame_id.c_str());
          camera_pose.pose = current_vis_msg.pose;
          tf_l.waitForTransform("/base_link", camera_pose.header.frame_id, ros::Time(0), ros::Duration(4));
          tf_l.transformPose("/base_link", camera_pose, tag_base_link_pose);
        }
        catch (tf::TransformException ex) {ROS_INFO ("%s", ex.what());}

        move_base_msgs::MoveBaseGoal goal;

        Eigen::Quaternionf quat = inverse_quat (tag_base_link_pose.pose);
        geometry_msgs::Quaternion q = eigen_to_geoQuat(quat);
        double yaw_t = tf::getYaw(q);

        double delta_yaw = yaw_t;

        ROS_INFO("delta_yaw : %f ", delta_yaw);

        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.header.frame_id = "/base_link";
        goal.target_pose.pose.position.x = 0.0;
        goal.target_pose.pose.position.y = 0.0;
        goal.target_pose.pose.position.z = 0.0;
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(delta_yaw);

        ac.sendGoal(goal);
        ac.waitForResult();
        marker_seen = false;
        ros::Duration(2).sleep();
      }
      marker_seen = false;
    }
  }
