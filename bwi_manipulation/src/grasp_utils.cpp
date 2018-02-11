#include <ros/ros.h>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/parse.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/time.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

//tf stuff
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include "agile_grasp/Grasps.h"

struct GraspCartesianCommand {
    sensor_msgs::JointState approach_joint_state;
    geometry_msgs::PoseStamped approach_pose;

    sensor_msgs::JointState grasp_joint_state;
    geometry_msgs::PoseStamped grasp_pose;

};

namespace bwi_manipulation {
    namespace grasp_utils {


        Eigen::Matrix3d reorderHandAxes(const Eigen::Matrix3d &Q) {
            std::vector<int> axis_order_;
            axis_order_.push_back(2);
            axis_order_.push_back(0);
            axis_order_.push_back(1);

            Eigen::Matrix3d R = Eigen::MatrixXd::Zero(3, 3);
            R.col(axis_order_[0]) = Q.col(0); // grasp approach vector
            R.col(axis_order_[1]) = Q.col(1); // hand axis
            R.col(axis_order_[2]) = Q.col(2); // hand binormal
            return R;
        }

        bool checkPlaneConflict(GraspCartesianCommand gcc, Eigen::Vector4f plane_c, float min_distance_to_plane) {
            pcl::PointXYZ p_a;
            p_a.x = gcc.approach_pose.pose.position.x;
            p_a.y = gcc.approach_pose.pose.position.y;
            p_a.z = gcc.approach_pose.pose.position.z;

            pcl::PointXYZ p_g;
            p_g.x = gcc.grasp_pose.pose.position.x;
            p_g.y = gcc.grasp_pose.pose.position.y;
            p_g.z = gcc.grasp_pose.pose.position.z;

            return !(pcl::pointToPlaneDistance(p_a, plane_c) < min_distance_to_plane
                     || pcl::pointToPlaneDistance(p_g, plane_c) < min_distance_to_plane);

        };


        GraspCartesianCommand grasp_command_from_agile_grasp(const agile_grasp::Grasp &grasp,
                                                             const float offset_approach, const float offset_grasp,
                                                             const std::string &object_cloud_frameid) {

            Eigen::Vector3d center; // grasp position
            Eigen::Vector3d surface_center; //  grasp position projected back onto the surface of the object
            Eigen::Vector3d axis; //  hand axis
            Eigen::Vector3d approach; //  grasp approach vector

            tf::vectorMsgToEigen(grasp.axis, axis);
            tf::vectorMsgToEigen(grasp.approach, approach);
            tf::vectorMsgToEigen(grasp.center, center);
            tf::vectorMsgToEigen(grasp.surface_center, surface_center);


            //step 1: calculate hand orientation
            Eigen::Quaterniond quat;
            // Swap the X and Y components of the axis
            double temp_x = axis(0);
            axis(0) = axis(1);
            axis(1) = temp_x;
            quat = Eigen::AngleAxisd(M_PI / 2, axis);


            // calculate grasp position
            Eigen::Vector3d position = surface_center + -offset_grasp * approach;

            geometry_msgs::PoseStamped grasp_msg;
            grasp_msg.header.stamp = ros::Time::now();
            grasp_msg.header.frame_id = object_cloud_frameid;
            tf::pointEigenToMsg(position, grasp_msg.pose.position);
            tf::quaternionEigenToMsg(quat, grasp_msg.pose.orientation);

            auto approach_position = position + -offset_approach * approach;
            geometry_msgs::PoseStamped approach_msg;
            approach_msg.header.stamp = ros::Time::now();
            approach_msg.header.frame_id = object_cloud_frameid;
            tf::pointEigenToMsg(approach_position, approach_msg.pose.position);
            tf::quaternionEigenToMsg(quat, approach_msg.pose.orientation);
            GraspCartesianCommand gc;
            gc.approach_pose = approach_msg;
            gc.grasp_pose = grasp_msg;

            return gc;
        }

        GraspCartesianCommand
        grasp_command_from_grasp_pose(const geometry_msgs::PoseStamped &grasp_pose, const double offset_approach) {
            GraspCartesianCommand command;
            command.grasp_pose = grasp_pose;

        }
    }
}
