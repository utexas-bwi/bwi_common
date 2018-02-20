#include <bwi_manipulation/GraspCartesianCommand.h>
#include <Eigen/Dense>
#include <agile_grasp/Grasp.h>
#include <eigen_conversions/eigen_msg.h>

namespace bwi_manipulation {
    GraspCartesianCommand GraspCartesianCommand::from_agile_grasp(const agile_grasp::Grasp &grasp,
                                                                                const float offset_approach,
                                                                                const float offset_grasp,
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
    GraspCartesianCommand::from_grasp_pose(const geometry_msgs::PoseStamped &grasp_pose,
                                                         const double offset_approach) {
        GraspCartesianCommand command;
        command.grasp_pose = grasp_pose;
        // We assume that the appr

        Eigen::Vector3d approach_axis(0, 0, 1);
        Eigen::Quaterniond quat;
        tf::quaternionMsgToEigen(grasp_pose.pose.orientation, quat);
        approach_axis = quat * approach_axis;

        geometry_msgs::PoseStamped approach_pose = grasp_pose;
        approach_pose.pose.position.x -= offset_approach * approach_axis.x();
        approach_pose.pose.position.y -= offset_approach * approach_axis.y();
        approach_pose.pose.position.z -= offset_approach * approach_axis.z();
        command.approach_pose = approach_pose;
        return command;
    }
}