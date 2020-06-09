#pragma once


#include <ros/ros.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#ifdef AGILE_GRASP_AVAILABLE
#include <agile_grasp/Grasp.h>
#endif
#include <bwi_manipulation/GraspCartesianCommand.h>
#include <bwi_perception/BoundingBox.h>

#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
namespace bwi_manipulation {
    namespace grasp_utils {

        void generate_poses_along_line(const std::string &frame_id,
                                       const Eigen::Vector4f &min,
                                       const Eigen::Vector4f &max,
                                       std::vector<geometry_msgs::PoseStamped> &poses,
                                       const geometry_msgs::Quaternion &orientation, int num_poses=10);

        void generate_grasps_varying_orientation(const std::string &frame_id,
                                                 const geometry_msgs::Quaternion &center_orientation,
                                                 const double angle_radius, const Eigen::Vector4f &position,
                                                 std::vector<geometry_msgs::PoseStamped> &poses);

        Eigen::Matrix3d reorderHandAxes(const Eigen::Matrix3d &Q);

        bool checkPlaneConflict(const GraspCartesianCommand &gcc, const Eigen::Vector4f &plane_c, float min_distance_to_plane);

        tf::Stamped<tf::Quaternion>
        compose_quaternions(const tf::Stamped<tf::Quaternion> &lhs, const tf::Stamped<tf::Quaternion> &rhs) {
            // This function is only correct if the two quaternions live in the same frame
            assert(lhs.frame_id_ == rhs.frame_id_);
            tf::Stamped<tf::Quaternion> result;
            tf::Quaternion rotated = lhs * rhs;
            rotated.normalize();
            result.setData(rotated);
            result.frame_id_= lhs.frame_id_;
            return result;
        }

        template<typename T>
        std::vector<GraspCartesianCommand>
        generate_heuristic_grasps(const typename pcl::PointCloud<T>::Ptr &target_cloud,
                                  const geometry_msgs::QuaternionStamped &grasp_x_orientation,
                                  const double approach_offset,
                                  const double grasp_offset,
                                  const tf::TransformListener &listener = tf::TransformListener()) {

            typedef typename pcl::PointCloud<T> PointCloudT;
            std::vector<geometry_msgs::PoseStamped> poses;
            std::string grasp_frame_id = grasp_x_orientation.header.frame_id;
            // Move from the sensor frame to the arm base frame
            const typename PointCloudT::Ptr &arm_frame(target_cloud);
            pcl_ros::transformPointCloud(grasp_frame_id, *arm_frame, *arm_frame, listener);
            auto boundingBox = bwi_perception::BoundingBox::from_cloud<T>(arm_frame);



            // We assume that the gripper points along the X axis. If this isn't the case,
            // pass a quaternion representing the transformation that corrects for axis
            // differences.
            tf::Stamped<tf::Quaternion> origin_orientation;
            tf::quaternionMsgToTF(grasp_x_orientation.quaternion, origin_orientation);
            origin_orientation.frame_id_= grasp_x_orientation.header.frame_id;
            geometry_msgs::QuaternionStamped quat_stamped;
            tf::Stamped<tf::Quaternion> quat;
            quat.frame_id_ = grasp_x_orientation.header.frame_id;


            // Center of the object, but the minimum along the X axis
            Eigen::Vector4f min = boundingBox.position;
            min.x() = boundingBox.min.x() + grasp_offset;
            min.z() = boundingBox.min.z();
            // Vary along Z axis between object min and max
            Eigen::Vector4f max = min;
            max.z() = boundingBox.max.z();


            generate_poses_along_line(boundingBox.frame_id, min, max, poses,
                                      grasp_x_orientation.quaternion);

            Eigen::Vector4f grasp_point = boundingBox.position;
            grasp_point.z() = boundingBox.max.z() - grasp_offset;

            // Point down
            quat.setRPY(0.0, M_PI, 0);

            quat = compose_quaternions(origin_orientation, quat);
            tf::quaternionStampedTFToMsg(quat, quat_stamped);

            generate_grasps_varying_orientation(boundingBox.frame_id, quat_stamped.quaternion, 0.25, grasp_point,
                                                poses);

            // Right side grasp
            min = boundingBox.position;
            min.y() = boundingBox.min.y() + grasp_offset;
            min.z() = boundingBox.min.z();
            // Vary along Z axis between object min and max
            max = min;
            max.z() = boundingBox.max.z();
            // Point left
            quat.setRPY(0.0, 0.0, M_PI / 2);
            quat = compose_quaternions(origin_orientation, quat);
            tf::quaternionStampedTFToMsg(quat, quat_stamped);
            generate_poses_along_line(boundingBox.frame_id, min, max, poses,
                                      quat_stamped.quaternion);

            // Left side grasp
            min = boundingBox.position;
            min.y() = boundingBox.max.y() - grasp_offset;
            min.z() = boundingBox.min.z();
            max = min;
            max.z() = boundingBox.max.z();
            // Point right
            quat.setRPY(0.0, 0.0, -M_PI / 2);
            quat = compose_quaternions(origin_orientation, quat);
            tf::quaternionStampedTFToMsg(quat, quat_stamped);
            generate_poses_along_line(boundingBox.frame_id, min, max, poses,
                                      quat_stamped.quaternion);

            std::vector<GraspCartesianCommand> grasp_commands(poses.size());
            std::transform(poses.begin(), poses.end(), grasp_commands.begin(), [=](geometry_msgs::PoseStamped pose) {
                return bwi_manipulation::GraspCartesianCommand::from_grasp_pose(pose, approach_offset);
            });

            return grasp_commands;

        }

    }
}