#include <ros/ros.h>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <bwi_manipulation/GraspCartesianCommand.h>
#include <pcl/common/common.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <bwi_perception/BoundingBox.h>
#include <bwi_manipulation/grasp_utils.h>

using namespace std;

namespace bwi_manipulation {
    namespace grasp_utils {

        void generate_poses_along_line(const string &frame_id,
                                       const Eigen::Vector4f &min,
                                       const Eigen::Vector4f &max,
                                       vector<geometry_msgs::PoseStamped> &poses,
                                       const geometry_msgs::Quaternion &orientation, int num_poses) {
            Eigen::Vector4f range = max - min;
            Eigen::Vector4f step_size = range / num_poses;

            geometry_msgs::PoseStamped grasp_pose;

            grasp_pose.header.frame_id = frame_id;
            grasp_pose.pose.orientation = orientation;

            for (int i = 0; i < num_poses; ++i) {
                Eigen::Vector4f position = min + step_size * i;
                grasp_pose.pose.position.x = position.x();
                grasp_pose.pose.position.y = position.y();
                grasp_pose.pose.position.z = position.z();
                poses.push_back(grasp_pose);
            }
        }


        void
        generate_grasps_varying_orientation(const string &frame_id, const geometry_msgs::Quaternion &center_orientation,
                                            const double angle_radius, const Eigen::Vector4f &position,
                                            vector<geometry_msgs::PoseStamped> &poses) {
            tf::Quaternion q;
            tf::quaternionMsgToTF(center_orientation, q);
            tf::Matrix3x3 m(q);

            // Get the min RPY values
            double b_r, b_p, b_y;
            m.getRPY(b_r, b_p, b_y);
            b_r -= angle_radius;
            b_p -= angle_radius;
            b_y -= angle_radius;

            int steps = 3;

            double step_size = angle_radius * 2.0 / steps;

            geometry_msgs::PoseStamped grasp_pose;

            grasp_pose.header.frame_id = frame_id;
            grasp_pose.pose.position.x = position.x();
            grasp_pose.pose.position.y = position.y();
            grasp_pose.pose.position.z = position.z();

            tf::Stamped<tf::Quaternion> quat;
            quat.frame_id_ = frame_id;

            geometry_msgs::QuaternionStamped quat_stamped;

            for (int i = 0; i < steps; ++i) {
                for (int j = 0; j < steps; ++j) {
                    for (int k = 0; k < steps; ++k) {
                        quat.setRPY(b_r + i * step_size, b_p * j * step_size, b_y * step_size);
                        tf::quaternionStampedTFToMsg(quat, quat_stamped);
                        grasp_pose.pose.orientation = quat_stamped.quaternion;
                        poses.push_back(grasp_pose);
                    }
                }

            }
        }


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

        bool checkPlaneConflict(const bwi_manipulation::GraspCartesianCommand &gcc, const Eigen::Vector4f &plane_c, const float min_distance_to_plane) {
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



    }
}
