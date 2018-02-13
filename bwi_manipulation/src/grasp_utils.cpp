#include <ros/ros.h>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <bwi_manipulation/GraspCartesianCommand.h>
#include <pcl/common/common.h>
#include <pcl/sample_consensus/sac_model_plane.h>

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
