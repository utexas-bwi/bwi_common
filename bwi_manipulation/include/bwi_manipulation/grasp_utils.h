#ifndef GRASP_UTILS_H
#define GRASP_UTILS_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#include "agile_grasp/Grasp.h"
#include <bwi_manipulation/GraspCartesianCommand.h>

namespace bwi_manipulation {
    namespace grasp_utils {

        Eigen::Matrix3d reorderHandAxes(const Eigen::Matrix3d &Q);

        bool checkPlaneConflict(const GraspCartesianCommand &gcc, const Eigen::Vector4f &plane_c, float min_distance_to_plane);

    }
}
#endif
