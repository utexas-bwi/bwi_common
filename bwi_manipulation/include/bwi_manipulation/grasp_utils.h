#ifndef GRASP_UTILS_H
#define GRASP_UTILS_H

#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>


#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

//tf stuff
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include "agile_grasp/Grasps.h"
#include "agile_grasp/Grasp.h"

struct GraspCartesianCommand {
    sensor_msgs::JointState approach_joint_state;
    geometry_msgs::PoseStamped approach_pose;

    sensor_msgs::JointState grasp_joint_state;
    geometry_msgs::PoseStamped grasp_pose;

};

namespace bwi_manipulation {
    namespace grasp_utils {

        Eigen::Matrix3d reorderHandAxes(const Eigen::Matrix3d &Q);

        bool checkPlaneConflict(GraspCartesianCommand gcc, Eigen::Vector4f plane_c, float min_distance_to_plane);

        GraspCartesianCommand grasp_command_from_agile_grasp(const agile_grasp::Grasp &grasp,
                                                             float offset_approach, const float offset_grasp,
                                                             const std::string &object_cloud_frameid);

        GraspCartesianCommand grasp_command_from_grasp_pose(const geometry_msgs::PoseStamped &grasp_pose,
                                                            const double offset_approach = 0.1);
    }
}
#endif
