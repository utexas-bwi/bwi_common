#ifndef BWI_PERCEPTION_CONVENIENCE_H
#define BWI_PERCEPTION_CONVENIENCE_H

#include <ros/ros.h>
#include <bwi_perception/PerceiveTabletopScene.h>
#include <bwi_perception/PerceiveLargestHorizontalPlane.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

namespace bwi_perception {
    bwi_perception::PerceiveTabletopScene::Response getTabletopScene(ros::NodeHandle &n);

    bwi_perception::PerceiveLargestHorizontalPlane::Response perceiveLargestHorizontalPlane(ros::NodeHandle &n);


    void quaternion_to_frame(const tf::Stamped<tf::Quaternion> &quaternion, const std::string &target_frame,
                                             tf::Stamped<tf::Quaternion> &out_quaternion, tf::TransformListener &tf_listener);

    void vector_to_frame(const geometry_msgs::Vector3Stamped &vector, const std::string &target_frame,
                         geometry_msgs::Vector3Stamped &out_vector, tf::TransformListener &tf_listener);

    void vector_to_frame(const Eigen::Vector3f &vector, const std::string &frame, const std::string &target_frame,
                         Eigen::Vector3f &out_vector, tf::TransformListener &tf_listener);

    template <typename PointT>
    void cloud_to_frame(const typename pcl::PointCloud<PointT>::Ptr &input, const std::string &target_frame, typename pcl::PointCloud<PointT>::Ptr &output, tf::TransformListener &listener) {
        ROS_INFO("Transforming Input Point Cloud to %s frame...", target_frame.c_str());
        ROS_INFO("    Input Cloud Size: %zu", input->size());
        if (input->header.frame_id == target_frame) {
            output = input;
            return;
        }
        while (ros::ok()) {
            tf::StampedTransform stamped_transform;
            try {
                // Look up transform
                listener.lookupTransform(target_frame, input->header.frame_id, ros::Time(0), stamped_transform);

                // Apply transform
                pcl_ros::transformPointCloud(*input, *output, stamped_transform);

                // Store Header Details
                output->header.frame_id = target_frame;
                pcl_conversions::toPCL(ros::Time::now(), output->header.stamp);

                break;
            }
                //keep trying until we get the transform
            catch (tf::TransformException &ex) {
                ROS_ERROR_THROTTLE(1, "%s", ex.what());
                ROS_WARN_THROTTLE(1, "    Waiting for transform from cloud frame (%s) to %s frame. Trying again",
                                  input->header.frame_id.c_str(), target_frame.c_str());
                continue;
            }
        }
    }

}
#endif //BWI_PERCEPTION_CONVENIENCE_H
