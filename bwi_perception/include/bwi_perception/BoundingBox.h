#ifndef BWI_PERCEPTION_BOUNDINGBOX_H
#define BWI_PERCEPTION_BOUNDINGBOX_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <pcl/common/common.h>
#include <visualization_msgs/Marker.h>
#include <pcl/common/centroid.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <tf/transform_listener.h>

namespace bwi_perception {
    struct BoundingBox {
    public:
        Eigen::Vector4f centroid;
        Eigen::Vector4f min;
        Eigen::Vector4f max;

        Eigen::Quaternionf orientation;
        Eigen::Vector4f position;

        std::string frame_id;

        BoundingBox(const Eigen::Vector4f &min, const Eigen::Vector4f &max, const Eigen::Vector4f &centroid,
                    const Eigen::Quaternionf &orientation, const Eigen::Vector4f &position,
                    const std::string &frame_id);

        template<typename T>
        static BoundingBox from_cloud(const typename pcl::PointCloud<T>::Ptr &plane_cloud);

        template<typename T>
        static BoundingBox oriented_from_cloud(const typename pcl::PointCloud<T>::Ptr &plane_cloud);

        visualization_msgs::Marker to_marker(const int marker_index,
                                             const std::string &ns) const;

        static void transform(const std::string &target_frame, const BoundingBox &in, BoundingBox &out, const tf::TransformListener &listener = tf::TransformListener());
    };

    template<typename T>
    BoundingBox BoundingBox::from_cloud(const typename pcl::PointCloud<T>::Ptr &plane_cloud) {
        Eigen::Vector4f centroid = Eigen::Vector4f::Zero();
        pcl::compute3DCentroid(*plane_cloud, centroid);
        Eigen::Vector4f min, max = Eigen::Vector4f::Zero();
        pcl::getMinMax3D(*plane_cloud, min, max);

        Eigen::Vector4f position = min + (max - min) / 2;
        position.w() = 1;
        return BoundingBox(min, max, centroid, Eigen::Quaternionf(0, 0, 0, 1), position, plane_cloud->header.frame_id);
    }

    template<typename T>
    BoundingBox BoundingBox::oriented_from_cloud(const typename pcl::PointCloud<T>::Ptr &plane_cloud) {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*plane_cloud, centroid);

        pcl::MomentOfInertiaEstimation<T> feature_extractor;
        feature_extractor.setInputCloud(plane_cloud);
        feature_extractor.compute();

        T min, max, position;
        Eigen::Matrix3f rotational_matrix;
        feature_extractor.getOBB(min, max, position, rotational_matrix);
        Eigen::Quaternionf quat(rotational_matrix);
        // Homogeneous coordinates
        Eigen::Vector4f min_vec(min.x, min.y, min.z, 1);
        Eigen::Vector4f max_vec(max.x, max.y, max.z, 1);
        Eigen::Vector4f position_vec(position.x, position.y, position.z, 1);

        return BoundingBox(min_vec, max_vec, centroid, quat, position_vec, plane_cloud->header.frame_id);

    }

}


#endif //BWI_PERCEPTION_BOUNDINGBOX_H
