#ifndef BWI_PERCEPTION_PCL_FILTER_H
#define BWI_PERCEPTION_PCL_FILTER_H


#include "ros/ros.h"
#include "BoundingBox.h"
#include "pcl.h"
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/impl/point_types.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <bwi_perception/tabletop.h>
#include <bwi_perception/bwi_perception.h>

namespace bwi_perception {
    template<typename PointT>
    void filter_points_between(typename pcl::PointCloud<PointT>::ConstPtr input_cloud,
                               typename pcl::PointCloud<PointT>::Ptr output_cloud, Eigen::Vector2f origin, float start_angle,
                               float end_angle) {

        typedef typename pcl::PointCloud<PointT> PointCloudT;
        ROS_INFO("Removing input cloud points between %f and %f", start_angle, end_angle);


        ROS_INFO("Started with %zu points", input_cloud->size());

        float half_ignore_size = (end_angle - start_angle) / 2.0f;
        float mid_angle = (start_angle + end_angle) / 2.0f;
        Eigen::Vector2f ignore_range_bisect(cos(mid_angle), sin(mid_angle));
        ignore_range_bisect.normalize();

        for (auto i = input_cloud->begin(); i < input_cloud->end(); ++i) {
            PointT point = *i;
            Eigen::Vector2f point_vec(point.x, point.y);
            Eigen::Vector2f origin_to_center = point_vec - origin;
            origin_to_center.normalize();
            float point_cosine = origin_to_center.dot(ignore_range_bisect);
            if (acos(point_cosine) > half_ignore_size) {
                continue;
            }
            output_cloud->push_back(point);
        }
        output_cloud->header.frame_id = input_cloud->header.frame_id;
        ROS_INFO("Finished with %zu points", output_cloud->size());
    }

    template<typename PointT>
    void
    filter_plane_selection(const typename pcl::PointCloud<PointT>::Ptr in, typename pcl::PointCloud<PointT>::Ptr out,
                           const double cluster_extraction_tolerance) {
        typedef pcl::PointCloud<PointT> PointCloudT;

        // Voxelization doesn't work in place!
        assert(in != out);
        pcl::VoxelGrid<PointT> vg;
        //downsample the plane cloud and segment out noise
        vg.setInputCloud(in);
        vg.setLeafSize(0.005f, 0.005f, 0.005f);
        vg.filter(*out);

        //find the largest plane and segment out noise
        typename PointCloudT::Ptr temp_out = bwi_perception::get_largest_component<PointT>(out,
                                                                                           cluster_extraction_tolerance,
                                                                                           500);
        out = temp_out;
    }

    template<typename PointT>
    bool
    within_plane_margin(const typename pcl::PointCloud<PointT>::Ptr &blob, const Eigen::Vector4f plane_coefficients,
                        double tolerance_min,
                        double tolerance_max) {

        double min_distance = std::numeric_limits<double>::max();
        double max_distance = std::numeric_limits<double>::min();

        //first, we find the point in the blob closest to the plane
        for (auto &point : blob->points) {
            PointT p_i;
            p_i.x = point.x;
            p_i.y = point.y;
            p_i.z = point.z;

            double distance = pcl::pointToPlaneDistance(p_i, plane_coefficients);

            if (distance < min_distance) {
                min_distance = distance;
            }

            if (distance > max_distance)
                max_distance = distance;
        }

        if (min_distance > tolerance_min)
            return false;
        else if (max_distance < tolerance_max)
            return false;


        ROS_INFO("\nMin Distance to plane for cluster with %i points: %f", (int) blob->points.size(), min_distance);
        ROS_INFO("Max Distance to plane for cluster with %i points: %f", (int) blob->points.size(), max_distance);


        return true;
    }

    template<typename PointT>
    void filter_cloud_region(const typename pcl::PointCloud<PointT>::Ptr &in, typename pcl::PointCloud<PointT>::Ptr &out, const std::string &region_frame, float z_min, float z_max, float x_min, float x_max, tf::TransformListener &listener) {

        typename pcl::PointCloud<PointT>::Ptr temp(new pcl::PointCloud<PointT>());

        cloud_to_frame<PointT>(in, region_frame, temp, listener);
        // Crop the cloud
        typename pcl::CropBox<PointT> pass;
        pass.setInputCloud(temp);
        pass.setMin({x_min, -std::numeric_limits<float>::max(), z_min, 1.f});
        pass.setMax({x_max, std::numeric_limits<float>::max(), z_max, 1.f});
        pass.filter(*out);
        temp.swap(out);

        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        typename pcl::VoxelGrid<PointT> vg;
        typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new typename pcl::PointCloud<PointT>());
        vg.setInputCloud(temp);
        vg.setLeafSize(0.0025f, 0.0025f, 0.0025f);
        vg.filter(*out);
    }
}
#endif //BWI_PERCEPTION_PCL_FILTER_H
