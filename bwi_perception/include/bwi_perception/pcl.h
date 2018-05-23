#ifndef BWI_PERCEPTION_PCL_H
#define BWI_PERCEPTION_PCL_H

#include "BoundingBox.h"
#include <pcl/segmentation/extract_clusters.h>
#include <bwi_perception/convenience.h>
#include <bwi_perception/comparison.h>

namespace bwi_perception {

    template<typename PointT>
    double calculate_density(const typename pcl::PointCloud<PointT>::Ptr &cloud,
                             const typename bwi_perception::BoundingBox &box) {
        // TODO: Calculate true volume
        // If the cloud is one point thick in some dimension, we'll assign that dimension a magnitude of 1cm
        double x_dim = std::max(std::abs(box.max[0] - box.min[0]), 0.01f);
        double y_dim = std::max(std::abs(box.max[1] - box.min[1]), 0.01f);
        double z_dim = std::max(std::abs(box.max[2] - box.min[2]), 0.01f);
        double volume = x_dim * y_dim * z_dim;
        return (double) cloud->size() / volume;
    }


    template<typename PointT>
    pcl::PointIndices get_largest_component_indices(const typename pcl::PointCloud<PointT>::Ptr &in, double tolerance,
                                                    size_t min_num_points) {
        typedef typename pcl::PointCloud<PointT> PointCloudT;
        typename pcl::search::KdTree<PointT>::Ptr tree(new typename pcl::search::KdTree<PointT>);
        tree->setInputCloud(in);
        ROS_INFO("point cloud size of 'plane cloud' : %ld", in->size());

        //use euclidean cluster extraction to eliminate noise and get largest plane
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(tolerance);
        ec.setMinClusterSize(min_num_points);
        ec.setSearchMethod(tree);
        ec.setInputCloud(in);
        ec.extract(cluster_indices);

        if (cluster_indices.empty()) {
            return pcl::PointIndices();
        }

        return *std::max_element(cluster_indices.begin(), cluster_indices.end(), compare_cluster_size);

    }


    template<typename PointT>
    typename pcl::PointCloud<PointT>::Ptr
    get_largest_component(const typename pcl::PointCloud<PointT>::Ptr &in, double tolerance,
                          size_t min_num_points) {
        typedef typename pcl::PointCloud<PointT> PointCloudT;
        auto largest_cluster_indices = get_largest_component_indices<PointT>(in, tolerance, min_num_points);
        return typename PointCloudT::Ptr(new PointCloudT(*in, largest_cluster_indices.indices));
    }


    template<typename T>
    void
    compute_clusters(const typename pcl::PointCloud<T>::Ptr &in, std::vector<typename pcl::PointCloud<T>::Ptr> &out,
                     double tolerance) {
        typedef typename pcl::PointCloud<T> PointCloudT;
        typename pcl::search::KdTree<T>::Ptr tree(new typename pcl::search::KdTree<T>);
        tree->setInputCloud(in);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<T> ec;
        ec.setClusterTolerance(tolerance);
        ec.setMinClusterSize(500);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(in);
        ec.extract(cluster_indices);

        out.clear();

        for (auto &indices : cluster_indices) {
            typename PointCloudT::Ptr cloud_cluster(new PointCloudT(*in, indices.indices));
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            out.push_back(cloud_cluster);
        }
    }


}

#endif //BWI_PERCEPTION_PCL_H
