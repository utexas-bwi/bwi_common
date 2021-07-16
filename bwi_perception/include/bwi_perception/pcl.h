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


    template<typename PointT>
    void
    compute_clusters(const typename pcl::PointCloud<PointT>::Ptr &in,
                     std::vector<typename pcl::PointCloud<PointT>::Ptr> &out,
                     double tolerance, double min_cluster_size = 500, double max_cluster_size = 25000) {
        typedef typename pcl::PointCloud<PointT> PointCloudT;
        typename pcl::search::KdTree<PointT>::Ptr tree(new typename pcl::search::KdTree<PointT>);
        tree->setInputCloud(in);

        std::vector<pcl::PointIndices> cluster_indices;
        compute_clusters(in, cluster_indices, tolerance, min_cluster_size, max_cluster_size);

        transform(cluster_indices.begin(), cluster_indices.end(), back_inserter(out), [in](pcl::PointIndices indices) {
            return typename PointCloudT::Ptr(new PointCloudT(*in, indices.indices));
        });
    }

    template<typename PointT>
    void
    compute_clusters(const typename pcl::PointCloud<PointT>::Ptr &in, std::vector<pcl::PointIndices> &out,
                     double tolerance, double min_cluster_size = 500, double max_cluster_size = 25000) {
        typedef typename pcl::PointCloud<PointT> PointCloudT;
        typename pcl::search::KdTree<PointT>::Ptr tree(new typename pcl::search::KdTree<PointT>);
        tree->setInputCloud(in);

        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(tolerance);
        ec.setMinClusterSize(min_cluster_size);
        ec.setMaxClusterSize(max_cluster_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(in);
        ec.extract(out);
    }


}

#endif //BWI_PERCEPTION_PCL_H
