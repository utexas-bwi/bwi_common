#pragma once


#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>

#include <sensor_msgs/PointCloud2.h>


// PCL specific includes
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>

#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/vfh.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/normal_3d.h>


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
#include <bwi_perception/PerceiveTabletopScene.h>
#include <bwi_perception/PerceiveLargestHorizontalPlane.h>
#include <bwi_perception/comparison.h>

namespace bwi_perception {

    enum Axis {
        X,
        Y,
        Z
    };

    int getLargestPointCloud(std::vector<sensor_msgs::PointCloud2> clouds_in){
        //select the object with most points as the target object
        int largest_pc_index = -1;
        int largest_num_points = -1;

        for (unsigned int i = 0; i < clouds_in.size(); i++){

            int num_points_i = clouds_in[i].height*clouds_in[i].width;

            if (num_points_i > largest_num_points){
                largest_num_points = num_points_i;
                largest_pc_index = i;
            }
        }

        return largest_pc_index;
    }


    // re-orders detected, non-overlapping clusters according to given coordinate and direction
    template<typename T>
    void order_clouds(std::vector<typename pcl::PointCloud<T>::Ptr> &clusters_on_plane, Axis comparison_axis, bool forward = true) {
        // get representative coordinates for each cluster
        typedef typename pcl::PointCloud<T> PointCloudT;
        std::vector<std::pair<float, typename PointCloudT::Ptr>> clouds_by_coord;
        for (auto c : clusters_on_plane) {
            T p = c->points.at(0);
            switch (comparison_axis) {
                case X:
                    clouds_by_coord.emplace_back(p.x, c);
                    break;
                case Y:
                    clouds_by_coord.emplace_back(p.y, c);
                    break;
                case Z:
                    clouds_by_coord.emplace_back(p.z, c);
                    break;
            }
        }

        // determine new order based on specified direction
        if (forward) {
            std::sort(clouds_by_coord.begin(), clouds_by_coord.end(), compare_by_first<float, typename PointCloudT::Ptr>);
        } else {
            std::sort(clouds_by_coord.end(), clouds_by_coord.begin(), compare_by_first<float, typename PointCloudT::Ptr>);
        }
        clusters_on_plane.clear();
        for (auto &pair : clouds_by_coord) {
            clusters_on_plane.push_back(pair.second);
        }
    }





}