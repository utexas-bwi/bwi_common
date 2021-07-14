#include <bwi_perception/tabletop.h>
#include <bwi_perception/bwi_perception.h>

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <tf/transform_listener.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <bwi_perception/bwi_perception.h>
#include <bwi_perception/filter.h>
#include <bwi_perception/plane.h>

namespace bwi_perception {

    bool segment_tabletop_scene(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &in_cloud,
                                const double cluster_extraction_tolerance, const std::string &up_frame,
                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr &table_cloud, Eigen::Vector4f plane_coefficients,
                                std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &table_object_clouds,
                                const double plane_distance_tolerance, const double plane_max_distance_tolerance, double height,
                                const double min_cluster_size, const double max_cluster_size) {
        /* define what kind of point clouds we're using */
        typedef pcl::PointXYZRGB PointT;
        typedef pcl::PointCloud<PointT> PointCloudT;
        PointCloudT::Ptr cloud_blobs(new PointCloudT);
        PointCloudT::Ptr working(new PointCloudT);
        pcl::PointIndices::Ptr table_indices(new pcl::PointIndices());


        //create listener for transforms
        tf::TransformListener tf_listener;

        get_largest_plane<PointT>(in_cloud, table_indices, plane_coefficients, up_frame, tf_listener);

        pcl::ExtractIndices<PointT> extract;
        // Extract the plane
        extract.setInputCloud(in_cloud);
        extract.setIndices(table_indices);
        extract.setNegative(false);
        extract.filter(*working);

        filter_plane_selection<PointT>(working, table_cloud, cluster_extraction_tolerance);

        extract.setNegative(true);
        extract.filter(*cloud_blobs);

        //Step 3: Eucledian Cluster Extraction
        pcl::PointCloud<pcl::PointXYZ>::Ptr structure_only(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud_blobs, *structure_only);
        std::vector<pcl::PointIndices> cluster_indices;
        bwi_perception::compute_clusters<pcl::PointXYZ>(structure_only, cluster_indices, cluster_extraction_tolerance,
                                                        min_cluster_size, max_cluster_size);

        std::vector<PointCloudT::Ptr> clusters;
        transform(cluster_indices.begin(), cluster_indices.end(), back_inserter(clusters),
                  [cloud_blobs](pcl::PointIndices indices) {
                      return typename PointCloudT::Ptr(new PointCloudT(*cloud_blobs, indices.indices));
                  });
        //if true, clouds on the other side of the plane will be rejected
        Eigen::Vector4f plane_centroid;
        {
            auto table_box_orig = BoundingBox::from_cloud<PointT>(table_cloud);
            auto centroid = table_box_orig.centroid;
            Eigen::Vector3f out_vector;
            Eigen::Vector3f in_vector(centroid.x(), centroid.y(), centroid.z());
            vector_to_frame(in_vector, table_box_orig.frame_id, up_frame, out_vector, tf_listener);
            plane_centroid = {out_vector.x(), out_vector.y(), out_vector.z(), 1};
	    height = out_vector.z();
        }

        for (auto &cluster: clusters) {
            // Check if the cluster is too far away
            if (!bwi_perception::within_plane_margin<PointT>(cluster, plane_coefficients, plane_distance_tolerance,
                                                             plane_max_distance_tolerance)) {
                continue;
            }

            //next check which clusters are below and which are aboe the plane

            auto cluster_box_orig = BoundingBox::from_cloud<PointT>(cluster);
            auto centroid = cluster_box_orig.centroid;
            Eigen::Vector3f out_vector;
            Eigen::Vector3f in_vector(centroid.x(), centroid.y(), centroid.z());
            vector_to_frame(in_vector, cluster_box_orig.frame_id, up_frame, out_vector, tf_listener);

            //check z's
            if (cluster_box_orig.centroid(2) < plane_centroid(2)) {
                //below the table (maybe the edge of the table, etc)
                ROS_INFO("[table_object_detection_node.cpp] Rejected as below the table...");
            } else {
                //above the table
                table_object_clouds.push_back(cluster);
            }

        }

        ROS_INFO("Found %i clusters on the plane.", (int) table_object_clouds.size());


        return true;

    }
}
