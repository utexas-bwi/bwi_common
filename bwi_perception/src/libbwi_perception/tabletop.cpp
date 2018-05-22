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

namespace bwi_perception {

    bool get_largest_plane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in, const pcl::PointIndices::Ptr &plane_indices,
                           Eigen::Vector4f &plane_coefficients,
                           const std::string &up_frame, tf::TransformListener &listener) {

        typedef pcl::PointXYZRGB PointT;
        typedef pcl::PointCloud<PointT> PointCloudT;

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

        // Create the segmentation object
        pcl::SACSegmentation<PointT> seg;
        // Optional
        seg.setOptimizeCoefficients(true);
        // Mandatory
        //look for a plane perpendicular to a given axis
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(1000);
        seg.setDistanceThreshold(0.025);


        //create the axis to use
        geometry_msgs::Vector3Stamped ros_vec;
        geometry_msgs::Vector3Stamped out_vec;
        ros_vec.header.frame_id = up_frame;
        ros_vec.vector.x = 0.0;
        ros_vec.vector.y = 0.0;
        ros_vec.vector.z = 1.0;

        ROS_INFO("Ros axis: %f, %f, %f",
                 ros_vec.vector.x, ros_vec.vector.y, ros_vec.vector.z);

        //transform the vector to the camera frame of reference
        vector_to_frame(ros_vec, in->header.frame_id, out_vec, listener);

        //set the axis to the transformed vector
        Eigen::Vector3f axis = Eigen::Vector3f(out_vec.vector.x, out_vec.vector.y, out_vec.vector.z);
        seg.setAxis(axis);

        //set an epsilon that the table can differ from the axis above by
        seg.setEpsAngle(.09); //value in radians, corresponds to approximately 5 degrees

        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(in);
        seg.segment(*plane_indices, *coefficients);

        //get the plane coefficients
        plane_coefficients(0) = coefficients->values[0];
        plane_coefficients(1) = coefficients->values[1];
        plane_coefficients(2) = coefficients->values[2];
        plane_coefficients(3) = coefficients->values[3];

        return true;
    }


    void
    filter_plane_selection(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out,
                           const double cluster_extraction_tolerance) {
        typedef pcl::PointXYZRGB PointT;
        typedef pcl::PointCloud<PointT> PointCloudT;

        pcl::VoxelGrid<PointT> vg;
        //downsample the plane cloud and segment out noise
        vg.setInputCloud(in);
        vg.setLeafSize(0.005f, 0.005f, 0.005f);
        vg.filter(*out);

        //find the largest plane and segment out noise
        out = bwi_perception::get_largest_component<PointT>(out, cluster_extraction_tolerance, 500);
    }

    bool segment_tabletop_scene(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &in_cloud,
                                const double cluster_extraction_tolerance, const std::string &up_frame,
                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr &table_cloud, Eigen::Vector4f plane_coefficients,
                                std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &table_object_clouds,
                                const double plane_distance_tolerance, const double plane_max_distance_tolerance) {
        /* define what kind of point clouds we're using */
        typedef pcl::PointXYZRGB PointT;
        typedef pcl::PointCloud<PointT> PointCloudT;
        PointCloudT::Ptr cloud_blobs(new PointCloudT);
        PointCloudT::Ptr working(new PointCloudT);
        pcl::PointIndices::Ptr table_indices(new pcl::PointIndices());


        //create listener for transforms
        tf::TransformListener tf_listener;

        get_largest_plane(in_cloud, table_indices, plane_coefficients, up_frame, tf_listener);

        pcl::ExtractIndices<PointT> extract;
        // Extract the plane
        extract.setInputCloud(in_cloud);
        extract.setIndices(table_indices);
        extract.setNegative(false);
        extract.filter(*working);

        filter_plane_selection(working, table_cloud, cluster_extraction_tolerance);

        extract.setNegative(true);
        extract.filter(*cloud_blobs);

        std::vector<PointCloudT::Ptr> clusters;

        //Step 3: Eucledian Cluster Extraction
        bwi_perception::compute_clusters<PointT>(cloud_blobs, clusters, cluster_extraction_tolerance);

        //if true, clouds on the other side of the plane will be rejected
        bool check_below_plane = true;
        Eigen::Vector4f plane_centroid;

        if (check_below_plane) {
            auto table_box_orig = BoundingBox::from_cloud<PointT>(table_cloud);
            auto centroid = table_box_orig.centroid;
            Eigen::Vector3f out_vector;
            Eigen::Vector3f in_vector(centroid.x(), centroid.y(), centroid.z());
            vector_to_frame(in_vector, table_box_orig.frame_id, up_frame, out_vector, tf_listener);


            ROS_INFO("[table_object_detection_node.cpp] Plane xyz: %f, %f, %f", centroid(0), centroid(1),
                     centroid(2));
        }

        for (auto &cluster: clusters) {

            // Check if the cluster is too far away
            if (!bwi_perception::filter<PointT>(cluster, plane_coefficients, plane_distance_tolerance,
                                                plane_max_distance_tolerance)) {
                continue;
            }

            //next check which clusters are below and which are aboe the plane
            if (!check_below_plane) {
                table_object_clouds.push_back(cluster);
                continue;
            }

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