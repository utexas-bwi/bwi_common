#ifndef BWI_PERCEPTION_PLANE_H
#define BWI_PERCEPTION_PLANE_H

namespace bwi_perception {

    const std::string bounding_box_marker_ns = "horizontal_planes_marker";
#define Z_AXIS_REFERENCE_FRAME "base_link" // Find planes perpendicular to the z-axis of this frame
#define TARGET_FRAME "map" //target frame name DONT CHANGE!
#define EPS_ANGLE 0.05 //epsilon angle for segmenting, value in radians
#define VOXEL_LEAF_SIZE 0.02 //size of voxel leaf for processing
#define RANSAC_MAX_ITERATIONS 10000
#define PLANE_DIST_TRESH 0.025 //maximum distance from plane
#define CLUSTER_TOL 0.05 //clustering tolerance for largest plane extraction
#define MIN_NUMBER_PLANE_POINTS 500
#define MIN_PLANE_DENSITY 30000
#define STOPPING_PERCENTAGE 0.25 // Stop once we've processed all but X percentage of the cloud
#define IGNORE_FLOOR false // If the input cloud doesn't already have z filtered, we can do it
#define MIN_Z 0.05 // minimum z-value of point cloud in map frame to be considered
#define MAX_Z 2.0 // maximum z-value of point cloud in map frame to be considered

    template<typename PointT>
    bool get_largest_plane(const typename pcl::PointCloud<PointT>::Ptr &in, const pcl::PointIndices::Ptr &plane_indices,
                            Eigen::Vector4f &plane_coefficients,
                            const std::string &up_frame, tf::TransformListener &listener) {

        typedef typename pcl::PointCloud<PointT> PointCloudT;

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

        // Create the segmentation object
        typename pcl::SACSegmentation<PointT> seg;
        // Optional
        seg.setOptimizeCoefficients(true);
        // Mandatory
        //look for a plane perpendicular to a given axis
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType(pcl::SAC_PROSAC);
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

    template<typename PointT>
    void extract_horizontal_planes(const typename pcl::PointCloud<PointT>::Ptr in,
                                   std::vector<typename pcl::PointCloud<PointT>::Ptr> plane_clouds,
                                   std::vector<float> plane_coefficients, const std::string &up_frame,
                                   tf::TransformListener &listener) {
        typedef typename pcl::PointCloud<PointT> PointCloudT;
        //create onjects for use in segmenting
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        typename PointCloudT::Ptr working_cloud(in);
        typename PointCloudT::Ptr plane_cloud(in);
        typename PointCloudT::Ptr cloud_remainder(new PointCloudT);

        // Create the segmentation object
        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients(true);

        //look for a plane perpendicular to a given axis
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(RANSAC_MAX_ITERATIONS);
        seg.setDistanceThreshold(PLANE_DIST_TRESH);
        seg.setEpsAngle(EPS_ANGLE);

        //create the axis to use
        ROS_INFO("Finding planes perpendicular to z-axis of %s frame", up_frame.c_str());
        geometry_msgs::Vector3Stamped ros_vec;
        ros_vec.header.frame_id = up_frame;
        ros_vec.vector.x = 0.0;
        ros_vec.vector.y = 0.0;
        ros_vec.vector.z = 1.0;

        geometry_msgs::Vector3Stamped out_vec;
        bwi_perception::vector_to_frame(ros_vec, in->header.frame_id, out_vec, listener);

        //set the axis to the transformed vector
        Eigen::Vector3f axis = Eigen::Vector3f(out_vec.vector.x, out_vec.vector.y, out_vec.vector.z);
        seg.setAxis(axis);
        ROS_INFO("SAC axis value: %f, %f, %f", seg.getAxis()[0], seg.getAxis()[1], seg.getAxis()[2]);

        // Create the filtering object
        pcl::ExtractIndices<PointT> extract;

        // Prepare object containers for responses
        std::vector<Eigen::Vector4f> horizontal_plane_coefs;
        std::vector<std::pair<int, float> > indices_with_densities;
        int num_planes = 0;
        size_t num_start_points = working_cloud->points.size();
        while (working_cloud->points.size() > STOPPING_PERCENTAGE * num_start_points) {
            // Segment the largest planar component from the remaining cloud
            ROS_INFO("Extracting a horizontal plane...");
            seg.setInputCloud(working_cloud);
            ROS_INFO("    Number of Points to Process: %zu", working_cloud->size());

            seg.segment(*inliers, *coefficients);
            if (inliers->indices.empty()) {
                ROS_WARN("    Could not estimate a planar model for the given dataset.");
                break;
            }

            ROS_INFO("    Found a horizontal plane!");
            // Extract the inliers
            extract.setInputCloud(in);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*plane_cloud);


            // Create the filtering object to extract everything else
            extract.setNegative(true);
            extract.filter(*cloud_remainder);
            working_cloud.swap(cloud_remainder);

            // Perform clustering on this plane.
            // use the largest cluster as the representative points of the plane.

            // if no clusters are found, this is an invalid plane extraction
            try {
                double cluster_extraction_tolerance = CLUSTER_TOL;
                plane_cloud = bwi_perception::get_largest_component<PointT>(plane_cloud, cluster_extraction_tolerance,
                                                                            MIN_NUMBER_PLANE_POINTS);
                ROS_INFO("    Extracted Plane Cloud Size: %zu", plane_cloud->size());

                if (plane_cloud->size() < MIN_NUMBER_PLANE_POINTS) {
                    ROS_WARN("Plane contains insufficient points. Discarding");
                    continue;
                }

            }
            catch (std::exception &e) {
                ROS_WARN("No clusters were found. Invalid plane points exist in this iteration.");
                continue;
            }

            //get the plane coefficients
            Eigen::Vector4f plane_coefficients;
            plane_coefficients(0) = coefficients->values[0];
            plane_coefficients(1) = coefficients->values[1];
            plane_coefficients(2) = coefficients->values[2];
            plane_coefficients(3) = coefficients->values[3];
            horizontal_plane_coefs.push_back(plane_coefficients);

            // Extract the bonding box parameters of this plane

            #if PCL_VERSION_COMPARE(>=, 1, 7, 2)
            const bwi_perception::BoundingBox &oriented_bbox_params = bwi_perception::BoundingBox::oriented_from_cloud<PointT>(
                    plane_cloud);

            // Use the oriented bounding box for a better estimate of density. Non oriented box
            // penalizes shelves that don't happen to be perfectly aligned with the map frame
            double plane_bounding_box_density = bwi_perception::calculate_density<PointT>(plane_cloud,
                                                                                          oriented_bbox_params);
            #else
            const bwi_perception::BoundingBox &bbox_params = bwi_perception::BoundingBox::from_cloud<PointT>(
                    plane_cloud);
            double plane_bounding_box_density = bwi_perception::calculate_density<PointT>(plane_cloud,
                                                                                          bbox_params);
            #endif

            plane_clouds.push_back(typename PointCloudT::Ptr(plane_cloud));

            indices_with_densities.emplace_back(num_planes, (float) plane_bounding_box_density);
            num_planes += 1;
        }

        std::sort(indices_with_densities.begin(), indices_with_densities.end(), compare_by_second<int, float>);

        // Populate the response with planes sorted by density
        for (auto it = indices_with_densities.begin(); it < indices_with_densities.end(); ++it) {
            std::pair<int, float> pair = *it;
            int i = pair.first;
            float density = pair.second;

            if (density < MIN_PLANE_DENSITY) {
                ROS_INFO("Rejecting candidate plane with low density (%f)", density);
                continue;
            };


        }

    }

}

#endif //BWI_PERCEPTION_PLANE_H
