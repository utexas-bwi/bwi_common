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

namespace bwi_perception {

    enum Axis {
    X,
        Y,
        Z
    };
	
	pcl::PointCloud<pcl::Normal>::Ptr computeNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, double normals_search_radius) {
		// Create the normal estimation class, and pass the input dataset to it
		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
		ne.setInputCloud (cloud);

		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
		ne.setSearchMethod (tree);

		// Output datasets
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

		// Use all neighbors in a sphere of radius 3cm
		ne.setRadiusSearch (normals_search_radius);

		// Compute the features
		ne.compute (*cloud_normals);
		// Check for undefined values
		for (int i = 0; i < cloud_normals->points.size(); i++)
		{
			if (!pcl::isFinite<pcl::Normal>(cloud_normals->points[i]))
			{
				PCL_WARN("normals[%d] is not finite\n", i);
			}
		}

		return cloud_normals;
	}
	
	std::vector<double> computeCVFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, double normals_search_radius) {
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = computeNormals(cloud,normals_search_radius);
		ROS_INFO("Computing CVFH...");

		// Create the CVFH estimation class, and pass the input dataset+normals to it
		pcl::CVFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> cvfh;
		cvfh.setInputCloud (cloud);
		cvfh.setInputNormals (cloud_normals);

		// Create an empty kdtree representation, and pass it to the FPFH estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
		cvfh.setSearchMethod (tree);

		pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

		cvfh.compute(*vfhs);

		// Convert into normalized vector
		int histogram_sum = 0;
		std::vector<double> histogram_double_vector (308);
		for (int i = 0; i < histogram_double_vector.size(); i++) {
			histogram_double_vector[i] = vfhs->points[0].histogram[i];
			histogram_sum += vfhs->points[0].histogram[i];
		}
		for (int i = 0; i < histogram_double_vector.size(); i++) {
			histogram_double_vector[i] /= (double)histogram_sum;
		}

		return histogram_double_vector;
	}
	
	std::vector<double> computeFPFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, double neighbor_radius, double donormal_search_radius) {
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = computeNormals(cloud,donormal_search_radius);
		ROS_INFO("Computing FPFH...");

		// Create the FPFH estimation class, and pass the input dataset+normals to it
		pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
		fpfh.setInputCloud (cloud);
		fpfh.setInputNormals (cloud_normals);
		// alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

		// Create an empty kdtree representation, and pass it to the FPFH estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

		fpfh.setSearchMethod (tree);

		// Output datasets
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());


		// Use all neighbors in a sphere of radius 5cm
		// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
		fpfh.setRadiusSearch (neighbor_radius);

		// Compute the features
		fpfh.compute (*fpfhs);
		
		// Convert into normalized vector
		int histogram_sum = 0;
		std::vector<double> histogram_double_vector (308);
		for (int i = 0; i < histogram_double_vector.size(); i++) {
			histogram_double_vector[i] = fpfhs->points[0].histogram[i];
			histogram_sum += fpfhs->points[0].histogram[i];
		}
		for (double &i : histogram_double_vector) {
            i /= (double)histogram_sum;
		}

		return histogram_double_vector;
		
		// fpfhs->points.size() should have the same size as the input cloud->points.size ()*
		// return fpfhs;      Put this back in the function definition if returning fpfhs---> pcl::PointCloud<pcl::FPFHSignature33>::Ptr 
	}
	
	std::vector<std::vector<std::vector<uint> > > computeRGBColorHistogram(pcl::PointCloud<pcl::PointXYZRGB> &cloud, int dim) {
     
		//a 3D array
		std::vector<std::vector<std::vector<uint> > > hist3;
     
		hist3.resize(dim);
        for (int i = 0; i < dim; i++) {
            hist3[i].resize(dim);
            for (int j = 0; j < dim; j++) {
                hist3[i][j].resize(dim);
                std::fill( hist3[i][j].begin(), hist3[i][j].end(), 0 );
            }
        }
     
        int cloud_size = cloud.points.size();
        for (int i = 0; i < cloud_size; i++) {
            // Max value of 255. We want 256 because we want the rgb division to result in
            // a value always slightly smaller than the dim due to index starting from 0
            double round = (double)256 / dim;
            int r = (int)((double)(cloud.points[i].r) / round);
            int g = (int)((double)(cloud.points[i].g) / round);
            int b = (int)((double)(cloud.points[i].b) / round);
            hist3[r][g][b]++;
        }
        
        return hist3;
    }
	
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

    bwi_perception::PerceiveTabletopScene::Response getTabletopScene(ros::NodeHandle n) {

        ros::ServiceClient client_tabletop_perception = n.serviceClient<bwi_perception::PerceiveTabletopScene>(
                "perceive_tabletop_scene");

        bwi_perception::PerceiveTabletopScene srv;
        if (client_tabletop_perception.call(srv)) {
            return srv.response;
        } else {
            ROS_ERROR("Failed to call service tabletop perception service");
            return srv.response;
        }
    }


// re-orders detected, non-overlapping clusters according to given coordinate and direction
    bool compare_by_second(const std::pair<int, float> &lhs, const std::pair<int, float> &rhs) {
        return lhs.second > rhs.second;
    }

    template<typename T>
    void order_clouds(std::vector<typename pcl::PointCloud<T>::Ptr> &clusters_on_plane, Axis comparison_axis, bool forward = true) {
        // get representative coordinates for each cluster
        std::vector<std::pair<float, typename pcl::PointCloud<T>::Ptr>> clouds_by_coord;
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
            std::sort(clouds_by_coord.begin(), clouds_by_coord.end(), compare_by_second);
        } else {
            std::sort(clouds_by_coord.end(), clouds_by_coord.begin(), compare_by_second);
        }
        clusters_on_plane.clear();
        for (auto &pair : clouds_by_coord) {
            clusters_on_plane.push_back(pair.second);
        }
    }

    bool compare_cluster_size(const pcl::PointIndices &lhs, const pcl::PointIndices &rhs) {
        return lhs.indices.size() < rhs.indices.size();
    }

    template<typename T>
    typename pcl::PointCloud<T>::Ptr seg_largest_plane(const typename pcl::PointCloud<T>::Ptr &in, double tolerance, size_t min_num_points){
        typename pcl::search::KdTree<T>::Ptr tree (new typename pcl::search::KdTree<T>);
        tree->setInputCloud (in);
        ROS_INFO("point cloud size of 'plane cloud' : %ld", in->size());

        //use euclidean cluster extraction to eliminate noise and get largest plane
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<T> ec;
        ec.setClusterTolerance (tolerance);
        ec.setMinClusterSize (min_num_points);
        ec.setSearchMethod (tree);
        ec.setInputCloud (in);
        ec.extract (cluster_indices);

        ROS_INFO("number of 'plane clouds' : %ld", cluster_indices.size());

        if (cluster_indices.empty()) {
            throw std::exception();
        }

        pcl::PointIndices largest_cluster_indices = *std::max_element(cluster_indices.begin(), cluster_indices.end(), compare_cluster_size);

        return typename pcl::PointCloud<T>::Ptr (new typename pcl::PointCloud<T>(*in, largest_cluster_indices.indices));
    }

    template<typename T>
    bool filter(const typename pcl::PointCloud<T>::Ptr &blob, const Eigen::Vector4f plane_coefficients, double tolerance_min,
                double tolerance_max) {

        double min_distance = std::numeric_limits<double>::max();
        double max_distance = std::numeric_limits<double>::min();

        //first, we find the point in the blob closest to the plane
        for (auto &point : blob->points) {
            T p_i;
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
    template<typename T>
    void compute_clusters(const typename pcl::PointCloud<T>::Ptr &in, std::vector<typename pcl::PointCloud<T>::Ptr> &out, double tolerance) {
        typename pcl::search::KdTree<T>::Ptr tree(new typename pcl::search::KdTree<T>);
        tree->setInputCloud(in);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<T> ec;
        ec.setClusterTolerance(tolerance);
        ec.setMinClusterSize(200);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(in);
        ec.extract(cluster_indices);

        out.clear();

        for (auto &indices : cluster_indices) {
            typename pcl::PointCloud<T>::Ptr cloud_cluster(new typename pcl::PointCloud<T>(*in, indices.indices));
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            out.push_back(cloud_cluster);
        }
    }


}
