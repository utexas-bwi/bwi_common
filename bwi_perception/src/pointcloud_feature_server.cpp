#include <signal.h>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <sys/stat.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/vfh.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/vtk.h>
#include <pcl/visualization/pcl_plotter.h>

#include "bwi_perception/FeatureExtraction.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef unsigned int uint;

//true if Ctrl-C is pressed
bool g_caught_sigint = false;

pcl::visualization::PCLPlotter * plotter;
ros::Publisher cloud_pub;
sensor_msgs::PointCloud2 cloud_ros;

#define NORMALS_SEARCH_RADIUS 0.02
#define FPFH_NEIGHBOR_RADIUS 0.05

/* what happens when ctr-c is pressed */
void sigint_handler(int sig)
{
    g_caught_sigint = true;
    ROS_INFO("Caught sigint, init shutdown sequence...");
    ros::shutdown();
    exit(1);
}

class ColorHistogram {
  private:
    std::vector<std::vector<std::vector<uint> > > hist3;
    int dim, cloud_size;
  public:
    ColorHistogram(int dim):dim(dim) {
        cloud_size = 0;
        hist3.resize(dim);
        for (int i = 0; i < dim; i++) {
            hist3[i].resize(dim);
            for (int j = 0; j < dim; j++) {
                hist3[i][j].resize(dim);
                std::fill( hist3[i][j].begin(), hist3[i][j].end(), 0 );
            }
        }
    }

    void computeHistogram(PointCloudT &cloud) {
        ROS_INFO("Computing color histogram...");
        cloud_size = cloud.points.size();
        for (int i = 0; i < cloud_size; i++) {
            // Max value of 255. We want 256 because we want the rgb division to result in
            // a value always slightly smaller than the dim due to index starting from 0
            double round = (double)256 / dim;
            int r = (int)((double)(cloud.points[i].r) / round);
            int g = (int)((double)(cloud.points[i].g) / round);
            int b = (int)((double)(cloud.points[i].b) / round);
            hist3[r][g][b]++;
        }
    }
    uint get(int r, int g, int b) {
        return hist3[r][g][b];
    }
    // Breaks program, don't use
   std::string rosPrintHist() {
        for (int i = 0; i < dim; i++) {
            ROS_INFO("i = %d", i);
            ROS_INFO("[");
            for (int j = 0; j < dim; j++) {
                std::string output;
                for (int k = 0; k < dim; k++) {
                    output += boost::lexical_cast<std::string>(hist3[i][j][k]) + ", ";
                }
                ROS_INFO("%s", output.c_str());
            }
            ROS_INFO("]");
        }
    }
    std::vector<double> toDoubleVector() {
        int i_offset = dim * dim;
        int j_offset = dim;
        std::vector<double> hist3_double_vector (dim * dim * dim, 0);
        for (int i = 0; i < dim; i++) {
            for (int j = 0; j < dim; j++) {
                for (int k = 0; k < dim; k++) {
                    hist3_double_vector[i * i_offset + j * j_offset + k] = hist3[i][j][k];
                }
            }
        }
 
        return hist3_double_vector;
    }
    std::vector<double> toDoubleVectorNormalized() {
        std::vector<double> hist_double_vector = toDoubleVector();
        for (int i = 0; i < hist_double_vector.size(); i++) {
            hist_double_vector[i] /= cloud_size;
        }
        return hist_double_vector;
    }
};


pcl::PointCloud<pcl::Normal>::Ptr computeNormals(PointCloudT::Ptr &cloud) {
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (NORMALS_SEARCH_RADIUS);

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



std::vector<double> computeCVFH(PointCloudT::Ptr &cloud) {
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = computeNormals(cloud);
    ROS_INFO("Computing CVFH...");

    // Create the CVFH estimation class, and pass the input dataset+normals to it
    pcl::CVFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308> cvfh;
    cvfh.setInputCloud (cloud);
    cvfh.setInputNormals (cloud_normals);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
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


// http://pointclouds.org/documentation/tutorials/vfh_estimation.php#vfh-estimation
pcl::PointCloud<pcl::VFHSignature308>::Ptr computeVFH(PointCloudT::Ptr &cloud) {
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = computeNormals(cloud);
    ROS_INFO("Computing VFH...");

    // Create the VFH estimation class, and pass the input dataset+normals to it
    pcl::VFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308> vfh;
    vfh.setInputCloud (cloud);
    vfh.setInputNormals (cloud_normals);
    // alternatively, if cloud is of type PointNormal, do vfh.setInputNormals (cloud);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    vfh.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

    // Compute the features
    vfh.compute (*vfhs);

    // vfhs->points.size () should be of size 1*
    return vfhs;
}

std::vector<double> computeFPFH(PointCloudT::Ptr &cloud) {
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = computeNormals(cloud);
    ROS_INFO("Computing FPFH...");

    // Create the FPFH estimation class, and pass the input dataset+normals to it
    pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud (cloud);
    fpfh.setInputNormals (cloud_normals);
    // alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

    fpfh.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());


    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    fpfh.setRadiusSearch (FPFH_NEIGHBOR_RADIUS);

    // Compute the features
    fpfh.compute (*fpfhs);
	
    // Convert into normalized vector
    int histogram_sum = 0;
    std::vector<double> histogram_double_vector (308);
    for (int i = 0; i < histogram_double_vector.size(); i++) {
        histogram_double_vector[i] = fpfhs->points[0].histogram[i];
        histogram_sum += fpfhs->points[0].histogram[i];
    }
    for (int i = 0; i < histogram_double_vector.size(); i++) {
        histogram_double_vector[i] /= (double)histogram_sum;
    }

    return histogram_double_vector;
    
    // fpfhs->points.size() should have the same size as the input cloud->points.size ()*
    // return fpfhs;      Put this back in the function definition if returning fpfhs---> pcl::PointCloud<pcl::FPFHSignature33>::Ptr 
}


// Point Feature Histograms (PFH) descriptors
// pointclouds.org/documentation/tutorials/pfh_estimation.php
pcl::PointCloud<pcl::PFHSignature125>::Ptr computePFH(PointCloudT::Ptr &cloud) {
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ROS_INFO("Computing PFH...");

    ne.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);

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

    // Create the PFH estimation class, and pass the input dataset+normals to it
    pcl::PFHEstimation<PointT, pcl::Normal, pcl::PFHSignature125> pfh;
    pfh.setInputCloud (cloud);
    pfh.setInputNormals (cloud_normals);
    // alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);

    // Create an empty kdtree representation, and pass it to the PFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointT>::Ptr tree2 (new pcl::search::KdTree<PointT> ());
    pfh.setSearchMethod (tree2);

    // Output datasets
    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs (new pcl::PointCloud<pcl::PFHSignature125> ());

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    pfh.setRadiusSearch (0.05);
    // Compute the features
    pfh.compute (*pfhs);
}


bool colorhist_cb(
    bwi_perception::FeatureExtraction::Request &req,
    bwi_perception::FeatureExtraction::Response &res) {
    
    //how many bins per color channel
    //TO DO, check if params_int is actuall set, if not, set default
    int kColorHistBins = req.params_int[0];
    
    PointCloudT::Ptr cloud(new PointCloudT);
    pcl::fromROSMsg(req.cloud, *cloud);
    
    // Color
    ColorHistogram ch(kColorHistBins);
    ch.computeHistogram(*cloud);
    std::vector<double> color_vector = ch.toDoubleVectorNormalized();
    for (int i = 0; i < color_vector.size(); i++) {

		//fill in response
        res.feature_vector.push_back(color_vector[i]);
    }

    return true;
}

bool shapehist_cvfh_cb(
    bwi_perception::FeatureExtraction::Request &req,
    bwi_perception::FeatureExtraction::Response &res) {
    
    
    PointCloudT::Ptr cloud(new PointCloudT);
    pcl::fromROSMsg(req.cloud, *cloud);
    

	std::vector<double> feature_vector = computeCVFH(cloud);
	for (int i = 0; i < feature_vector.size(); i++) {

		//fill in response
        res.feature_vector.push_back(feature_vector[i]);
    }

	

    return true;
}


bool shapehist_fpfh_cb(
    bwi_perception::FeatureExtraction::Request &req,
    bwi_perception::FeatureExtraction::Response &res) {
    
    
    PointCloudT::Ptr cloud(new PointCloudT);
    pcl::fromROSMsg(req.cloud, *cloud);
    

	std::vector<double> feature_vector = computeFPFH(cloud);
	for (int i = 0; i < feature_vector.size(); i++) {

		//fill in response
        res.feature_vector.push_back(feature_vector[i]);
    }

	

    return true;
}



int main (int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_feature_server");
    ros::NodeHandle nh;

    signal(SIGINT, sigint_handler);

    ros::ServiceServer service_colorhist = nh.advertiseService("/bwi_perception/color_histogram_service", colorhist_cb);
	ros::ServiceServer service_shapehist_cvfh = nh.advertiseService("/bwi_perception/shape_cvfh_histogram_service", shapehist_cvfh_cb);
	ros::ServiceServer service_shapehist_fpfh = nh.advertiseService("/bwi_perception/shape_fpfh_histogram_service", shapehist_fpfh_cb);

    // Debug cloud
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("feature_extraction_server/cloud", 10);

    ROS_INFO("Feature extraction server ready");
    ros::spin();
    return 0;
}
