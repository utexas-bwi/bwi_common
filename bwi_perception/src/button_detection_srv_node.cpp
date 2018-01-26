#include <signal.h>
#include <vector>
#include <string>
#include <sys/stat.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <pcl_ros/impl/transforms.hpp>

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

#include "bwi_perception/ButtonDetection.h"

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//threshold 
static int red_min;

// Select mode
const bool save_pl_mode = false;

// Mutex: //
boost::mutex cloud_mutex;

bool new_cloud_available_flag = false;
PointCloudT::Ptr cloud (new PointCloudT);
PointCloudT::Ptr cloud_aggregated (new PointCloudT);
PointCloudT::Ptr cloud_plane (new PointCloudT);
PointCloudT::Ptr cloud_blobs (new PointCloudT);
PointCloudT::Ptr empty_cloud (new PointCloudT);
std::vector<PointCloudT::Ptr > clusters_on_plane;

sensor_msgs::PointCloud2 cloud_ros;

ros::Publisher cloud_pub;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

double plane_distance_tolerance = 0.04;

// Check if a file exist or not
bool file_exist(std::string& name) {
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0);
}

/* what happens when ctr-c is pressed */
void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};


void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{


	cloud_mutex.lock ();

	//convert to PCL format
	pcl::fromROSMsg (*input, *cloud);

	//state that a new cloud is available
	new_cloud_available_flag = true;

	cloud_mutex.unlock ();
}


bool filter(PointCloudT::Ptr blob, Eigen::Vector4f plane_coefficients, double tolerance){
	
	double min_distance = 1000.0;
	double max_distance = -1000.0;
	
	//first, we find the point in the blob closest to the plane
	for (unsigned int i = 0; i < blob->points.size(); i++){
		pcl::PointXYZ p_i;
		p_i.x=blob->points.at(i).x;
		p_i.y=blob->points.at(i).y;
		p_i.z=blob->points.at(i).z;

		double distance = pcl::pointToPlaneDistance(p_i, plane_coefficients);
		
		if (distance < min_distance){
			min_distance = distance;
		}
		
		if (distance > max_distance)
			max_distance = distance;
		
		
		
	}
	
	
	if (min_distance > tolerance)
		return false;
	else if (max_distance < 0.8*tolerance)
		return false;	
	
	
	ROS_INFO("\nMin Distance to plane for cluster with %i points: %f",(int)blob->points.size(),min_distance);
	ROS_INFO("Max Distance to plane for cluster with %i points: %f",(int)blob->points.size(),max_distance);

	
	return true;
	
}

double computeAvgRedValue(PointCloudT::Ptr in){
	double total_red = 0;

	for (unsigned int i = 0; i < in->points.size(); i++){
		total_red += in->points.at(i).r;

	}

	total_red /= (in->points.size());
	return total_red;
}

std::vector<PointCloudT::Ptr > computeClusters(PointCloudT::Ptr in, double tolerance){
	std::vector<PointCloudT::Ptr > clusters;
	
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (in);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (tolerance); // 2cm
	ec.setMinClusterSize (50);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (in);
	ec.extract (cluster_indices);

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		PointCloudT::Ptr cloud_cluster (new PointCloudT);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (in->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		clusters.push_back(cloud_cluster);
	}
	return clusters;
}

void waitForCloud(){
	ros::Rate r(30);
	
	while (ros::ok()){
		ros::spinOnce();
		
		r.sleep();
		
		if (new_cloud_available_flag){
			new_cloud_available_flag = false;
			break;
		}
	}
	
}

/* collects a cloud by aggregating k successive frames */
void waitForCloudK(int k){
	ros::Rate r(30);
	
	cloud_aggregated->clear();
	
	int counter = 0;
	
	while (ros::ok()){
		ros::spinOnce();
		
		r.sleep();
		
		if (new_cloud_available_flag){
			
			*cloud_aggregated+=*cloud;
			
			new_cloud_available_flag = false;
			
			counter ++;
			
			if (counter >= k){
				cloud_aggregated->header = cloud->header;
				break;
			}
		}
	}
	
}

bool seg_cb(bwi_perception::ButtonDetection::Request &req, bwi_perception::ButtonDetection::Response &res)
{
	//get the point cloud by aggregating k successive input clouds
	waitForCloudK(15);
	cloud = cloud_aggregated;

	//**Step 1: z-filter and voxel filter**//
	
	// Create the filtering object
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.0, 1.15);
	pass.filter (*cloud);
	
	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<PointT> vg;
	pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
	vg.setInputCloud (cloud);
	vg.setLeafSize (0.0025f, 0.0025f, 0.0025f);
	vg.filter (*cloud_filtered);

	//publish point cloud for debugging
	ROS_INFO("Publishing point cloud...");
	/*pcl::toROSMsg(*cloud_filtered,cloud_ros);
	cloud_ros.header.frame_id = cloud->header.frame_id;
	cloud_pub.publish(cloud_ros);*/

    ROS_INFO("After voxel grid filter: %i points",(int)cloud_filtered->points.size());
    
    //**Step 2: plane fitting**//
    
    //find palne
    //one cloud contains plane other cloud contains other objects
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
	pcl::SACSegmentation<PointT> seg;
	// Optional
    seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (1000);
	seg.setDistanceThreshold (0.02);

	// Create the filtering object
	pcl::ExtractIndices<PointT> extract;

	// Segment the largest planar component from the remaining cloud
	seg.setInputCloud (cloud_filtered);
	seg.segment (*inliers, *coefficients);

	// Extract the plane
	extract.setInputCloud (cloud_filtered);
	extract.setIndices (inliers);
	extract.setNegative (false);
	extract.filter (*cloud_plane);
     
    //for everything else, cluster extraction; segment extraction
    //extract everything else
	extract.setNegative (true);
	extract.filter (*cloud_blobs);
	
    //get the plane coefficients
	Eigen::Vector4f plane_coefficients;
	plane_coefficients(0)=coefficients->values[0];
	plane_coefficients(1)=coefficients->values[1];
	plane_coefficients(2)=coefficients->values[2];
	plane_coefficients(3)=coefficients->values[3];

    	
    //**Step 3: Eucledian Cluster Extraction**//
	std::vector<PointCloudT::Ptr > clusters = computeClusters(cloud_blobs,0.04);
	
	ROS_INFO("clustes found: %i", (int)clusters.size());
	
	clusters_on_plane.clear();
	
	//if clusters are touching the table put them in a vector
	for (unsigned int i = 0; i < clusters.size(); i++){
		Eigen::Vector4f centroid_i;
		pcl::compute3DCentroid(*clusters.at(i), centroid_i);
		pcl::PointXYZ center;
		center.x=centroid_i(0);center.y=centroid_i(1);center.z=centroid_i(2);

		double distance = pcl::pointToPlaneDistance(center, plane_coefficients);
		if (distance < 0.1 /*&& clusters.at(i).get()->points.size() >*/ ){
			clusters_on_plane.push_back(clusters.at(i));
		}
	}
	ROS_INFO("clustes_on_plane found: %i", (int)clusters_on_plane.size());
	
	// if the clousters size == 0 return false
	if(clusters_on_plane.size() == 0) {
		
		cloud_mutex.unlock ();

		res.button_found = false; 
		return true;
	}
	//**Step 4: detect the button among the remaining clusters**//
	int max_index = -1;

	double max_red = 0.0;
	// Find the max red value
	for (unsigned int i = 0; i < clusters_on_plane.size(); i++){
		double red_i = computeAvgRedValue(clusters_on_plane.at(i));
		//ROS_INFO("Cluster %i: %i points, red_value = %f",i,(int)clusters_on_plane.at(i)->points.size(),red_i);

		if (red_i > max_red){
			max_red = red_i;
			max_index = i;
		}
	}
	
	//publish  cloud if we think it's a button
	/*max_red > 170 && max_red < 250 && */
	ROS_INFO("max_red=%f", max_red);

	
	if (max_index >= 0 && max_red > red_min) {
			
	    ROS_INFO("Button_found");

		pcl::toROSMsg(*clusters_on_plane.at(max_index),cloud_ros);
		cloud_ros.header.frame_id = cloud->header.frame_id;
		cloud_pub.publish(cloud_ros);

		//fill in response
	    res.button_found = true;
	    res.cloud_button = cloud_ros;

		/*Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*clusters_on_plane.at(max_index), centroid);

		//transforms the pose into /map frame
		geometry_msgs::Pose pose_i;
		pose_i.position.x=centroid(0);
		pose_i.position.y=centroid(1);
		pose_i.position.z=centroid(2);
		pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-3.14/2);

		geometry_msgs::PoseStamped stampedPose;

		stampedPose.header.frame_id = cloud->header.frame_id;
		stampedPose.header.stamp = ros::Time(0);
		stampedPose.pose = pose_i;*/

		//geometry_msgs::PoseStamped stampOut;
		//listener.waitForTransform(cloud->header.frame_id, "m1n6s200_link_base", ros::Time(0), ros::Duration(3.0));
		//listener.transformPose("m1n6s200_link_base", stampedPose, stampOut);

		//stampOut.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,-3.14/2,0);
		//pose_pub.publish(stampOut);

	}
	
	else {
		res.button_found = false;
		
	}
	
	cloud_mutex.unlock ();

	return true;
}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "segbot_arm_button_detector");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	std::string param_topic = "/xtion_camera/depth_registered/points"; 
	ros::Subscriber sub = nh.subscribe (param_topic, 1, cloud_cb);

	//debugging publisher
	cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("segbot_arm_button_detector/cloud", 10);

	//service
	ros::ServiceServer service = nh.advertiseService("segbot_arm_button_detector/detect", seg_cb); 
	
	
	tf::TransformListener listener;

	//register ctrl-c
	signal(SIGINT, sig_handler);

	//refresh rate
	double ros_rate = 10.0;
	ros::Rate r(ros_rate);

	// Main loop:
	while (!g_caught_sigint && ros::ok())
	{
		//collect messages
		ros::spinOnce();

		r.sleep();
	}
};
