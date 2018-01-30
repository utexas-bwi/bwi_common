#include <signal.h>
#include <vector>
#include <Eigen/Dense>
#include <string>
#include <sys/stat.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3Stamped.h>

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
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/kdtree/kdtree.h>

#include "bwi_perception/TabletopPerception.h"
#include "bwi_perception/TabletopReorder.h"
#include "bwi_perception/GetCloud.h"
#include "bwi_perception/GetPCD.h"

//how many frames to stitch into a single cloud
#define WAIT_CLOUD_K 25

//default Z filter
#define Z_FILTER_DEFAULT 1.15

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// Mutex: //
boost::mutex cloud_mutex;

bool new_cloud_available_flag = false;
PointCloudT::Ptr cloud (new PointCloudT);
PointCloudT::Ptr cloud_aggregated (new PointCloudT);
PointCloudT::Ptr cloud_plane (new PointCloudT);
PointCloudT::Ptr cloud_blobs (new PointCloudT);
PointCloudT::Ptr empty_cloud (new PointCloudT);
std::vector<PointCloudT::Ptr > clusters;
std::vector<PointCloudT::Ptr > clusters_on_plane;

sensor_msgs::PointCloud2 cloud_ros;

ros::Publisher cloud_pub;
ros::Publisher table_cloud_pub;

ros::Publisher cloud_costmap_pub;
PointCloudT::Ptr cloud_costmap (new PointCloudT);

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

//an object whose closest point to the plane is further than this is rejected
double plane_distance_tolerance = 0.09;

//an object whose furthers point to the plane is smaller than this is rejected
double plane_max_distance_tolerance = 0.02;

//how close to points outside the plane must go into the same object cluster
double cluster_extraction_tolerance = 0.075;

bool collecting_cloud = false;


//epsilon angle for segmenting, value in radians
#define EPS_ANGLE 0.09 

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
		/*if (collecting_cloud){
			ROS_INFO("Cloud received while collecting.");
		}*/

		//convert to PCL format
		pcl::fromROSMsg (*input, *cloud);
		
		//state that a new cloud is available
		new_cloud_available_flag = true;
}

bool get_pcd_cb(bwi_perception::GetPCD::Request &req, bwi_perception::GetPCD::Response &res){
	
	//get the start time of recording
	double begin = ros::Time::now().toSec();
	std::string startTime = boost::lexical_cast<std::string>(begin);
		
	//save file
	std::string filename = req.generalImageFilePath +"/"+ startTime+".pcd";
	pcl::io::savePCDFileASCII(filename, *cloud);
	ROS_INFO("Saved pcd file %s", filename.c_str());
	
	res.success = true;
	return true;
}


bool filter(PointCloudT::Ptr blob, PointCloudT::Ptr plane_cloud, Eigen::Vector4f plane_coefficients, double tolerance_min, double tolerance_max){
	
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
	
	
	if (min_distance > tolerance_min)
		return false;
	else if (max_distance < tolerance_max)
		return false;	
	
	
	ROS_INFO("\nMin Distance to plane for cluster with %i points: %f",(int)blob->points.size(),min_distance);
	ROS_INFO("Max Distance to plane for cluster with %i points: %f",(int)blob->points.size(),max_distance);

	
	return true;
	
}

/*Function for finding the largest plane from the segmented "table"
 * removes noise*/
PointCloudT::Ptr seg_largest_plane(PointCloudT::Ptr in, double tolerance){
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (in);
	ROS_INFO("point cloud size of 'plane cloud' : %ld", in->size());

	//use euclidean cluster extraction to eliminate noise and get largest plane
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (tolerance); 
	ec.setMinClusterSize (200);
	ec.setSearchMethod (tree);
	ec.setInputCloud (in);
	ec.extract (cluster_indices);

	std::vector<PointCloudT::Ptr> clusters_vec;

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		PointCloudT::Ptr cloud_cluster (new PointCloudT);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (in->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		
		clusters_vec.push_back(cloud_cluster);
  }
    
    //find the largest point cloud 
	int largest_pc_index = -1;
	int largest_num_points = -1;
	
	for (unsigned int i = 0; i < clusters_vec.size(); i++){
		int num_points_i = clusters_vec[i]->height* clusters_vec[i]->width;
		if (num_points_i > largest_num_points){
			largest_num_points = num_points_i;
			largest_pc_index = i;
		}
	}
	
	ROS_INFO("number of 'plane clouds' : %ld", clusters_vec.size());
	
	if(largest_pc_index == -1){
		//did not find any objects, an error has occurred
		ROS_ERROR("There is no table. Should not reach this point");
		exit(1); 
	}
	return clusters_vec[largest_pc_index];
} 

/*Function to compute clusters on the table*/
void computeClusters(PointCloudT::Ptr in, double tolerance){
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (in);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (tolerance); 
	ec.setMinClusterSize (200);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (in);
	ec.extract (cluster_indices);

	clusters.clear();

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		PointCloudT::Ptr cloud_cluster (new PointCloudT);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (in->points[*pit]); 
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		clusters.push_back(cloud_cluster);
  }
}

void waitForCloud(){
	ros::Rate r(30);
	
	collecting_cloud = true;
	
	while (ros::ok()){
		ros::spinOnce();
		
		r.sleep();
		
		if (new_cloud_available_flag){
			new_cloud_available_flag = false;
			break;
		}
	}
	
	collecting_cloud = false;
}

/* collects a cloud by aggregating k successive frames */
void waitForCloudK(int k){
	ros::Rate r(30);
	
	cloud_aggregated->clear();
	
	int counter = 0;
	collecting_cloud = true;
	while (ros::ok()){
		ros::spinOnce();
		
		r.sleep();
		
		if (new_cloud_available_flag){
			
			*cloud_aggregated+=*cloud;
			
			new_cloud_available_flag = false;
			
			counter ++;
			
			ROS_INFO("Adding cloud %i",counter);
			
			if (counter >= k){
				cloud_aggregated->header = cloud->header;
				break;
			}
		}
	}
	collecting_cloud = false;
}

// re-orders detected, non-overlapping clusters according to given coordinate and direction
bool cluster_reorder_cb_helper(float i, float j) { return (i>j); }
bool cluster_reorder_cb(bwi_perception::TabletopReorder::Request &req, bwi_perception::TabletopReorder::Response &res)
{
	// get representative coordinates for each cluster
	std::vector<float> representative_coords;
	for (unsigned int i = 0; i < clusters_on_plane.size(); i++){
		PointCloudT::Ptr c = clusters_on_plane.at(i);
		PointT p = c->points.at(0);
		if (req.coord.compare("x") == 0)
			representative_coords.push_back(p.x);
		else if (req.coord.compare("y") == 0)
			representative_coords.push_back(p.y);
		else if (req.coord.compare("z") == 0)
			representative_coords.push_back(p.z);
		else{
			ROS_INFO("Unrecognized coord arg: %s", req.coord.c_str());
			return false;
		}
	}

	// determine new order based on specified direction
	std::vector<int> idx_order;
	std::vector<float> coords_ordered(representative_coords);
	if (req.forward)
		std::sort(coords_ordered.begin(), coords_ordered.end());
	else
		std::sort(coords_ordered.begin(), coords_ordered.end(), cluster_reorder_cb_helper);
	for (unsigned int i = 0; i < coords_ordered.size(); i++){
		idx_order.push_back(std::distance(representative_coords.begin(), std::find(representative_coords.begin(), representative_coords.end(), coords_ordered.at(i))));
	}

	// iterate through clusters on plane in determined order
	for (unsigned int i = 0; i < clusters_on_plane.size(); i++){
		pcl::toROSMsg(*clusters_on_plane.at(idx_order.at(i)), cloud_ros);
		cloud_ros.header.frame_id = cloud->header.frame_id;
		res.ordered_cloud_clusters.push_back(cloud_ros);
	}

	return true;
}

bool seg_cb(bwi_perception::TabletopPerception::Request &req, bwi_perception::TabletopPerception::Response &res)
{
	ROS_INFO("Request received...starting pipeline.");
	
	//create listener for transforms
	tf::TransformListener tf_listener;
	
	//get the point cloud by aggregating k successive input clouds
	new_cloud_available_flag = false;
	ROS_INFO("waiting for cloud...");
	waitForCloudK(WAIT_CLOUD_K);
	cloud = cloud_aggregated;
	
	ROS_INFO("collected cloud success");

	double filter_z = Z_FILTER_DEFAULT;
	if (req.override_filter_z){
		filter_z = req.filter_z_value;
	}

	// Apply z filter -- we don't care for anything X m away in the z direction
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.0, filter_z);
	pass.filter (*cloud);
	
	// Apply x filter 
	if (req.apply_x_box_filter){
		pass.setInputCloud (cloud);
		pass.setFilterFieldName ("x");
		pass.setFilterLimits (req.x_min, req.x_max);
		pass.filter (*cloud);
	}
	
	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<PointT> vg;
	pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
	vg.setInputCloud (cloud);
	vg.setLeafSize (0.0025f, 0.0025f, 0.0025f);
	vg.filter (*cloud_filtered);
	
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	// Create the segmentation object
	pcl::SACSegmentation<PointT> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	//look for a plane perpendicular to a given axis
	seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (1000);
	seg.setDistanceThreshold (0.025);
	
	
	//create the axis to use
	geometry_msgs::Vector3Stamped ros_vec;
	ros_vec.header.frame_id = "/base_link";
	ros_vec.vector.x = 0.0;
	ros_vec.vector.y = 0.0;
	ros_vec.vector.z = 1.0;
	
	ROS_INFO("Ros axis: %f, %f, %f",
		ros_vec.vector.x,ros_vec.vector.y,ros_vec.vector.z);
	
	//transform the vector to the camera frame of reference 
	tf_listener.transformVector(cloud->header.frame_id, ros::Time(0), ros_vec, "/base_link", ros_vec); 

	//set the axis to the transformed vector
	Eigen::Vector3f axis = Eigen::Vector3f(ros_vec.vector.x, ros_vec.vector.y , ros_vec.vector.z);
	seg.setAxis(axis);
	
	ROS_INFO("sac axis value: %f, %f, %f", seg.getAxis()[0],seg.getAxis()[1], seg.getAxis()[2]); 

	//set an epsilon that the table can differ from the axis above by
  	seg.setEpsAngle(EPS_ANGLE); //value in radians, corresponds to approximately 5 degrees
	
	ROS_INFO("epsilon value: %f", seg.getEpsAngle()); 	
	
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

	//downsample the plane cloud and segment out noise
	vg.setInputCloud (cloud_plane);
	vg.setLeafSize (0.005f, 0.005f, 0.005f);
	vg.filter (*cloud_plane);
	
	//find the largest plane and segment out noise
	cloud_plane = seg_largest_plane(cloud_plane, cluster_extraction_tolerance);
	
	//make sure the cloud plane exists still
	res.is_plane_found = true;
	if(cloud_plane->empty()){
		res.is_plane_found = false;
		return true;
	}
		
	//extract everything else
	extract.setNegative (true);
	extract.filter (*cloud_blobs);

	//get the plane coefficients
	Eigen::Vector4f plane_coefficients;
	plane_coefficients(0)=coefficients->values[0];
	plane_coefficients(1)=coefficients->values[1];
	plane_coefficients(2)=coefficients->values[2];
	plane_coefficients(3)=coefficients->values[3];
	
	ROS_INFO("Planar coefficients: %f, %f, %f, %f",
		plane_coefficients(0),plane_coefficients(1),plane_coefficients(2),	plane_coefficients(3));
	
	//debug plane cloud 
	//convert plane cloud to ROS
	sensor_msgs::PointCloud2 ros_plane;
	
	//pcl::toROSMsg(*cloud_plane, ros_plane);
	pcl::toROSMsg(*cloud_plane, ros_plane);
	
	ros_plane.header.frame_id = cloud->header.frame_id;
	table_cloud_pub.publish(ros_plane);
	
	//Step 3: Eucledian Cluster Extraction
	computeClusters(cloud_blobs,cluster_extraction_tolerance);
	
	ROS_INFO("Found %i clusters.",(int)clusters.size());

	clusters_on_plane.clear();
	
	//if true, clouds on the other side of the plane will be rejected
	bool check_below_plane = true; 
	double plane_z = -1.0;
	PointCloudT::Ptr cloud_plane_baselink (new PointCloudT);
	Eigen::Vector4f plane_centroid;
   
	if (check_below_plane){
		
		//wait for transform and perform it
		tf_listener.waitForTransform(cloud->header.frame_id,"/base_link",ros::Time(0), ros::Duration(3.0)); 
		
		//convert plane cloud to ROS
		sensor_msgs::PointCloud2 plane_cloud_ros;
		pcl::toROSMsg(*cloud_plane,plane_cloud_ros);
		plane_cloud_ros.header.frame_id = cloud->header.frame_id;
		
		//transform it to base link frame of reference
		pcl_ros::transformPointCloud ("/base_link", plane_cloud_ros, plane_cloud_ros, tf_listener);
					
		//convert to PCL format and take centroid
		pcl::fromROSMsg (plane_cloud_ros, *cloud_plane_baselink);
		pcl::compute3DCentroid (*cloud_plane_baselink, plane_centroid);
		
		ROS_INFO("[table_object_detection_node.cpp] Plane xyz: %f, %f, %f",plane_centroid(0),plane_centroid(1),plane_centroid(2));
	}

	for (unsigned int i = 0; i < clusters.size(); i++){
		
		if (filter(clusters.at(i),cloud_plane,plane_coefficients,plane_distance_tolerance,plane_max_distance_tolerance)){
			
			//next check which clusters are below and which are aboe the plane
			if (check_below_plane){
				sensor_msgs::PointCloud2 cloud_i_ros;
				pcl::toROSMsg(*clusters.at(i),cloud_i_ros);
				cloud_i_ros.header.frame_id = cloud->header.frame_id;
				
				pcl_ros::transformPointCloud ("/base_link", cloud_i_ros,cloud_i_ros, tf_listener);
				
				//convert to PCL format and take centroid
				PointCloudT::Ptr cluster_i_baselink (new PointCloudT);
				pcl::fromROSMsg (cloud_i_ros, *cluster_i_baselink);
				
				Eigen::Vector4f centroid_i;
				pcl::compute3DCentroid (*cluster_i_baselink, centroid_i);
				
				ROS_INFO("[table_object_detection_node.cpp] cluster %i xyz: %f, %f, %f",i,centroid_i(0),centroid_i(1),centroid_i(2));
				
				//check z's
				if (centroid_i(2) < plane_centroid(2)){
					//below the table (maybe the edge of the table, etc)
					ROS_INFO("[table_object_detection_node.cpp] Rejected as below the table...");
				}
				else {
					//above the table
					clusters_on_plane.push_back(clusters.at(i));
				}
				
				//TO DO: check if cloud_i_ros has larger z than plane_cloud_ros
			}
			else {
				clusters_on_plane.push_back(clusters.at(i));
			}
		}
	}
	
	ROS_INFO("Found %i clusters on the plane.",(int)clusters_on_plane.size());
	
	//fill in responses
	//plane cloud and coefficient
	pcl::toROSMsg(*cloud_plane,res.cloud_plane);
	res.cloud_plane.header.frame_id = cloud->header.frame_id;
	for (int i = 0; i < 4; i ++){
		res.cloud_plane_coef[i] = plane_coefficients(i);
	}
	
	//blobs on the plane
	for (unsigned int i = 0; i < clusters_on_plane.size(); i++){
		pcl::toROSMsg(*clusters_on_plane.at(i),cloud_ros);
		cloud_ros.header.frame_id = cloud->header.frame_id;
		res.cloud_clusters.push_back(cloud_ros);
	}
	
	//for debugging purposes
	//now, put the clouds in cluster_on_plane in one cloud and publish it
	cloud_blobs->clear();

	for (unsigned int i = 0; i < clusters_on_plane.size(); i++){
		*cloud_blobs += *clusters_on_plane.at(i);
	}
	
	ROS_INFO("Publishing debug cloud...");
	pcl::toROSMsg(*cloud_blobs,cloud_ros);
	cloud_ros.header.frame_id = cloud->header.frame_id;
	cloud_pub.publish(cloud_ros);
	
	pcl::toROSMsg(*cloud_plane,cloud_ros);
	cloud_ros.header.frame_id = cloud->header.frame_id;
	table_cloud_pub.publish(cloud_ros);
	

	
	return true;
}


bool get_cloud_cb(bwi_perception::GetCloud::Request &req, bwi_perception::GetCloud::Response &res){
	
	ROS_INFO("[table_object_detection_node.cpp] retrieving point cloud...");
	waitForCloudK(15);
	cloud = cloud_aggregated;
	
	pcl::toROSMsg(*cloud,res.cloud);
	res.cloud.header.frame_id = cloud->header.frame_id;
	
	ROS_INFO("[table_object_detection_node.cpp] ...done");
	
	
	return true;
}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "segbot_arm_table_object_detector");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	std::string param_topic = "/xtion_camera/depth_registered/points";
	ros::Subscriber sub = nh.subscribe (param_topic, 100, cloud_cb);

	//debugging publisher
	cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("table_object_detection_node/cloud", 1);
	table_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("table_object_detection_node/plane_cloud", 1);

	//publisher for cost map cloud
	cloud_costmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/xtion_obstacle_cloud", 1);

	//services
	ros::ServiceServer service = nh.advertiseService("tabletop_object_detection_service", seg_cb);
	
	ros::ServiceServer reorder_service = nh.advertiseService("tabletop_object_reorder_service", cluster_reorder_cb);
	
	ros::ServiceServer getcloud_service = nh.advertiseService("tabletop_get_cloud_service", get_cloud_cb);
	
	ros::ServiceServer get_pcd_service = nh.advertiseService("tabletop_get_pcd_service", get_pcd_cb);

	//register ctrl-c
	signal(SIGINT, sig_handler);

	//refresh rate
	double ros_rate = 3.0;
	ros::Rate r(ros_rate);

	// Main loop:
	while (!g_caught_sigint && ros::ok())
	{
		//collect messages
		ros::spinOnce();
		r.sleep();
	}
};
