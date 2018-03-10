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

#include <pcl/octree/octree.h>

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

#include <std_srvs/Empty.h>

#define OCTREE_RESOLUTION 0.01

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
//the change cloud
PointCloudT::Ptr filtered_cloud;


sensor_msgs::PointCloud2 cloud_ros;

boost::shared_ptr<pcl::octree::OctreePointCloudChangeDetector<PointT>> octree;


ros::Publisher cloud_pub;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

//true if change is being computed
bool computeChange = false;
bool firstRun = true;

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
	if (computeChange){

		//cloud_mutex.lock ();

		//convert to PCL format
		pcl::fromROSMsg (*input, *cloud);

		//state that a new cloud is available
		new_cloud_available_flag = true;
		
		// assign point cloud to octree
		octree->setInputCloud (cloud);

		// add points from cloud to octree
        octree->addPointsFromInputCloud();


        std::vector<int> newPointIdxVector;

        // get a vector of new points, which did not exist in previous buffer

        octree->getPointIndicesFromNewVoxels(newPointIdxVector, 7); //the last argument is "noise_filter"

		

		filtered_cloud.reset (new PointCloudT);

        filtered_cloud->points.reserve(newPointIdxVector.size());
        for (int &it : newPointIdxVector)
            filtered_cloud->points.push_back(cloud->points[it]);

		//publish change only after 1st frame
		if (!firstRun){
			pcl::toROSMsg(*filtered_cloud,cloud_ros);
			cloud_ros.header.frame_id = cloud->header.frame_id;
			cloud_pub.publish(cloud_ros);
		}
		else firstRun = false;
				
		// switch buffers - reset tree
		octree->switchBuffers ();

		//cloud_mutex.unlock ();
	}
}



bool start_service_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    octree = boost::shared_ptr<pcl::octree::OctreePointCloudChangeDetector<PointT>>(
            new pcl::octree::OctreePointCloudChangeDetector<PointT>(OCTREE_RESOLUTION));
	
	computeChange = true;
	firstRun = true;
	
	return true;
}



bool stop_service_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	computeChange = false;
    octree = boost::shared_ptr<pcl::octree::OctreePointCloudChangeDetector<PointT>>();
	
	return true;
}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "segbot_arm_table_change_detector");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	std::string param_topic = "/xtion_camera/depth_registered/points";
	ros::Subscriber sub = nh.subscribe (param_topic, 1, cloud_cb);

	//debugging publisher
	cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("segbot_arm_table_change_detector/cloud", 10);

	//service
	ros::ServiceServer service_start = nh.advertiseService("segbot_arm_table_change_detector/start", start_service_cb);
	ros::ServiceServer service_stop = nh.advertiseService("segbot_arm_table_change_detector/stop", stop_service_cb);
	
	//register ctrl-c
	signal(SIGINT, sig_handler);

	//create octree structure
    octree = boost::shared_ptr<pcl::octree::OctreePointCloudChangeDetector<PointT>>(
            new pcl::octree::OctreePointCloudChangeDetector<PointT>(OCTREE_RESOLUTION));

	ros::spin();

};
