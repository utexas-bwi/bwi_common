#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cstdio>
#include <iostream>
#include <sstream>
#include <signal.h>
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/image_encodings.h>
#include "bwi_perception/ProcessVision.h"
#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
  #include <opencv/cv.h>
#elif CV_MAJOR_VERSION == 3
  #include <opencv2/imgcodecs.hpp>      // for cv::imwrite()
#endif
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <boost/lexical_cast.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/compression/octree_pointcloud_compression.h>

using namespace std;
using namespace cv;

bool g_caught_sigint = false;

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// General point cloud to store the whole image
PointCloudT::Ptr image_cloud (new PointCloudT);
// output pointcloud
PointCloudT::Ptr image_cloud_compressed (new PointCloudT);

// For point-cloud compression purposes
pcl::io::OctreePointCloudCompression<PointT>* PointCloudEncoder;
pcl::io::OctreePointCloudCompression<PointT>* PointCloudDecoder;
pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
		
//z-filter
pcl::PassThrough<PointT> pass;

//the current image pointer
cv_bridge::CvImagePtr cv_image;

// should start recording or not				
bool recording_samples;
string generalImageFileName;
string generalDepthImageName;    

int image_count = 0;
int pcd_count = 0;

// function to handle Ctrl-C
void sig_handler(int sig)
{
	g_caught_sigint = true;
	exit (0);
};

//callback funtion to store depth images
void collect_vision_depth_data(const sensor_msgs::PointCloud2ConstPtr& msg){
	if(recording_samples == true){
		 if ((msg->width * msg->height) == 0)
			return;
		
		//convert the msg to PCL format
		pcl::fromROSMsg (*msg, *image_cloud);
		
		//get the start time of recording
		double begin = ros::Time::now().toSec();
		string startTime = boost::lexical_cast<std::string>(begin);
		
		// append start timestamp with filenames
		std::stringstream convert;
		convert << pcd_count;
		std::string filename = generalDepthImageName+convert.str()+"_"+startTime+".pcd";
		
		//Before saving, do a z-filter	
		pass.setInputCloud (image_cloud);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (0.0, 2.15);
		pass.filter (*image_cloud);
		
		// stringstream to store compressed point cloud
		std::stringstream compressedData;
		// instantiate point cloud compression for encoding and decoding
		PointCloudEncoder = new pcl::io::OctreePointCloudCompression<PointT> (compressionProfile, true);
		PointCloudDecoder = new pcl::io::OctreePointCloudCompression<PointT> ();
    
		//ROS_INFO("Starting compression");
		
		// compress point cloud
		PointCloudEncoder->encodePointCloud (image_cloud, compressedData);
		// decompress point cloud
		PointCloudDecoder->decodePointCloud (compressedData, image_cloud_compressed);	
		
		//ROS_INFO("Saving compressed cloud to file");
		
		//Save the cloud to a .pcd file
		pcl::io::savePCDFileASCII(filename, *image_cloud);
		ROS_INFO("Saved pcd file %s", filename.c_str());
		
		// delete point cloud compression instances
		delete (PointCloudEncoder);
		delete (PointCloudDecoder);
		
		pcd_count++;
	}
}

//callback funtion to store rgb images
void collect_vision_rgb_data(const sensor_msgs::ImageConstPtr& msg){
	if(recording_samples == true){
		
		try{	
			cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		
		if (cv_image) {
			//get the start time of recording
			double begin = ros::Time::now().toSec();
			string startTime = boost::lexical_cast<std::string>(begin);
		
			// append start timestamp with filenames
			std::stringstream convert;
			convert << image_count;
			std::string filename = generalImageFileName+"/test"+convert.str()+"_"+startTime+".jpg";
			//std::string filename = "./test"+convert.str()+"_"+startTime+".jpg";

			imwrite(filename.c_str(), cv_image->image);
			ROS_INFO("Saved image %s", filename.c_str());
					
			image_count++;
		}
	}
}

//callback funtion, if recording samples is true then make a call to store the visual feed 
bool vision_service_callback(bwi_perception::ProcessVision::Request &req,
							bwi_perception::ProcessVision::Response &res){
	if (req.start == 1){
		//start recording
		recording_samples = true;
		
		//also store the filenames that are in the request
		generalImageFileName = req.generalImageFilePath;
		generalDepthImageName = req.generalDepthImagePath;       
	}
	else{
		//set a flag to stop recording
		recording_samples = false;
		image_count = 0;
		pcd_count = 0;
	}
	
	res.success = true;
	return true;
};
							
int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "image_logging_server");
	ros::NodeHandle nh;
	
	//to store the topic to subscribe to
	string rgb_topic_;
	
	//Set up the service
	ros::ServiceServer service = nh.advertiseService("image_logger_service", vision_service_callback);
	
	//subscribe to the vision depth topic
	ros::Subscriber sub_depth = nh.subscribe ("/xtion_camera/depth_registered/points", 1, collect_vision_depth_data);

	//get the topic from the launch file
	nh.param<std::string>("rgb_topic", rgb_topic_, "/xtion_camera/rgb/image_rect_color");
	
	//subsribe to the vision rgb topic
	ros::Subscriber sub_rgb = nh.subscribe (rgb_topic_, 1, collect_vision_rgb_data);
	
	//register ctrl-c
	signal(SIGINT, sig_handler);
	
	ROS_INFO("Ready to record and process vision data.");
	ros::spin();
	
	return 0;
}
