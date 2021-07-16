#include <signal.h>
#include <vector>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <pcl_ros/impl/transforms.hpp>

// PCL specific includes

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/console/parse.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>

#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/vfh.h>
#include <pcl/features/cvfh.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>


#include <bwi_perception/DetectHorizontalPlanes.h>


#include <bwi_perception/BoundingBox.h>
#include <visualization_msgs/MarkerArray.h>
#include <bwi_perception/bwi_perception.h>
#include <bwi_perception/convenience.h>
#include <bwi_perception/pcl.h>
#include <bwi_perception/plane.h>


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

#define VISUALIZE true
#define DEBUG_ENTER false // if true, you have to press enter to continue the process

/* define what kind of point clouds we're using */
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;

//service for horizontal_plane segmenting
ros::ServiceServer horizontal_plane_serv;
ros::Publisher voxel_cloud_pub;
ros::Publisher horizontal_plane_marker_pub;
ros::Publisher current_plane_cloud_pub;
ros::Publisher remaining_cloud_pub;
//create listener for transforms
tf::TransformListener *tf_listener;


void pressEnter(string message) {
    std::cout << message;
    while (true) {
        char c = std::cin.get();
        if (c == '\n')
            break;
        else if (c == 'q') {
            ros::shutdown();
            exit(1);
        } else {
            std::cout << message;
        }
    }
}


bool find_horizontal_planes(bwi_perception::DetectHorizontalPlanes::Request &req,
                            bwi_perception::DetectHorizontalPlanes::Response &res) {

    PointCloudT::Ptr cloud(new PointCloudT);
    //convert to PCL format
    pcl::fromROSMsg(req.cloud_input, *cloud);
    cloud->header.frame_id = req.cloud_input.header.frame_id;


    if (cloud->points.empty()) {
        ROS_ERROR("The input cloud is empty. Cannot process service");
        return false;
    }

    // Convert cloud to map frame:

    // Define Frame Parameters
    string z_axis_reference_frame(Z_AXIS_REFERENCE_FRAME);
    string target_frame(TARGET_FRAME);

    PointCloudT::Ptr map_cloud(new PointCloudT);
    bwi_perception::cloud_to_frame<PointT>(cloud, TARGET_FRAME, map_cloud, *tf_listener);

    // If the floor is to be ignored, use the filtered map instead
    if (IGNORE_FLOOR) {

        // Create filtered point cloud
        PointCloudT::Ptr map_cloud_pass_through(new PointCloudT);

        // Create the filtering object
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud(map_cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(MIN_Z, MAX_Z);
        //pass.setFilterLimitsNegative (true);
        pass.filter(*map_cloud_pass_through);
        map_cloud = map_cloud_pass_through;
    }


    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    PointCloudT::Ptr cloud_filtered(new PointCloudT);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(map_cloud);
    // TODO: Do we need as much resolution in XY as Z?
    vg.setLeafSize(VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE);
    vg.filter(*cloud_filtered);

    vector<PointCloudT::Ptr> plane_clouds;
    vector<float> plane_coefficients;
    bwi_perception::extract_horizontal_planes<PointT>(cloud_filtered, plane_clouds, plane_coefficients,
                                                      Z_AXIS_REFERENCE_FRAME, *tf_listener);
    if (VISUALIZE) {
        //debugging publishing
        sensor_msgs::PointCloud2 ros_cloud;
        pcl::toROSMsg(*cloud_filtered, ros_cloud);
        ros_cloud.header.frame_id = map_cloud->header.frame_id;
        ROS_INFO("Visualizing pointcloud to extract horizontal planes from...");
        voxel_cloud_pub.publish(ros_cloud);
    }
    if (DEBUG_ENTER) {
        pressEnter("    Press ENTER to continue");
    }

    transform(plane_clouds.begin(), plane_clouds.end(), back_inserter(res.horizontal_planes),
              [](PointCloudT::Ptr pcl_cloud) {
                  sensor_msgs::PointCloud2 ros_cloud;
                  pcl::toROSMsg(*pcl_cloud, ros_cloud);
                  return ros_cloud;
              });
    res.horizontal_plane_coefs.insert(res.horizontal_plane_coefs.end(), plane_coefficients.begin(),
                                      plane_coefficients.end());

    return true;
}


int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "horizontal_plane_detector_server");
    ros::NodeHandle nh;
    tf_listener = new tf::TransformListener(nh);

    horizontal_plane_serv = nh.advertiseService("detect_horizontal_planes", find_horizontal_planes);

    if (VISUALIZE) {
        ros::NodeHandle pnh("~");
        voxel_cloud_pub = pnh.advertise<sensor_msgs::PointCloud2>("downsampled_input_cloud", 1, true);
        current_plane_cloud_pub = pnh.advertise<sensor_msgs::PointCloud2>("current_plane_cloud", 1, true);
        remaining_cloud_pub = pnh.advertise<sensor_msgs::PointCloud2>("remaining_cloud", 1, true);
        horizontal_plane_marker_pub = pnh.advertise<visualization_msgs::Marker>("current_plane_box", 1, true);
    }
    ros::spin();
};