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

#include "bwi_perception/PerceiveTabletopScene.h"
#include "bwi_perception/GetCloud.h"
#include "bwi_perception/GetPCD.h"
#include <mutex>
#include <condition_variable>
#include <bwi_perception/bwi_perception.h>

using namespace std;
//how many frames to stitch into a single cloud
#define WAIT_CLOUD_K 25

//default Z filter
#define Z_MIN_DEFAULT 0.00
#define Z_MAX_DEFAULT 1.15

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

ros::NodeHandle *persistent_nh;
ros::Subscriber camera_cloud_sub;
string camera_cloud_topic;


mutex cloud_lock;
PointCloudT::Ptr cloud(new PointCloudT);

ros::Publisher cloud_pub;
ros::Publisher table_cloud_pub;

uint64_t cloud_count = 0;
mutex cloud_count_lock;
condition_variable cloud_count_cv;

//true if Ctrl-C is pressed
bool g_caught_sigint = false;

//an object whose closest point to the plane is further than this is rejected
double plane_distance_tolerance = 0.09;

//an object whose furthers point to the plane is smaller than this is rejected
double plane_max_distance_tolerance = 0.02;

//how close to points outside the plane must go into the same object cluster
double cluster_extraction_tolerance = 0.075;

//epsilon angle for segmenting, value in radians
#define EPS_ANGLE 0.09

/* what happens when ctr-c is pressed */
void sig_handler(int sig) {
    g_caught_sigint = true;
    ROS_INFO("caught sigint, init shutdown sequence...");
    ros::shutdown();
    exit(1);
};


void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input) {
    //convert to PCL format
    {
        lock_guard<mutex> guard(cloud_lock);
        pcl::fromROSMsg(*input, *cloud);
    }

    {
        lock_guard<mutex> guard(cloud_count_lock);
        cloud_count++;
    }

    cloud_count_cv.notify_all();
}

bool get_pcd_cb(bwi_perception::GetPCD::Request &req, bwi_perception::GetPCD::Response &res) {

    //get the start time of recording
    double begin = ros::Time::now().toSec();
    std::string startTime = boost::lexical_cast<std::string>(begin);

    //save file
    std::string filename = req.generalImageFilePath + "/" + startTime + ".pcd";

    {
        lock_guard<mutex> guard(cloud_lock);
        pcl::io::savePCDFileASCII(filename, *cloud);
    }


    ROS_INFO("Saved pcd file %s", filename.c_str());

    res.success = true;
    return true;
}


void wait_for_cloud(uint n = 1) {
    // Wait until the cloud cb notifies us
    uint64_t old_cloud_count = cloud_count;
    unique_lock<std::mutex> unique_lock(cloud_count_lock);
    cloud_count_cv.wait(unique_lock, [=]{ return old_cloud_count + n <= cloud_count; });
}

/* collects a cloud by aggregating k successive frames */
void aggregate_clouds(uint n, PointCloudT::Ptr &out) {

    int counter = 0;
    while (ros::ok()) {

        wait_for_cloud();
        {
            lock_guard<mutex> guard(cloud_lock);
            *out += *cloud;
            if (counter >= n) {
                out->header = cloud->header;
                break;
            }
        }
        counter++;
    }

}

bool seg_cb(bwi_perception::PerceiveTabletopScene::Request &req, bwi_perception::PerceiveTabletopScene::Response &res) {
    ROS_INFO("Request received...starting pipeline.");

    PointCloudT::Ptr cloud_plane(new PointCloudT);
    PointCloudT::Ptr cloud_blobs(new PointCloudT);
    PointCloudT::Ptr working(new PointCloudT);

    //create listener for transforms
    tf::TransformListener tf_listener;
    // TODO: Separate perception and detection.
    // There should be a service that accepts a pointcloud so that we can feed in stuff
    // from an octomap. Filtering would be done by the caller. This service
    // would become the convenience wrapper
    //get the point cloud by aggregating k successive input clouds
    ROS_INFO("waiting for cloud...");
    aggregate_clouds(WAIT_CLOUD_K, working);

    ROS_INFO("collected cloud success");

    double min_z = Z_MIN_DEFAULT;
    double max_z = Z_MAX_DEFAULT;
    if (req.override_filter_z) {
        min_z = req.min_z_value;
        max_z = req.max_z_value;
    }

    // Apply z filter -- we don't care for anything X m away in the z direction
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(working);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_z, max_z);
    pass.filter(*working);

    // Apply x filter
    if (req.apply_x_box_filter) {
        pass.setInputCloud(working);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(req.x_min, req.x_max);
        pass.filter(*working);
    }

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<PointT> vg;
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    vg.setInputCloud(working);
    vg.setLeafSize(0.0025f, 0.0025f, 0.0025f);
    vg.filter(*cloud_filtered);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
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
    ros_vec.header.frame_id = "/base_link";
    ros_vec.vector.x = 0.0;
    ros_vec.vector.y = 0.0;
    ros_vec.vector.z = 1.0;

    ROS_INFO("Ros axis: %f, %f, %f",
             ros_vec.vector.x, ros_vec.vector.y, ros_vec.vector.z);

    //transform the vector to the camera frame of reference
    tf_listener.transformVector(working->header.frame_id, ros::Time(0), ros_vec, "/base_link", ros_vec);

    //set the axis to the transformed vector
    Eigen::Vector3f axis = Eigen::Vector3f(ros_vec.vector.x, ros_vec.vector.y, ros_vec.vector.z);
    seg.setAxis(axis);

    ROS_INFO("sac axis value: %f, %f, %f", seg.getAxis()[0], seg.getAxis()[1], seg.getAxis()[2]);

    //set an epsilon that the table can differ from the axis above by
    seg.setEpsAngle(EPS_ANGLE); //value in radians, corresponds to approximately 5 degrees

    ROS_INFO("epsilon value: %f", seg.getEpsAngle());

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);

    // Extract the plane
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_plane);

    //downsample the plane cloud and segment out noise
    vg.setInputCloud(cloud_plane);
    vg.setLeafSize(0.005f, 0.005f, 0.005f);
    vg.filter(*cloud_plane);

    //find the largest plane and segment out noise
    cloud_plane = bwi_perception::seg_largest_plane<PointT>(cloud_plane, cluster_extraction_tolerance, 500);

    //make sure the cloud plane exists still
    res.is_plane_found = true;
    if (cloud_plane->empty()) {
        res.is_plane_found = false;
        return true;
    }

    //extract everything else
    extract.setNegative(true);
    extract.filter(*cloud_blobs);

    //get the plane coefficients
    Eigen::Vector4f plane_coefficients;
    plane_coefficients(0) = coefficients->values[0];
    plane_coefficients(1) = coefficients->values[1];
    plane_coefficients(2) = coefficients->values[2];
    plane_coefficients(3) = coefficients->values[3];

    ROS_INFO("Planar coefficients: %f, %f, %f, %f",
             plane_coefficients(0), plane_coefficients(1), plane_coefficients(2), plane_coefficients(3));

    //debug plane cloud
    //convert plane cloud to ROS
    sensor_msgs::PointCloud2 ros_plane;

    //pcl::toROSMsg(*cloud_plane, ros_plane);
    pcl::toROSMsg(*cloud_plane, ros_plane);

    ros_plane.header.frame_id = cloud->header.frame_id;
    table_cloud_pub.publish(ros_plane);

    vector<PointCloudT::Ptr> clusters;

    //Step 3: Eucledian Cluster Extraction
    bwi_perception::compute_clusters<PointT>(cloud_blobs, clusters, cluster_extraction_tolerance);

    ROS_INFO("Found %i clusters.", (int) clusters.size());


    //if true, clouds on the other side of the plane will be rejected
    bool check_below_plane = true;
    double plane_z = -1.0;
    PointCloudT::Ptr cloud_plane_baselink(new PointCloudT);
    Eigen::Vector4f plane_centroid;

    if (check_below_plane) {

        //wait for transform and perform it
        tf_listener.waitForTransform(working->header.frame_id, "/base_link", ros::Time(0), ros::Duration(3.0));

        //convert plane cloud to ROS
        sensor_msgs::PointCloud2 plane_cloud_ros;
        pcl::toROSMsg(*cloud_plane, plane_cloud_ros);
        plane_cloud_ros.header.frame_id = working->header.frame_id;

        //transform it to base link frame of reference
        pcl_ros::transformPointCloud("/base_link", plane_cloud_ros, plane_cloud_ros, tf_listener);

        //convert to PCL format and take centroid
        pcl::fromROSMsg(plane_cloud_ros, *cloud_plane_baselink);
        pcl::compute3DCentroid(*cloud_plane_baselink, plane_centroid);

        ROS_INFO("[table_object_detection_node.cpp] Plane xyz: %f, %f, %f", plane_centroid(0), plane_centroid(1),
                 plane_centroid(2));
    }

    vector<PointCloudT::Ptr> clusters_on_plane;
    for (unsigned int i = 0; i < clusters.size(); i++) {

        if (bwi_perception::filter<PointT>(clusters.at(i), plane_coefficients, plane_distance_tolerance,
                   plane_max_distance_tolerance)) {

            //next check which clusters are below and which are aboe the plane
            if (check_below_plane) {
                sensor_msgs::PointCloud2 cloud_i_ros;
                pcl::toROSMsg(*clusters.at(i), cloud_i_ros);
                cloud_i_ros.header.frame_id = cloud->header.frame_id;

                pcl_ros::transformPointCloud("/base_link", cloud_i_ros, cloud_i_ros, tf_listener);

                //convert to PCL format and take centroid
                PointCloudT::Ptr cluster_i_baselink(new PointCloudT);
                pcl::fromROSMsg(cloud_i_ros, *cluster_i_baselink);

                Eigen::Vector4f centroid_i;
                pcl::compute3DCentroid(*cluster_i_baselink, centroid_i);

                ROS_INFO("[table_object_detection_node.cpp] cluster %i xyz: %f, %f, %f", i, centroid_i(0),
                         centroid_i(1), centroid_i(2));

                //check z's
                if (centroid_i(2) < plane_centroid(2)) {
                    //below the table (maybe the edge of the table, etc)
                    ROS_INFO("[table_object_detection_node.cpp] Rejected as below the table...");
                } else {
                    //above the table
                    clusters_on_plane.push_back(clusters.at(i));
                }

                //TODO: check if cloud_i_ros has larger z than plane_cloud_ros
            } else {
                clusters_on_plane.push_back(clusters.at(i));
            }
        }
    }

    ROS_INFO("Found %i clusters on the plane.", (int) clusters_on_plane.size());

    //fill in responses
    //plane cloud and coefficient
    pcl::toROSMsg(*cloud_plane, res.cloud_plane);
    res.cloud_plane.header.frame_id = cloud->header.frame_id;
    for (int i = 0; i < 4; i++) {
        res.cloud_plane_coef[i] = plane_coefficients(i);
    }

    //blobs on the plane
    for (auto &i : clusters_on_plane) {
        sensor_msgs::PointCloud2 cloud_ros;
        pcl::toROSMsg(*i, cloud_ros);
        cloud_ros.header.frame_id = cloud->header.frame_id;
        res.cloud_clusters.push_back(cloud_ros);
    }

    sensor_msgs::PointCloud2 cloud_ros;
    if (cloud_pub.getNumSubscribers() > 0) {
        //for debugging purposes
        //now, put the clouds in cluster_on_plane in one cloud and publish it
        cloud_blobs->clear();

        for (auto &i : clusters_on_plane) {
            *cloud_blobs += *i;
        }
        ROS_INFO("Publishing debug cloud...");
        pcl::toROSMsg(*cloud_blobs, cloud_ros);
        cloud_ros.header.frame_id = cloud->header.frame_id;
        cloud_pub.publish(cloud_ros);
    }


    if (table_cloud_pub.getNumSubscribers() > 0) {
        pcl::toROSMsg(*cloud_plane, cloud_ros);
        cloud_ros.header.frame_id = cloud->header.frame_id;
        table_cloud_pub.publish(cloud_ros);
    }


    return true;
}


bool get_cloud_cb(bwi_perception::GetCloud::Request &req, bwi_perception::GetCloud::Response &res) {
    ROS_INFO("[table_object_detection_node.cpp] retrieving point cloud...");
    PointCloudT::Ptr working(new PointCloudT);
    aggregate_clouds(15, working);
    pcl::toROSMsg(*working, res.cloud);
    res.cloud.header.frame_id = cloud->header.frame_id;
    return true;
}

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "tabletop_scene_perceiver");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    // TODO: Move all ROS data juggling into a class so we don't need to expose stack pointers like this
    persistent_nh = &nh;
    // Create a ROS subscriber for the input point cloud
    bool got_param = pnh.getParam("pointcloud_topic", camera_cloud_topic);
    if (!got_param) {
        ROS_ERROR("Could not retrieve pointcloud_topic argument.");
        exit(1);
    }

    std::string param_up_frame;
    got_param = pnh.getParam("pointcloud_topic", param_up_frame);
    if (!got_param) {
        ROS_ERROR("Could not retrieve up_frame argument.");
        exit(1);
    }
    camera_cloud_sub = persistent_nh->subscribe(camera_cloud_topic, 100, cloud_cb);

    //debugging publisher
    cloud_pub = pnh.advertise<sensor_msgs::PointCloud2>("objects_cloud", 1);
    table_cloud_pub = pnh.advertise<sensor_msgs::PointCloud2>("plane_cloud", 1);

    //services
    ros::ServiceServer service = nh.advertiseService("perceive_tabletop_scene", seg_cb);

    ros::ServiceServer getcloud_service = nh.advertiseService("get_aggregated_cloud", get_cloud_cb);

    ros::ServiceServer get_pcd_service = nh.advertiseService("save_pcd", get_pcd_cb);

    //register ctrl-c
    signal(SIGINT, sig_handler);

    ros::MultiThreadedSpinner spinner(2); // Use 2 threads
    spinner.spin();
};
