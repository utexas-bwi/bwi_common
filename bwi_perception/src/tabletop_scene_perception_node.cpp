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


#include <pcl/kdtree/kdtree.h>

#include "bwi_perception/PerceiveTabletopScene.h"
#include "bwi_perception/PerceiveLargestHorizontalPlane.h"
#include "bwi_perception/GetCloud.h"
#include "bwi_perception/GetPCD.h"
#include <bwi_perception/filter.h>
#include <mutex>
#include <condition_variable>
#include <bwi_perception/bwi_perception.h>
#include <bwi_perception/tabletop.h>
#include <bwi_perception/plane.h>
#include <bwi_perception/BoundingBox.h>

using namespace std;
//how many frames to stitch into a single cloud
int num_clouds;

//default Z filter
double z_filter_min, z_filter_max;

double cluster_extraction_tolerance, plane_max_distance_tolerance, plane_distance_tolerance, eps_angle;

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

ros::NodeHandle *persistent_nh;
ros::Subscriber camera_cloud_sub;
std::string up_frame;
string camera_cloud_topic;


mutex cloud_lock;
PointCloudT::Ptr cloud(new PointCloudT);

ros::Publisher objects_cloud_pub;
ros::Publisher table_cloud_pub;

uint64_t cloud_count = 0;
mutex cloud_count_lock;
condition_variable cloud_count_cv;

//true if Ctrl-C is pressed
bool g_caught_sigint = false;

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
    ROS_INFO("waiting for cloud...");
    aggregate_clouds(num_clouds, working);

    ROS_INFO("collected cloud success");
    float x_min = -std::numeric_limits<float>::max();
    float x_max = std::numeric_limits<float>::max();
    float z_min = z_filter_min;
    float z_max = z_filter_max;
    if (req.override_filter_z) {
        z_min = req.min_z_value;
        z_max = req.max_z_value;
    }
    if (req.apply_x_box_filter) {
        x_min = req.x_min;
        x_max = req.x_max;
    }
    bwi_perception::filter_cloud_region<PointT>(working, cloud_plane, up_frame, z_min, z_max, x_min, x_max, tf_listener);
    working.swap(cloud_plane);

    double height;
    PointCloudT::Ptr table_cloud(new PointCloudT);
    Eigen::Vector4f plane_coefficients;
    vector<PointCloudT::Ptr> table_objects;
    bwi_perception::segment_tabletop_scene(working, cluster_extraction_tolerance, up_frame, table_cloud,
                                           plane_coefficients, table_objects, plane_distance_tolerance,
                                           plane_max_distance_tolerance, height, 50);

    ROS_INFO("Found %i clusters on the plane.", (int) table_objects.size());

    //fill in responses
    //plane cloud and coefficient
    pcl::toROSMsg(*table_cloud, res.cloud_plane);
    for (int i = 0; i < 4; i++) {
        res.cloud_plane_coef[i] = plane_coefficients(i);
    }

    res.table_height = height;

    transform(table_objects.begin(), table_objects.end(), back_inserter(res.cloud_clusters),
              [](PointCloudT::Ptr pcl_cloud) {
                  sensor_msgs::PointCloud2 ros_cloud;
                  pcl::toROSMsg(*pcl_cloud, ros_cloud);
                  return ros_cloud;
              });

    sensor_msgs::PointCloud2 cloud_ros;

    for (int i = 0; i < table_objects.size(); i++) {
        const bwi_perception::BoundingBox &oriented_bbox_params = bwi_perception::BoundingBox::oriented_from_cloud<PointT>(table_objects[i]);
        res.bounding_boxes.markers.push_back(oriented_bbox_params.to_marker(0, "map"));
    }

    if (objects_cloud_pub.getNumSubscribers() > 0) {
        //for debugging purposes
        //now, put the clouds in cluster_on_plane in one cloud and publish it
        cloud_blobs->clear();

        for (auto &i : table_objects) {
            *cloud_blobs += *i;
        }
        ROS_INFO("Publishing debug cloud...");
        pcl::toROSMsg(*cloud_blobs, cloud_ros);
        objects_cloud_pub.publish(cloud_ros);
    }

    if (table_cloud_pub.getNumSubscribers() > 0) {
        table_cloud_pub.publish(res.cloud_plane);
    }


    return true;
}

bool get_largest_horizontal_plane_cb(bwi_perception::PerceiveLargestHorizontalPlane::Request &req, bwi_perception::PerceiveLargestHorizontalPlane::Response &res) {

    PointCloudT::Ptr working(new PointCloudT);
    PointCloudT::Ptr filtered(new PointCloudT);

    pcl::PointIndices::Ptr plane_indices(new pcl::PointIndices());
    //create listener for transforms
    tf::TransformListener tf_listener;

    ROS_INFO("waiting for cloud...");
    aggregate_clouds(num_clouds, working);

    float x_min = -std::numeric_limits<float>::max();
    float x_max = std::numeric_limits<float>::max();
    float z_min = z_filter_min;
    float z_max = z_filter_max;
    if (req.override_filter_z) {
        z_min = req.min_z_value;
        z_max = req.max_z_value;
    }
    if (req.apply_x_box_filter) {
        x_min = req.x_min;
        x_max = req.x_max;
    }
    bwi_perception::filter_cloud_region<PointT>(working, filtered, up_frame, z_min, z_max, x_min, x_max, tf_listener);
    working.swap(filtered);

    Eigen::Vector4f plane_coefficients;
    bool success = bwi_perception::get_largest_plane<PointT>(working, plane_indices, plane_coefficients, up_frame, tf_listener);

    if (!success) {
        res.is_plane_found = false;
        return true;
    }

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    // Extract the plane
    extract.setInputCloud(working);
    extract.setIndices(plane_indices);
    extract.setNegative(false);
    extract.filter(*working);


    bwi_perception::filter_plane_selection<PointT>(working, filtered, cluster_extraction_tolerance);


    res.is_plane_found = true;
    pcl::toROSMsg(*filtered, res.cloud_plane);
    for (int i = 0; i < 4; i++) {
        res.cloud_plane_coef[i] = plane_coefficients(i);
    }

    if (table_cloud_pub.getNumSubscribers() > 0) {
        table_cloud_pub.publish(res.cloud_plane);
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


    got_param = pnh.getParam("up_frame", up_frame);
    if (!got_param) {
        ROS_ERROR("Could not retrieve up_frame argument.");
        exit(1);
    }

    pnh.param("num_clouds", num_clouds, 1);
    pnh.param("z_filter_min", z_filter_min, 0.55);
    pnh.param("z_filter_max", z_filter_max, 1.55);

    //an object whose closest point to the plane is further than this is rejected
    pnh.param("plane_distance_tolerance", plane_distance_tolerance, 0.09);
    //an object whose furthest point to the plane is nearer than this is rejected
    pnh.param("plane_max_distance_tolerance", plane_max_distance_tolerance, 0.02);
    //how close points outside the plane must be to go into the same object cluster
    pnh.param("cluster_extraction_tolerance", cluster_extraction_tolerance, 0.05);

    pnh.param("eps_angle", eps_angle, 0.09);

    camera_cloud_sub = persistent_nh->subscribe(camera_cloud_topic, 100, cloud_cb);

    //debugging publisher
    objects_cloud_pub = pnh.advertise<sensor_msgs::PointCloud2>("objects_cloud", 1);
    table_cloud_pub = pnh.advertise<sensor_msgs::PointCloud2>("plane_cloud", 1);

    //services
    ros::ServiceServer service = nh.advertiseService("perceive_tabletop_scene", seg_cb);

    ros::ServiceServer get_largest_plane_service = nh.advertiseService("perceive_largest_horizontal_plane", get_largest_horizontal_plane_cb);

    ros::ServiceServer getcloud_service = nh.advertiseService("get_aggregated_cloud", get_cloud_cb);

    ros::ServiceServer get_pcd_service = nh.advertiseService("save_pcd", get_pcd_cb);

    //register ctrl-c
    signal(SIGINT, sig_handler);

    ros::MultiThreadedSpinner spinner(2); // Use 2 threads
    spinner.spin();
};
