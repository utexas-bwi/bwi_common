/*
 * Author: Matteo Munaro
 *
 * M. Munaro, F. Basso and E. Menegatti,
 * Tracking people within groups with RGB-D data,
 * In Proceedings of the International Conference on Intelligent Robots and Systems (IROS) 2012, Vilamoura (Portugal), 2012.
 */

#include <signal.h>
#include <vector>
#include <string.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/common/time.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//some custom functions
#include "file_io.h"

std::string data_topic;
std::string ground_coef_file;

//store the plane coefficients for the ground in kinnect frame of reference
Eigen::VectorXf ground_coeffs;

// Mutex: //
boost::mutex cloud_mutex;

bool new_cloud_available_flag = false;
PointCloudT::Ptr cloud (new PointCloudT);

// viewer
pcl::visualization::PCLVisualizer *viewer_display;

void sig_handler(int sig)
{
    ros::shutdown();
    exit(1);
};

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr &input)
{
    //ROS_INFO("Heard cloud: %s",input->header.frame_id.c_str());
    cloud_mutex.lock ();

    //convert to PCL format
    pcl::fromROSMsg (*input, *cloud);

    //state that a new cloud is available
    new_cloud_available_flag = true;

    cloud_mutex.unlock ();
}

struct callback_args
{
    // structure used to pass arguments to the callback function
    PointCloudT::Ptr clicked_points_3d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void pp_callback (const pcl::visualization::PointPickingEvent &event, void *args)
{
    struct callback_args *data = (struct callback_args *)args;
    if (event.getPointIndex () == -1)
        return;

    PointT current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);
    data->clicked_points_3d->points.push_back(current_point);

    // Draw clicked points in red:
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red (data->clicked_points_3d, 255, 0, 0);
    data->viewerPtr->removePointCloud("clicked_points");
    data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
    data->viewerPtr->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
}

int main (int argc, char **argv)
{
    ros::init (argc, argv, "segbot_pcl_person_detector");
    ros::NodeHandle nh;

    nh.param<std::string>("calibrate_floor/data_topic", data_topic, "nav_kinect/depth_registered/points");
    ros::Subscriber sub = nh.subscribe (data_topic, 1, cloud_cb);

    signal(SIGINT, sig_handler);

    ROS_INFO("Waiting for next cloud...");
    while(!new_cloud_available_flag)
    {
        ros::spinOnce();
    }
    ROS_INFO("Heard first cloud, calibrating ground plane...");

    new_cloud_available_flag = false;

    cloud_mutex.lock ();    // for not overwriting the point cloud

    // Display pointcloud:
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);

    // PCL viewer
    pcl::visualization::PCLVisualizer viewer_calibrate("PCL Viewer");
    viewer_calibrate.addPointCloud<PointT> (cloud, rgb, "input_cloud");
    viewer_calibrate.setCameraPosition(0, 0, -2, 0, -1, 0, 0);

    // Add point picking callback to viewer:
    struct callback_args cb_args;
    PointCloudT::Ptr clicked_points_3d (new PointCloudT);
    cb_args.clicked_points_3d = clicked_points_3d;
    cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(&viewer_calibrate);
    viewer_calibrate.registerPointPickingCallback (pp_callback, (void *)&cb_args);
    ROS_INFO("Shift+click on three floor points, then press 'Q'...");

    // Spin until 'Q' is pressed:
    viewer_calibrate.spin();
    cloud_mutex.unlock ();

    // Ground plane estimation:
    std::vector<int> clicked_points_indices;
    for (unsigned int i = 0; i < clicked_points_3d->points.size(); i++)
        clicked_points_indices.push_back(i);

    // initialize ground coefficiend vector
    ground_coeffs.resize(4);
    pcl::SampleConsensusModelPlane<PointT> model_plane(clicked_points_3d);
    model_plane.computeModelCoefficients(clicked_points_indices, ground_coeffs);

    //save to file
    nh.param<std::string>("calibrate_floor/ground_coef_file", ground_coef_file, 
        ros::package::getPath("pcl_detection") + "/data/ground_coef.txt");
    write_vector_to_file(ground_coef_file.c_str(), ground_coeffs, 4);

    ROS_INFO("Coefficients written to \"%s\": %f %f %f %f", ground_coef_file.c_str(),
             ground_coeffs(0), ground_coeffs(1), ground_coeffs(2), ground_coeffs(3));
    ROS_INFO("Process will be dead. Relaunch for recalibration.");
    
    ros::shutdown();
    return 0;
}

