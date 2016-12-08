/*
 * Implements the people detection algorithm described here:
 * M. Munaro, F. Basso and E. Menegatti,
 * Tracking people within groups with RGB-D data,
 * In Proceedings of the International Conference on Intelligent Robots and Systems (IROS) 2012, Vilamoura (Portugal), 2012.
 */

#include <signal.h>
#include <vector>
#include <string>
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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/common/time.h>
#include <pcl/filters/crop_box.h>

using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//some custom functions
#include "file_io.h"

//some constants
bool visualize;
bool write_to_file;
string record_file;
ofstream outfile;

const string data_topic = "nav_kinect/depth_registered/points";
const string classifier_location = ros::package::getPath("person_detector") + "/data/classifier.yaml";

//refresh rate
double ros_rate = 10.0;

Eigen::VectorXf ground_coef;

// Mutex: //
boost::mutex cloud_mutex;

bool new_cloud_available_flag = false;
PointCloudT::Ptr cloud (new PointCloudT);
PointCloudT::Ptr person_cloud (new PointCloudT);
sensor_msgs::PointCloud2 person_cloud_ros;

void sig_handler(int sig)
{
    ros::shutdown();
    exit(1);
};

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr &input)
{
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

visualization_msgs::Marker create_marker(geometry_msgs::PoseStamped stampOut,
        string frame_id, string ns,	int k)
{

    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "people";
    marker.id = k;

    // Set the marker type
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker
    marker.pose = stampOut.pose;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.25;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.6;

    marker.lifetime = ros::Duration(0.0);

    return marker;
}

int main (int argc, char **argv)
{
    // Initialize ROS
    ros::init (argc, argv, "segbot_person_detection");
    ros::NodeHandle nh;

    nh.param<bool>("person_detection/visualize", visualize, false);
    nh.param<double>("person_detection/rate", ros_rate, 10.0);
    nh.param<bool>("person_detection/write_to_file", write_to_file, true);
    nh.param<string>(string("person_detection/record_file"), record_file,
                     ros::package::getPath("person_detector") + "/data/record.txt");

    string param_out_frame_id;
    nh.param<string>(string("person_detection/out_frame_id"), param_out_frame_id, "/map");

    string param_topic;
    nh.param<string>(string("person_detection/rgbd_topic"), param_topic, data_topic);

    string param_classifier;
    nh.param<string>(string("person_detection/classifier_location"), param_classifier,
                     ros::package::getPath("person_detector") + "/data/classifier.yaml");

    string param_sensor_frame_id;
    nh.param<string>(string("person_detection/sensor_frame_id"), param_sensor_frame_id,
                     "/nav_kinect_rgb_optical_frame");


    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("pcl_detector/marker", 10);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pcl_detector/human_poses", 10);
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_detector/human_clouds", 10);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe (param_topic, 1, cloud_cb);

    // Algorithm parameters:
    string svm_filename = param_classifier;
    float min_confidence = -1.5;//-1.9
    float min_height = 1.3;
    float max_height = 2.3;
    float voxel_size = 0.06;
    Eigen::Matrix3f rgb_intrinsics_matrix;
    rgb_intrinsics_matrix << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0, 1.0; // Kinect RGB camera intrinsics

    //register ctrl-c
    signal(SIGINT, sig_handler);

    //load ground plane coeffs
    ground_coef.resize(4);
    string ground_plane_file, path_to_package, path;

    if (!ros::param::get("person_detection/ground_plane_file", ground_plane_file)){
        ROS_ERROR("ground_plane_file parameter needs to be set");
        ros::shutdown();
        return -1;
    }

    ROS_INFO("Reading ground coefficients from \"%s\"", ground_plane_file.c_str());
    ground_coef = load_vector_from_file(ground_plane_file.c_str(), 4);

    // Initialize new viewer:
    pcl::visualization::PCLVisualizer *viewer_display;          // viewer initialization
    if (visualize)
    {
        viewer_display = new pcl::visualization::PCLVisualizer("People Viewer");
        viewer_display->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
    }

    // Create classifier for people detection:
    pcl::people::PersonClassifier<pcl::RGB> person_classifier;
    person_classifier.loadSVMFromFile(param_classifier);   // load trained SVM

    // People detection app initialization:
    pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector;    // people detection object
    people_detector.setVoxelSize(voxel_size);                        // set the voxel size
    people_detector.setIntrinsics(rgb_intrinsics_matrix);            // set RGB camera intrinsic parameters
    people_detector.setClassifier(person_classifier);                // set person classifier
    people_detector.setHeightLimits(min_height, max_height);         // set person classifier
    //  people_detector.setSensorPortraitOrientation(true);             // set sensor orientation to vertical
    ROS_INFO("Initializing people detection...");

    // For timing:
    static unsigned count = 0;
    static double last = pcl::getTime ();

    int detection_count = 0;
    bool set_ground = false;

    ros::Rate r(ros_rate);

    tf::TransformListener listener;
    tf::StampedTransform transform;

    // Main loop:
    while (ros::ok())
    {
        //collect messages
        ros::spinOnce();

        r.sleep();

        if (new_cloud_available_flag && cloud_mutex.try_lock ())    // if a new cloud is available
        {
            new_cloud_available_flag = false;

            // Perform people detection on the new cloud:
            vector<pcl::people::PersonCluster<PointT> > clusters;   // vector containing persons clusters
            vector<pcl::people::PersonCluster<PointT> > clusters_filtered;
            people_detector.setInputCloud(cloud);
            people_detector.setGround(ground_coef);

            people_detector.compute(clusters);                           // perform people detection

            ground_coef = people_detector.getGround();                 // get updated floor coefficients

            // Draw cloud and people bounding boxes in the viewer:
            if (visualize)
            {
                viewer_display->removeAllPointClouds();
                viewer_display->removeAllShapes();
                pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
                viewer_display->addPointCloud<PointT> (cloud, rgb, "input_cloud");
            }

            int k = 0;

            for(vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
            {
                if(it->getPersonConfidence() > min_confidence)             // draw only people with confidence above a threshold
                {

                    Eigen::Vector3f centroid_k = it->getCenter();
                    Eigen::Vector3f top_k = it->getTop();
                    Eigen::Vector3f bottom_k = it->getBottom();

                    //calculate the distance from the centroid of the cloud to the plane
                    pcl::PointXYZ p_k;

                    p_k.x = bottom_k(0);
                    p_k.y = bottom_k(1);
                    p_k.z = bottom_k(2);
                    double dist_to_ground_bottom = pcl::pointToPlaneDistance(p_k, ground_coef);

                    p_k.x = top_k(0);
                    p_k.y = top_k(1);
                    p_k.z = top_k(2);
                    double dist_to_ground_top = pcl::pointToPlaneDistance(p_k, ground_coef);

                    p_k.x = centroid_k(0);
                    p_k.y = centroid_k(1);
                    p_k.z = centroid_k(2);
                    double dist_to_ground = pcl::pointToPlaneDistance(p_k, ground_coef);

                    //ROS_INFO("Cluter centroid: %f, %f, %f",centroid_k(0),centroid_k(1),centroid_k(2));
                    //ROS_INFO("\tDistance to ground (top): %f",dist_to_ground_top);
                    //ROS_INFO("\tDistance to ground (centroid): %f",dist_to_ground);
                    //ROS_INFO("\tDistance to ground (bottom): %f",dist_to_ground_bottom);
                    //ROS_INFO("\tCluster height: %f",it->getHeight());
                    //ROS_INFO("\tCluster points: %i",it->getNumberPoints());
                    //ROS_INFO("\tDistance from sensor: %f",it->getDistance());
                    //ROS_INFO("\tconfidence: %f",it->getPersonConfidence());

                    if (it->getNumberPoints() > 250 && it->getNumberPoints() < 500
                            && it->getHeight() > 1.2 && it->getHeight() < 2.1
                            && dist_to_ground_bottom < 0.3)
                    {

                        // draw theoretical person bounding box in the PCL viewer:
                        if (visualize)
                            it->drawTBoundingBox(*viewer_display, k);

                        //get just the person out of the whole cloud
                        Eigen::Vector4f minPoint, maxPoint;
                        for (int i = 0; i < 3; i++)
                        {
                            minPoint[i] = it->getMin()[i];
                            maxPoint[i] = it->getMax()[i];
                        }

                        pcl::CropBox<PointT> cropFilter;
                        cropFilter.setInputCloud (cloud);
                        cropFilter.setMin(minPoint);
                        cropFilter.setMax(maxPoint);
                        cropFilter.filter (*person_cloud);


                        //publish person cloud
                        pcl::toROSMsg(*person_cloud, person_cloud_ros);
                        person_cloud_ros.header.frame_id = param_sensor_frame_id;
                        cloud_pub.publish(person_cloud_ros);
                        ROS_INFO("Person Cloud published.");

                        //transforms the pose into /map frame
                        geometry_msgs::PoseStamped stampedPose;

                        stampedPose.header.frame_id = param_sensor_frame_id;
                        stampedPose.header.stamp = ros::Time(0);
                        stampedPose.pose.position.x = centroid_k(0);
                        stampedPose.pose.position.y = centroid_k(1);
                        stampedPose.pose.position.z = centroid_k(2);
                        stampedPose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -3.14 / 2);
                        //cout << param_sensor_frame_id << endl << stampedPose.pose.position << endl;

                        geometry_msgs::PoseStamped stampOut;
                        listener.waitForTransform(param_sensor_frame_id, param_out_frame_id, ros::Time(0), ros::Duration(3.0));
                        listener.transformPose(param_out_frame_id, stampedPose, stampOut);

                        //transform the human point cloud into presumably the /map frame of reference
                        pcl_ros::transformPointCloud (param_out_frame_id, person_cloud_ros, person_cloud_ros, listener);

                        //save to file for analysis
                        ros::Time nowTime = ros::Time::now();

                        //save cloud in map frame of reference
                        pcl::fromROSMsg(person_cloud_ros, *person_cloud);

                        stampOut.header.stamp = nowTime;
                        //cout << param_out_frame_id << endl << stampOut.pose.position << endl;
                        if (write_to_file)
				write_pos_to_file(record_file.c_str(), stampOut);

                        //publish the marker
                        visualization_msgs::Marker marker_k = create_marker(stampOut, param_out_frame_id, "pcl_detector", detection_count);
                        marker_k.pose = stampOut.pose;
                        marker_pub.publish(marker_k);

                        //publish the pose
                        pose_pub.publish(stampOut);

                        k++;

                        detection_count++;
                    }
                }
            }

            if (visualize)
            {
                viewer_display->spinOnce();
            }

            cloud_mutex.unlock ();
        }
    }

    return 0;
}

