#include <stdio.h>
#include <iostream>
#include <cstdio> 
#include <boost/filesystem.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "bwi_scavenger/ObjectDetection.h"

using namespace cv;

cv_bridge::CvImageConstPtr cv_ptr;
Mat frame;

std::string file, name, directory; 
std::string default_dir = "/home/bwi/shiqi/";


void callback(const sensor_msgs::ImageConstPtr& msg) 
{
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8); 
    frame = cv_ptr->image;
}


bool callback_detection(bwi_scavenger::ObjectDetection::Request &req, 
    bwi_scavenger::ObjectDetection::Response &res) {

    file  = req.path_to_template; 
    Mat img_object = imread(file, CV_LOAD_IMAGE_GRAYSCALE );
    if(! img_object.data )
        ROS_ERROR("No template image is provided for object detection"); 

    cv_ptr.reset (new cv_bridge::CvImage);

    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 400;
    SurfFeatureDetector detector( minHessian );
    std::vector<KeyPoint> keypoints_object, keypoints_frame;

    detector.detect( img_object, keypoints_object );

    //-- Step 2: Calculate descriptors (feature vectors)
    SurfDescriptorExtractor extractor;
    Mat descriptors_object, descriptors_frame;

    extractor.compute( img_object, keypoints_object, descriptors_object );

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;

    // namedWindow("WindowName", CV_WINDOW_AUTOSIZE);

    int cnt = 0;

    ros::param::param<std::string>("~name", name, "NAME");

    while (ros::ok()) {

        ros::spinOnce();

        if (frame.empty()) {
          ROS_WARN("frame empty");
          ros::Duration(0.1).sleep(); 
          ros::spinOnce();
          continue;
        }

        detector.detect(frame, keypoints_frame);
        extractor.compute(frame, keypoints_frame, descriptors_frame);
        // Finds the best match for each descriptor from a query set.
        matcher.match(descriptors_object, descriptors_frame, matches);

        //-- Quick calculation of max and min distances between keypoints
        double max_dist = 0; double min_dist = 100;

        for( int i = 0; i < descriptors_object.rows; i++ )
        { double dist = matches[i].distance;
          if( dist < min_dist ) min_dist = dist;
          if( dist > max_dist ) max_dist = dist;
        }
        // printf("-- Max dist : %f \n", max_dist );
        // printf("-- Min dist : %f \n", min_dist );

        //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
        std::vector< DMatch > good_matches;

        for( int i = 0; i < descriptors_object.rows; i++ )
        { if( matches[i].distance < 3*min_dist )
           { good_matches.push_back( matches[i]); }
        }

        Mat img_matches;
        drawMatches( img_object, keypoints_object, frame, keypoints_frame,
                     good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                     vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

        //-- Localize the object
        std::vector<Point2f> obj;
        std::vector<Point2f> scene;

        for( int i = 0; i < good_matches.size(); i++ )
        {
          //-- Get the keypoints from the good matches
          obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
          scene.push_back( keypoints_frame[ good_matches[i].trainIdx ].pt );
        }

        if (obj.size() < 4 || scene.size() < 4) 
          continue;

        Mat H = findHomography( obj, scene, CV_RANSAC );

        //-- Get the corners from the image_1 ( the object to be "detected" )
        std::vector<Point2f> obj_corners(4);
        obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
        obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); 
        obj_corners[3] = cvPoint( 0, img_object.rows );
        std::vector<Point2f> scene_corners(4);

        perspectiveTransform( obj_corners, scene_corners, H);

        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
        line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), 
              scene_corners[1] + Point2f( img_object.cols, 0), 
              Scalar(0, 255, 0), 4 );
        line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), 
              scene_corners[2] + Point2f( img_object.cols, 0), 
              Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), 
              scene_corners[3] + Point2f( img_object.cols, 0), 
              Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), 
              scene_corners[0] + Point2f( img_object.cols, 0), 
              Scalar( 0, 255, 0), 4 );

        float dis = 0.0;
        dis += norm( (scene_corners[0] + Point2f( img_object.cols, 0)) - 
                     (scene_corners[1] + Point2f( img_object.cols, 0)) );
        dis += norm( (scene_corners[1] + Point2f( img_object.cols, 0)) - 
                     (scene_corners[2] + Point2f( img_object.cols, 0)) );
        dis += norm( (scene_corners[2] + Point2f( img_object.cols, 0)) - 
                     (scene_corners[3] + Point2f( img_object.cols, 0)) );
        dis += norm( (scene_corners[3] + Point2f( img_object.cols, 0)) - 
                     (scene_corners[0] + Point2f( img_object.cols, 0)) );
     
        //-- Show detected matches

        if (dis > 500)
          cnt++;
        else
          cnt = 0;

        ros::param::param<std::string>("~directory", directory, default_dir);
        if (boost::filesystem::is_directory(directory) == false)
            boost::filesystem::create_directory(directory);
        std::string file_object = directory + "object_matched.jpg"; 
        if (cnt >= 5) {
            imwrite(file_object, frame);
            res.path_to_image = file_object; 
            return true;
        }

        imshow( "WindowName", img_matches );
        waitKey(1);

    }

    destroyAllWindows(); 

    return true;
}

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "object_detection_server");
    ros::NodeHandle nh;

    
    ros::Subscriber sub = nh.subscribe("/nav_kinect/rgb/image_color", 1, callback);
    ros::ServiceServer service = nh.advertiseService("object_detection_service", 
        callback_detection); 

    ros::Rate r(10);
    ros::Duration(2.0).sleep();

    ros::spin(); 
    
    return true; 
}
