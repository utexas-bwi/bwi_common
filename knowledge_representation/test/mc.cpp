#include <knowledge_representation/LongTermMemoryConduitInterface.h>
#include <iostream>
#include <string>
#include <knowledge_representation/MemoryConduit.h>
#include <ros/ros.h>
#include <knowledge_representation/ShortTermMemoryConduit.h>

using namespace std;


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "test_mc");
    ros::NodeHandle nh("~");
    ros::Publisher facing_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("facing_cloud",1,true);
    knowledge_rep::MemoryConduit mc;

    //facing_cloud_pub.publish(mc.get_facing_cloud_ros());

    // Spin to keep the message available. Ctrl-C when done inspecting
    ros::spin();

}

