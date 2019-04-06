#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>


#include "LongTermMemoryConduitInterface.h"
#include "ShortTermMemoryConduit.h"
#include <ros/ros.h>
#include <knowledge_representation/convenience.h>

namespace knowledge_rep {

    class MemoryConduit {
        typedef pcl::PointXYZRGB PointT;
        typedef pcl::PointCloud<PointT> PointCloudT;
        knowledge_rep::LongTermMemoryConduit ltmc;
        ShortTermMemoryConduit stmc;



        ros::NodeHandle _pnh;
        ros::ServiceClient get_octomap_service;
    public:
        static const int robot_id = 1;

        explicit MemoryConduit() : ltmc(knowledge_rep::get_default_ltmc()) {
            _pnh = ros::NodeHandle("~");
           

        }

        bool get_object_cloud(int id, PointCloudT::Ptr &cloud_out);

        bool encode(std::vector<PointCloudT::Ptr> &objects, PointCloudT::Ptr &table);

        std::vector<EntityAttribute> relevant_to(std::vector<int> objects);

       
    };

}

