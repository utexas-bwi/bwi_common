#pragma once
#include <ros/ros.h>
#include <knowledge_representation/ResolveObjectCorrespondences.h>
#include <knowledge_representation/AddObject.h>
#include <knowledge_representation/RemoveObject.h>

namespace knowledge_rep {

    class ShortTermMemoryConduit {
        ros::ServiceClient correspondences_service;
        ros::ServiceClient add_service;
        ros::ServiceClient remove_service;
    public:
        ShortTermMemoryConduit() {
            ros::NodeHandle nh = ros::NodeHandle();
            correspondences_service = nh.serviceClient<knowledge_representation::ResolveObjectCorrespondences>(
                    "/resolve_object_correspondences");
          add_service = nh.serviceClient<knowledge_representation::AddObject>("/add_instance");
            remove_service = nh.serviceClient<knowledge_representation::RemoveObject>("/remove_object");
        }

        std::vector<int>
        resolve_object_correspondences(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &objects,
                                       std::vector<int> candidates) {
            //correspondences_service.call(req, res);
        }


    };

}
