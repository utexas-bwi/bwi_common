#ifndef BWI_PERCEPTION_TABLETOP_H
#define BWI_PERCEPTION_TABLETOP_H

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include "BoundingBox.h"
#include "convenience.h"

namespace bwi_perception {

    bool segment_tabletop_scene(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &in_cloud,
                                const double cluster_extraction_tolerance, const std::string &up_frame,
                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr &table_cloud, Eigen::Vector4f plane_coefficients,
                                std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &table_object_clouds,
                                const double plane_distance_tolerance, const double plane_max_distance_tolerance, double height,
                                const double min_cluster_size = 250, const double max_cluster_size = 25000);

}
#endif //BWI_PERCEPTION_TABLETOP_H
