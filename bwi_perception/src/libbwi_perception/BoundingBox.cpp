#include <bwi_perception/BoundingBox.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <visualization_msgs/Marker.h>
#include "Eigen/Dense"


/* define what kind of point clouds we're using */
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;

namespace bwi_perception {
    BoundingBox::BoundingBox(const Eigen::Vector4f &min, const Eigen::Vector4f &max, const Eigen::Vector4f &centroid,
                             const Eigen::Quaternionf &orientation, const Eigen::Vector4f &position,
                             const std::string &frame_id) :
            min(min), max(max),
            centroid(centroid), position(position), orientation(orientation),
            frame_id(frame_id) {

    }

    visualization_msgs::Marker BoundingBox::to_marker(const int marker_index,
                                                      const std::string &ns) const {
        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = ns;
        marker.id = marker_index;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = visualization_msgs::Marker::CUBE;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

        marker.pose.position.x = position[0];
        marker.pose.position.y = position[1];
        marker.pose.position.z = position[2];
        marker.pose.orientation.x = (double) orientation.x();
        marker.pose.orientation.y = (double) orientation.y();
        marker.pose.orientation.z = (double) orientation.z();
        marker.pose.orientation.w = (double) orientation.w();

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = abs(max[0] - min[0]);
        marker.scale.y = abs(max[1] - min[1]);
        marker.scale.z = abs(max[2] - min[2]);

        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 0.25;

        marker.lifetime = ros::Duration();

        return marker;
    }

    void
    transform_eigen(const tf::TransformListener &listener, const std::string &target_frame, const Eigen::Vector4f &in,
                    const std::string &in_frame, Eigen::Vector4f &out) {
        geometry_msgs::PointStamped in_p;
        in_p.header.frame_id = in_frame;
        geometry_msgs::PointStamped out_p;

        in_p.point.x = in.x();
        in_p.point.y = in.y();
        in_p.point.z = in.z();
        listener.transformPoint(target_frame, in_p, out_p);
        out.x() = out_p.point.x;
        out.y() = out_p.point.y;
        out.z() = out_p.point.z;
    }

    void BoundingBox::transform(const std::string &target_frame, const BoundingBox &in, BoundingBox &out, const tf::TransformListener &listener){

        string in_frame = in.frame_id;
        listener.waitForTransform(target_frame, in_frame, ros::Time::now(), ros::Duration(3));
        transform_eigen(listener, target_frame, in.centroid, in_frame, out.centroid);
        transform_eigen(listener, target_frame, in.min, in_frame, out.min);
        transform_eigen(listener, target_frame, in.max, in_frame, out.max);
        transform_eigen(listener, target_frame, in.position, in_frame, out.position);
        tf::Stamped <tf::Quaternion> quat;
        quat.frame_id_ = in_frame;
        quat.setX(in.orientation.x());
        quat.setY(in.orientation.y());
        quat.setZ(in.orientation.z());
        quat.setW(in.orientation.w());
        listener.transformQuaternion(target_frame, quat, quat);
        out.orientation.x() = quat.x();
        out.orientation.y() = quat.y();
        out.orientation.z() = quat.z();
        out.orientation.w() = quat.w();

        out.frame_id = target_frame;

    }
}