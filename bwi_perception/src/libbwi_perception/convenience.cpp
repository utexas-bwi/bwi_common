
#include <bwi_perception/PerceiveTabletopScene.h>
#include <ros/ros.h>
#include <bwi_perception/PerceiveLargestHorizontalPlane.h>
#include <bwi_perception/convenience.h>

bwi_perception::PerceiveTabletopScene::Response bwi_perception::getTabletopScene(ros::NodeHandle &n) {

    ros::ServiceClient client_tabletop_perception = n.serviceClient<bwi_perception::PerceiveTabletopScene>(
            "perceive_tabletop_scene");

    bwi_perception::PerceiveTabletopScene srv;
    if (client_tabletop_perception.call(srv)) {
        return srv.response;
    } else {
        ROS_ERROR("Failed to call perceive_tabletop_scene service");
        return srv.response;
    }
}

bwi_perception::PerceiveLargestHorizontalPlane::Response
bwi_perception::perceiveLargestHorizontalPlane(ros::NodeHandle &n) {

    ros::ServiceClient client_tabletop_perception = n.serviceClient<bwi_perception::PerceiveLargestHorizontalPlane>(
            "perceive_largest_horizontal_plane");

    bwi_perception::PerceiveLargestHorizontalPlane srv;
    if (client_tabletop_perception.call(srv)) {
        return srv.response;
    } else {
        ROS_ERROR("Failed to call perceive_largest_horizontal_plane service");
        return srv.response;
    }
}

void bwi_perception::quaternion_to_frame(const tf::Stamped<tf::Quaternion> &quaternion, const std::string &target_frame,
                                         tf::Stamped<tf::Quaternion> &out_quaternion,
                                         tf::TransformListener &tf_listener) {
    if (quaternion.frame_id_ == target_frame) {
        out_quaternion = tf::Stamped<tf::Quaternion>(quaternion);
        return;
    }

    //transform into the target frame
    while (ros::ok()) {
        //keep trying until we get the transform
        try {
            tf_listener.transformQuaternion(target_frame, quaternion, out_quaternion);
            break;
        } catch (tf::TransformException ex) {
            ROS_ERROR_THROTTLE(1, "%s", ex.what());
            ROS_WARN_THROTTLE(1, "Waiting for transform from %s to %s", quaternion.frame_id_.c_str(),
                              target_frame.c_str());
            continue;
        }
    }
}

void bwi_perception::vector_to_frame(const geometry_msgs::Vector3Stamped &vector, const std::string &target_frame,
                                     geometry_msgs::Vector3Stamped &out_vector, tf::TransformListener &tf_listener) {
    if (vector.header.frame_id == target_frame) {
        out_vector = geometry_msgs::Vector3Stamped(vector);
        return;
    }

    //transform into the target frame
    while (ros::ok()) {
        //keep trying until we get the transform
        try {
            tf_listener.transformVector(target_frame, vector, out_vector);
            break;
        } catch (tf::TransformException ex) {
            ROS_ERROR_THROTTLE(1, "%s", ex.what());
            ROS_WARN_THROTTLE(1, "Waiting for transform from %s to %s", vector.header.frame_id.c_str(),
                              target_frame.c_str());
            continue;
        }
    }
}

void bwi_perception::vector_to_frame(const Eigen::Vector3f &vector, const std::string &frame,
                                     const std::string &target_frame, Eigen::Vector3f &out_vector,
                                     tf::TransformListener &tf_listener) {
    geometry_msgs::Vector3Stamped vector_geom;
    vector_geom.header.frame_id = frame;
    vector_geom.vector.x = vector.x();
    vector_geom.vector.y = vector.y();
    vector_geom.vector.z = vector.z();
    geometry_msgs::Vector3Stamped out_vector_geom;
    vector_to_frame(vector_geom, target_frame, out_vector_geom, tf_listener);
    out_vector.x() = out_vector_geom.vector.x;
    out_vector.y() = out_vector_geom.vector.y;
    out_vector.z() = out_vector_geom.vector.z;
}
