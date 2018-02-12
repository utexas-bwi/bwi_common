#ifndef BWI_MANIPULATION_GRASPCARTESIANCOMMAND_H
#define BWI_MANIPULATION_GRASPCARTESIANCOMMAND_H

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <agile_grasp/Grasp.h>

namespace bwi_manipulation {
    struct GraspCartesianCommand {
    public:
        sensor_msgs::JointState approach_joint_state;
        geometry_msgs::PoseStamped approach_pose;

        sensor_msgs::JointState grasp_joint_state;
        geometry_msgs::PoseStamped grasp_pose;

        static GraspCartesianCommand
        from_agile_grasp(const agile_grasp::Grasp &grasp, float offset_approach,
                         float offset_grasp, const std::string &object_cloud_frameid);

        static GraspCartesianCommand
        from_grasp_pose(const geometry_msgs::PoseStamped &grasp_pose, const double offset_approach);
    };
}

#endif //BWI_MANIPULATION_GRASPCARTESIANCOMMAND_H
