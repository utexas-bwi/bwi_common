#ifndef ARM_POSITION_DB_H
#define ARM_POSITION_DB_H
#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <stdio.h>
#include <std_msgs/String.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

namespace bwi_manipulation {
    class ArmPositionDB {

    private:
        std::map<std::string, std::vector<float> > j_positions;
        std::map<std::string, geometry_msgs::Pose> t_positions;

    public:

        bool has_joint_position(const std::string &name);

        bool has_tool_position(const std::string &name);

        geometry_msgs::PoseStamped get_tool_position_stamped(
                const std::string &name,
                const std::string &frame_id);

        geometry_msgs::Pose get_tool_position(const std::string &name);

        std::vector<float> get_joint_position(const std::string &name);

        void print();

        ArmPositionDB(const std::string &joint_positions_filename,
                      const std::string &tool_positions_filename);


    };
}



#endif
