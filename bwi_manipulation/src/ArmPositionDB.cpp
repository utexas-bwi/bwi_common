#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include "bwi_manipulation/ArmPositionDB.h"

#define NUM_JOINTS_ARMONLY 6

using namespace std;
namespace bwi_manipulation {

    ArmPositionDB::ArmPositionDB(const string &joint_positions_filename,
                                 const string &tool_positions_filename) {


        FILE *fp_j = fopen(joint_positions_filename.c_str(), "r");

        //the first integer is the # of entries in the file
        int num_entries_j = 0;
        fscanf(fp_j, "%i\n", &num_entries_j);

        for (int i = 0; i < num_entries_j; i++) {
            char name_i[80];
            vector<float> p_i(NUM_JOINTS_ARMONLY);

            int count = fscanf(fp_j, "%s\t%f,%f,%f,%f,%f,%f\n", name_i, &p_i[0], &p_i[1], &p_i[2], &p_i[3], &p_i[4], &p_i[5]);

            if (count != NUM_JOINTS_ARMONLY) {
                ROS_ERROR("Read only %f values for entry index %d. Database will be malformed.", count, i);
            }

            string name_i_s(name_i);
            j_positions.emplace(name_i_s, p_i);
        }

        //load cartesean positions
        FILE *fp_c = fopen(tool_positions_filename.c_str(), "r");

        //the first integer is the # of entries in the file
        int num_entries_c = 0;
        fscanf(fp_c, "%i\n", &num_entries_c);

        for (int i = 0; i < num_entries_c; i++) {
            char name_i[80];
            vector<float> p_i(7);

            int count = fscanf(fp_c, "%s\t%f,%f,%f,%f,%f,%f,%f\n", name_i, &p_i[0], &p_i[1], &p_i[2], &p_i[3], &p_i[4], &p_i[5],
                   &p_i[6]);

            if (count != 7) {
                ROS_ERROR("Read only %f values for entry index %d. Database will be malformed.", count, i);
            }
            geometry_msgs::Pose pose_i;
            pose_i.position.x = p_i[0];
            pose_i.position.y = p_i[1];
            pose_i.position.z = p_i[2];


            tf::Quaternion quat;
            quat.setX(p_i[3]);
            quat.setY(p_i[4]);
            quat.setZ(p_i[5]);
            quat.setW(p_i[6]);
            quat.normalize();
            tf::quaternionTFToMsg(quat, pose_i.orientation);

            string name_i_s(name_i);
            t_positions.emplace(name_i_s, pose_i);
        }

    }
    bool ArmPositionDB::has_joint_position(const string &name) {
        return j_positions.count(name) != 0;
    }

    bool ArmPositionDB::has_tool_position(const string &name) {
        return t_positions.count(name) != 0;
    }

    vector<float> ArmPositionDB::get_joint_position(const string &name) {
        return j_positions[name];
    }

    geometry_msgs::Pose ArmPositionDB::get_tool_position(const string &name) {
        return t_positions[name];
    }

    geometry_msgs::PoseStamped
    ArmPositionDB::get_tool_position_stamped(const string &name, const string &frame_id) {
        geometry_msgs::PoseStamped target_pose;
        target_pose.header.stamp = ros::Time::now();
        target_pose.header.frame_id = frame_id;
        target_pose.pose = t_positions[name];
        return target_pose;
    }

    void ArmPositionDB::print() {
        cout << "# of joint-space positions: " << j_positions.size() << "\n";

        for (const auto &pair: j_positions) {
            cout << "\tname:" << pair.first << "\t";

            for (unsigned int j = 0; j < NUM_JOINTS_ARMONLY; j++) {
                cout << pair.second[j];
                if (j < NUM_JOINTS_ARMONLY - 1)
                    cout << ",";
                else cout << "\n";

            }
        }

        cout << "\n# of tool-space positions: " << t_positions.size() << "\n";

        for (const auto &pair: t_positions) {
            cout << "\tname:" << pair.first << "\t";
            ROS_INFO_STREAM(pair.second);
        }
    }


}
