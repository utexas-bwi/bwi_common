#ifndef BWI_LOGICAL_TRANSLATOR_BWI_LOGICAL_NAVIGATOR_H
#define BWI_LOGICAL_TRANSLATOR_BWI_LOGICAL_NAVIGATOR_H

#include <bwi_msgs/ResolveChangeFloor.h>
#include <bwi_msgs/LogicalNavAction.h>
#include <bwi_msgs/UpdateObject.h>
#include <bwi_msgs/LogicalLocation.h>
#include <bwi_msgs/CheckBool.h>
#include <bwi_logical_translator/bwi_logical_translator.h>
#include <actionlib/server/simple_action_server.h>
#include <nav_msgs/Odometry.h>
#include <multi_level_map_msgs/LevelMetaData.h>
#include <multi_level_map_msgs/MultiLevelMapData.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <message_filters/subscriber.h>
#include <actionlib/client/simple_action_client.h>

using bwi_planning_common::PlannerAtom;
using bwi_planning_common::NO_DOOR_IDX;

class BwiLogicalNavigator : public bwi_logical_translator::BwiLogicalTranslator {

public:

    typedef actionlib::SimpleActionServer<bwi_msgs::LogicalNavAction> LogicalNavActionServer;

    BwiLogicalNavigator();

    void execute(const bwi_msgs::LogicalNavGoalConstPtr &goal);
    bool changeFloorResolutionHandler(bwi_msgs::ResolveChangeFloor::Request &req,
                                      bwi_msgs::ResolveChangeFloor::Response &res);
    bool updateObject(bwi_msgs::UpdateObject::Request &req,
                      bwi_msgs::UpdateObject::Response &res);

protected:

    void senseState(bwi_msgs::LogicalLocation &observations);
    bool approachDoor(const std::string& door_name,
                      bwi_msgs::LogicalLocation &observations,
                      std::string& error_message);
    bool senseDoor(bwi_msgs::CheckBool::Request &req,
                   bwi_msgs::CheckBool::Response &res);

    bool approachObject(const std::string& object_name,
                        bwi_msgs::LogicalLocation &observations,
                        std::string &error_message);

    bool resolveChangeFloorRequest(const std::string& new_room,
                                   const std::string& facing_door,
                                   std::string& floor_name,
                                   geometry_msgs::PoseWithCovarianceStamped& pose,
                                   std::string& error_message);

    bool changeFloor(const std::string& new_room,
                     const std::string& facing_door,
                     bwi_msgs::LogicalLocation &observations,
                     std::string& error_message);

    bool executeNavigationGoal(const geometry_msgs::PoseStamped& pose);
    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom);

    void currentLevelHandler(const multi_level_map_msgs::LevelMetaData::ConstPtr& current_level);
    void multimapHandler(const multi_level_map_msgs::MultiLevelMapData::ConstPtr& multimap);

    void publishNavigationMap(bool publish_map_with_doors = false,
                              bool wait_for_costmap_change = false);

    float robot_x_;
    float robot_y_;
    float robot_yaw_;
    std::string current_level_id_;

    double door_proximity_distance_;
    double location_proximity_distance_;

    boost::shared_ptr<LogicalNavActionServer> execute_action_server_;
    bool execute_action_server_started_;

    ros::ServiceServer change_floor_resolution_server_;
    ros::ServiceServer add_object_server_;
    ros::ServiceServer sense_door_server_;

    boost::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> > robot_controller_;

    boost::shared_ptr<tf::TransformListener> tf_;
    boost::shared_ptr<tf::MessageFilter<nav_msgs::Odometry> > tf_filter_;
    boost::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry> > odom_subscriber_;

    ros::Subscriber current_level_subscriber_;
    ros::Subscriber multimap_subscriber_;
    ros::ServiceClient change_level_client_;
    bool change_level_client_available_;
    std::vector<multi_level_map_msgs::LevelMetaData> all_levels_;
    std::map<std::string, std::vector<bwi_planning_common::Door> > level_to_doors_map_;
    std::map<std::string, std::vector<std::string> > level_to_loc_names_map_;
    std::map<std::string, std::vector<int32_t> > level_to_loc_map_;

    ros::Publisher navigation_map_publisher_;
    bool last_map_published_with_doors_;

    // Subscribe to costmap and costmap updates message to ensure that a published navigation map has been accepted.
    void costmapSubscriber(const nav_msgs::OccupancyGrid::ConstPtr& costmap);
    void costmapUpdatesSubscriber(const map_msgs::OccupancyGridUpdate::ConstPtr& costmap_updates);
    ros::Subscriber costmap_subscriber_;
    ros::Subscriber costmap_updates_subscriber_;
    bool full_global_costmap_update_;
    int global_costmap_width_;

    bool robot_controller_available_;

    bool goThroughDoor(const std::string &door_name, bwi_msgs::LogicalLocation &observations, std::string &error_message);

    bool navigateTo(const std::string &location_name, bwi_msgs::LogicalLocation observations, std::string &status);
};

#endif //BWI_LOGICAL_TRANSLATOR_BWI_LOGICAL_NAVIGATOR_H
