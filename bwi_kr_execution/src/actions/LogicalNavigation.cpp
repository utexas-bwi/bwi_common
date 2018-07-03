#include "LogicalNavigation.h"

#include "actasp/AspFluent.h"
#include <plan_execution/AspFluent.h>
#include <knowledge_representation/convenience.h>

#include <ros/ros.h>

#include <sstream>

using namespace ros;
using namespace std;
using namespace actasp;

namespace bwi_krexec {


LogicalNavigation::LogicalNavigation(const std::string &logical_name) :
        name(logical_name), ltmc(knowledge_rep::get_default_ltmc()),
        LogicalNavigationRosAction("execute_logical_action") {}

boost::optional<bwi_msgs::LogicalNavGoal> LogicalNavigation::prepareGoal() {
    bwi_msgs::LogicalNavGoal goal;
    goal.command.name = name;
    goal.command.value = prepareGoalParameters();
    return goal;
}

void LogicalNavigation::onFinished(bool succeeded, ResultConstPtr result) {
    // Dump observations somewhere
    ltmc.remove_entity_attribute(1, "is_in");
    ltmc.remove_entity_attribute(1, "is_located");
    ltmc.remove_entity_attribute(1, "is_facing");

    ROS_INFO_STREAM(result->observations);

    if (!result->observations.room.empty()) {
        vector<int> room_ids = ltmc.get_entities_with_attribute_of_value("map_name", result->observations.room);
        if (!room_ids.empty()) {
            ltmc.add_entity_attribute(1, "is_in", room_ids[0]);
        }
    }
    if (!result->observations.location.empty()) {
        vector<int> location_ids = ltmc.get_entities_with_attribute_of_value("map_name", result->observations.location);
        if (!location_ids.empty()) {
            ltmc.add_entity_attribute(1, "is_located", location_ids[0]);
            if (result->observations.facing) {
                ltmc.add_entity_attribute(1, "is_facing", location_ids[0]);
            }
        }

    }

    if (!succeeded && name == "navigate_to") {

        ROS_INFO_STREAM("Sent speech goal for unstuck");
        // TODO: Add speech here
    }
}


} //namespace
