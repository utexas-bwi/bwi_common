#include "LogicalNavigation.h"

#include "actasp/AspFluent.h"
#include <plan_execution/AspFluent.h>
#include <knowledge_representation/LTMCEntity.h>
#include <knowledge_representation/LTMCInstance.h>
#include "../BwiResourceManager.h"

#include <ros/ros.h>

#include <sstream>

using namespace ros;
using namespace std;
using namespace actasp;
using namespace knowledge_rep;

namespace bwi_krexec {


LogicalNavigation::LogicalNavigation(const std::string &logical_name, knowledge_rep::LongTermMemoryConduit &ltmc) :
        name(logical_name), ltmc(ltmc),
        LogicalNavigationRosAction("execute_logical_action") {}

boost::optional<bwi_msgs::LogicalNavGoal> LogicalNavigation::prepareGoal() {
    auto params = prepareGoalParameters();
    if (params) {
        bwi_msgs::LogicalNavGoal goal;
        goal.command.name = name;
        goal.command.value = *params;
        return goal;
    }
    
}

void LogicalNavigation::onFinished(bool succeeded, const bwi_msgs::LogicalNavResult &result) {
    // Dump observations somewhere
    Instance self = ltmc.get().getRobot();
    self.removeAttribute("is_in");
    self.removeAttribute("is_near");
    self.removeAttribute("is_facing");

    ROS_INFO_STREAM(result.observations);

    if (!result.observations.room.empty()) {
        vector<Entity> rooms = ltmc.get().getEntitiesWithAttributeOfValue("name", result.observations.room);
        if (!rooms.empty()) {
            self.addAttribute("is_in", rooms[0]);
        }
    }
    for (int i = 0; i < result.observations.nearby_locations.size(); i++) {
        auto location_name = result.observations.nearby_locations.at(i);
        vector<Entity> locations = ltmc.get().getEntitiesWithAttributeOfValue("name", location_name);
        if (!locations.empty()) {
            self.addAttribute("is_near", locations[0]);
            if (result.observations.facing.at(i)) {
                self.addAttribute("is_facing", locations[0]);
            }
        } else {
            ROS_WARN_STREAM("Logical navigation state says robot is near " << location_name << " but no location with that map_name exists in the knowledge base.");
        }
    }

    if (!succeeded && name == "navigate_to") {

        //ROS_INFO_STREAM("Sent speech goal for unstuck");
        // TODO: Add speech here
    }
}

void LogicalNavigation::configureWithResources(ResourceManager &resource_manager) {
  auto& cast = dynamic_cast<BwiResourceManager&>(resource_manager);
  this->ltmc = cast.ltmc;
}


} //namespace
