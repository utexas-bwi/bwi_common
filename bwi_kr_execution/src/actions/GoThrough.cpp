#include "GoThrough.h"

#include <knowledge_representation/convenience.h>

#include <ros/ros.h>

#include <algorithm>

using namespace std;

namespace bwi_krexec {

GoThrough::GoThrough(int door_id, knowledge_rep::LongTermMemoryConduit &ltmc) : door_id(door_id),
              LogicalNavigation("go_through", ltmc){}


std::vector<std::string> GoThrough::getParameters() const {
  std::vector<std::string> parameters;
  parameters.push_back(to_string(door_id));
  return parameters;
}

boost::optional<std::vector<std::string> > GoThrough::prepareGoalParameters() const {

  knowledge_rep::Entity location(door_id, ltmc);
  if (!location.is_valid()) {
    return boost::none;
  }
  auto attrs = location.get_attributes("name");
  
  if (attrs.size() != 1) {
    return boost::none;
  }

  std::string door_name = attrs.at(0).get_string_value();

  vector<string> params;
  params.push_back(door_name);
  return params;
}

void GoThrough::onFinished(bool success, const bwi_msgs::LogicalNavResult &result) {
  // Allow super to update the knowledge base
  LogicalNavigation::onFinished(success, result);
  // TODO: Check that we're in a different place than when we started
}

  
}
