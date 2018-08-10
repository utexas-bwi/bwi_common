#include "GoThrough.h"

#include "plan_execution/CurrentStateQuery.h"

#include <ros/console.h>
#include <ros/ros.h>

#include <algorithm>

using namespace std;

namespace bwi_krexec {

GoThrough::GoThrough(const std::string &door_name, knowledge_rep::LongTermMemoryConduit &ltmc) : door_name(door_name),
              LogicalNavigation("gothrough", ltmc){}


std::vector<std::string> GoThrough::getParameters() const {
  std::vector<std::string> parameters;
  parameters.push_back(door_name);
  return parameters;
}

std::vector<std::string> GoThrough::prepareGoalParameters() const {
  vector<string> params;
  params.push_back(door_name);
  return params;
}

void GoThrough::onFinished(bool success, ResultConstPtr result) {
  // Allow super to update the knowledge base
  LogicalNavigation::onFinished(success, result);
  // TODO: Check that we're in a different place than when we started
}

  
}
