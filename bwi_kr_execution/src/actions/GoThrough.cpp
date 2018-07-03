#include "GoThrough.h"

#include "ActionFactory.h"

#include "plan_execution/CurrentStateQuery.h"

#include <ros/console.h>
#include <ros/ros.h>

#include <algorithm>

using namespace std;

namespace bwi_krexec {

  
GoThrough::GoThrough():
              LogicalNavigation("gothrough"),
              failed(false){}


std::vector<std::string> GoThrough::getParameters() const {
  std::vector<std::string> parameters;
  parameters.push_back(door_name);
  return parameters;
}

bool GoThrough::hasFailed() const {return failed;}

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

ActionFactory gothroughFactory(new GoThrough());
  
  
}
